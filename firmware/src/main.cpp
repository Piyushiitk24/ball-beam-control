#include <Arduino.h>
#include <Wire.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "app/state_machine.h"
#include "calibration.h"
#include "calibration_runtime.h"
#include "config.h"
#include "control/cascade_controller.h"
#include "hal/stepper_tmc2209.h"
#include "pins.h"
#include "sensors/as5600_sensor.h"
#include "sensors/hcsr04_sensor.h"
#include "types.h"

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

namespace {

using namespace bb;

constexpr float kLimitStopMarginDeg = 0.2f;
constexpr float kLimitMinSpanDeg = 2.0f;

StepperTMC2209 g_stepper(PIN_STEP, PIN_DIR, PIN_EN);
AS5600Sensor g_as5600;
HCSR04Sensor g_hcsr04(PIN_TRIG, PIN_ECHO);
CascadeController g_controller;
StateMachine g_state_machine;

SensorData g_sensor;
Setpoint g_setpoint;
FaultFlags g_fault_flags;

bool g_inner_loop_stable = true;
bool g_manual_enable = false;
bool g_telemetry_enabled = true;
uint8_t g_angle_src = 0;  // 0=AS5600, 1=STEPPER (step-count angle)

// AS5600 runtime filter state (EMA + jump reject). Reset whenever calibration changes mapping.
bool g_as5600_ema_initialized = false;
bool g_as5600_last_initialized = false;
float g_as5600_theta_ema_deg = 0.0f;
float g_as5600_theta_last_deg = 0.0f;

uint32_t g_last_control_ms = 0;
uint32_t g_last_telemetry_ms = 0;
uint32_t g_last_valid_angle_ms = 0;
uint32_t g_last_valid_pos_ms = 0;
uint32_t g_last_angle_read_ms = 0;
uint32_t g_last_angle_read_dt_ms = 0;

volatile uint8_t* g_echo_pin_reg = nullptr;
uint8_t g_echo_pin_mask = 0;

char g_cmd_buffer[96];
size_t g_cmd_len = 0;

struct SignCalSession {
  bool active = false;
  bool near_captured = false;
  bool complete = false;

  float near_distance_cm = 0.0f;
  float far_distance_cm = 0.0f;

  float theta_before_raw_deg = 0.0f;
  float theta_after_raw_deg = 0.0f;
  float theta_delta_raw_deg = 0.0f;

  int8_t suggested_stepper_sign = STEPPER_DIR_SIGN;
  int8_t suggested_as5600_sign = AS5600_SIGN;
  int8_t suggested_sonar_sign = SONAR_POS_SIGN;
};

SignCalSession g_sign_cal;
bool g_runtime_cal_dirty = false;

void printFaultInfo();
void printSonarDiag();
void printAs5600Diag();
bool captureSonarCalDistanceCm(float& out_cm);
bool readAs5600RawMedianDeg(float& out_raw_deg);
bool readAs5600RawMedianCounts(uint16_t& out_counts);
bool captureAs5600CalRawDeg(float& out_raw_deg);
void serviceControl(uint32_t now_ms);

const char* stateToString(AppState state) {
  switch (state) {
    case AppState::SAFE_DISABLED:
      return "SAFE_DISABLED";
    case AppState::CALIB_SIGN:
      return "CALIB_SIGN";
    case AppState::CALIB_SCALE:
      return "CALIB_SCALE";
    case AppState::READY:
      return "READY";
    case AppState::RUNNING:
      return "RUNNING";
    case AppState::FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

float clampf(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

void resetAs5600Filter() {
  g_as5600_ema_initialized = false;
  g_as5600_last_initialized = false;
  g_as5600_theta_ema_deg = 0.0f;
  g_as5600_theta_last_deg = 0.0f;
}

// Tiny float parser to avoid pulling in strtod()/atof() (saves significant flash).
// Supported: optional whitespace, optional +/- sign, digits, optional .digits.
// Not supported: exponent notation.
float parseFloatFast(const char* s) {
  if (s == nullptr) {
    return 0.0f;
  }

  // Skip leading spaces/tabs.
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') {
    ++s;
  }

  float sign = 1.0f;
  if (*s == '-') {
    sign = -1.0f;
    ++s;
  } else if (*s == '+') {
    ++s;
  }

  bool any = false;
  uint32_t int_part = 0;
  while (*s >= '0' && *s <= '9') {
    any = true;
    int_part = (int_part * 10u) + static_cast<uint32_t>(*s - '0');
    ++s;
  }

  uint32_t frac_part = 0;
  uint32_t frac_div = 1;
  if (*s == '.') {
    ++s;
    // Limit fractional digits to keep code small and bounded.
    for (uint8_t i = 0; i < 4; ++i) {
      if (*s < '0' || *s > '9') {
        break;
      }
      any = true;
      frac_part = (frac_part * 10u) + static_cast<uint32_t>(*s - '0');
      frac_div *= 10u;
      ++s;
    }
  }

  if (!any) {
    return 0.0f;
  }

  const float frac = (frac_div > 1u) ? (static_cast<float>(frac_part) / static_cast<float>(frac_div)) : 0.0f;
  return sign * (static_cast<float>(int_part) + frac);
}

uint32_t activeStaleThresholdMs() {
  return (g_state_machine.state() == AppState::RUNNING)
             ? kSensorInvalidFaultMsRunning
             : kSensorInvalidFaultMsBringup;
}

bool hasAnyFault() {
  return g_fault_flags.sonar_timeout || g_fault_flags.i2c_error ||
         g_fault_flags.angle_oob || g_fault_flags.pos_oob;
}

void applySafeDisable() {
  g_stepper.stop();
  g_stepper.enable(false);
}

void setupEchoPcint() {
  g_echo_pin_reg = portInputRegister(digitalPinToPort(PIN_ECHO));
  g_echo_pin_mask = digitalPinToBitMask(PIN_ECHO);

  volatile uint8_t* pcicr = digitalPinToPCICR(PIN_ECHO);
  volatile uint8_t* pcmsk = digitalPinToPCMSK(PIN_ECHO);

  if (pcicr == nullptr || pcmsk == nullptr) {
    return;
  }

  *pcicr |= _BV(digitalPinToPCICRbit(PIN_ECHO));
  *pcmsk |= _BV(digitalPinToPCMSKbit(PIN_ECHO));
}

bool sampleAngleNow(uint32_t now_ms,
                    float* theta_deg_out = nullptr,
                    float* raw_deg_out = nullptr) {
  float theta_deg = 0.0f;
  float theta_raw_deg = 0.0f;
  if (!readAs5600RawMedianDeg(theta_raw_deg)) {
    return false;
  }
  theta_deg = runtimeMapThetaDeg(theta_raw_deg);

  // Glitch/jump handling (common with noisy I2C, loose magnets, or mechanical wobble):
  // Slew-limit the delta rather than rejecting the sample. Rejecting makes the angle go "stale"
  // on a single spike, which then cascades into faults/driver-disable.
  if (g_as5600_last_initialized) {
    float delta = wrapAngleDeltaDeg(theta_deg - g_as5600_theta_last_deg);
    if (fabsf(delta) > kAs5600MaxJumpDeg) {
      delta = (delta >= 0.0f) ? kAs5600MaxJumpDeg : -kAs5600MaxJumpDeg;
      theta_deg = g_as5600_theta_last_deg + delta;
    }
  }
  g_as5600_theta_last_deg = theta_deg;
  g_as5600_last_initialized = true;

  // EMA smoothing on mapped theta (used by control).
  if (!g_as5600_ema_initialized) {
    g_as5600_theta_ema_deg = theta_deg;
    g_as5600_ema_initialized = true;
  } else {
    g_as5600_theta_ema_deg =
        (kAs5600EmaAlpha * theta_deg) + ((1.0f - kAs5600EmaAlpha) * g_as5600_theta_ema_deg);
  }

  theta_deg = g_as5600_theta_ema_deg;

  g_sensor.beam_angle_deg = theta_deg;
  g_sensor.beam_angle_rad = theta_deg * kDegToRad;
  g_sensor.beam_angle_raw_deg = theta_raw_deg;
  if (g_last_angle_read_ms != 0) {
    g_last_angle_read_dt_ms = static_cast<uint32_t>(now_ms - g_last_angle_read_ms);
  }
  g_last_angle_read_ms = now_ms;
  g_last_valid_angle_ms = now_ms;
  g_fault_flags.i2c_error = false;
  // Angle safety:
  // Use calibrated limits (mechanical travel). Before limits calibration, don't fault on angle.
  constexpr float kAngleOobMarginDeg = 1.0f;
  if (runtimeCalIsLimitsSet()) {
    g_fault_flags.angle_oob =
        (theta_deg < (runtimeCalThetaLowerLimitDeg() - kAngleOobMarginDeg)) ||
        (theta_deg > (runtimeCalThetaUpperLimitDeg() + kAngleOobMarginDeg));
  } else {
    g_fault_flags.angle_oob = false;
  }

  if (theta_deg_out != nullptr) {
    *theta_deg_out = theta_deg;
  }
  if (raw_deg_out != nullptr) {
    *raw_deg_out = theta_raw_deg;
  }
  return true;
}

bool sampleSonarNow(uint32_t now_ms,
                    float* x_cm_out = nullptr,
                    float* x_filt_cm_out = nullptr,
                    float* dist_cm_out = nullptr) {
  float x_cm = 0.0f;
  float x_filt_cm = 0.0f;
  float dist_cm = 0.0f;
  const bool has_pos_sample = g_hcsr04.getPosition(x_cm, x_filt_cm, dist_cm);
  if (!has_pos_sample) {
    g_sensor.valid_pos = false;
    g_fault_flags.pos_oob = false;
    return false;
  }

  g_sensor.ball_pos_cm = x_cm;
  g_sensor.ball_pos_m = x_cm * kCmToM;
  g_sensor.ball_pos_filt_cm = x_filt_cm;
  g_sensor.ball_pos_filt_m = x_filt_cm * kCmToM;
  g_sensor.sonar_distance_raw_cm = dist_cm;

  const bool fresh = g_hcsr04.hasFreshSample(now_ms);
  g_sensor.valid_pos = fresh;

  if (g_sensor.valid_pos) {
    // Track time of the last accepted sonar sample (not "last time we checked"),
    // so stale/fault timing is consistent even when we hold last-good on timeouts.
    g_last_valid_pos_ms = static_cast<uint32_t>(now_ms - g_hcsr04.sampleAgeMs(now_ms));
    g_fault_flags.sonar_timeout = false;
    g_fault_flags.pos_oob = fabsf(g_sensor.ball_pos_filt_m) > kBallPosHardLimitM;
  } else {
    g_fault_flags.pos_oob = false;
  }

  if (x_cm_out != nullptr) {
    *x_cm_out = x_cm;
  }
  if (x_filt_cm_out != nullptr) {
    *x_filt_cm_out = x_filt_cm;
  }
  if (dist_cm_out != nullptr) {
    *dist_cm_out = dist_cm;
  }

  return g_sensor.valid_pos;
}

bool readSensors(uint32_t now_ms) {
  g_sensor.ts_ms = now_ms;
  if (g_angle_src != 0) {
    const int32_t steps = g_stepper.positionSteps();
    const float theta_deg = static_cast<float>(steps) * kStepperDegPerStep;

    g_sensor.beam_angle_deg = theta_deg;
    g_sensor.beam_angle_rad = theta_deg * kDegToRad;
    g_sensor.beam_angle_raw_deg = 0.0f;
    if (g_last_angle_read_ms != 0) {
      g_last_angle_read_dt_ms = static_cast<uint32_t>(now_ms - g_last_angle_read_ms);
    }
    g_last_angle_read_ms = now_ms;
    g_last_valid_angle_ms = now_ms;

    g_sensor.valid_angle = true;
    g_fault_flags.i2c_error = false;
    const int32_t abs_steps = (steps >= 0) ? steps : -steps;
    g_fault_flags.angle_oob = abs_steps > (kStepperPosLimitSteps + kStepperPosLimitMarginSteps);
  } else {
    sampleAngleNow(now_ms);
    // Hold last-good AS5600 value until it becomes stale (matches sonar policy).
    g_sensor.valid_angle =
        g_as5600_last_initialized &&
        (static_cast<uint32_t>(now_ms - g_last_valid_angle_ms) <= kPosSampleFreshMs);
  }

  sampleSonarNow(now_ms);
  return g_sensor.valid_angle && g_sensor.valid_pos;
}

void updateFaultState(uint32_t now_ms) {
  const uint32_t stale_threshold_ms = activeStaleThresholdMs();

  if (g_angle_src == 0) {
    if (static_cast<uint32_t>(now_ms - g_last_valid_angle_ms) > stale_threshold_ms) {
      g_fault_flags.i2c_error = true;
    }
  } else {
    g_fault_flags.i2c_error = false;
  }

  if (static_cast<uint32_t>(now_ms - g_last_valid_pos_ms) > stale_threshold_ms) {
    g_fault_flags.sonar_timeout = true;
  }

  if (g_state_machine.state() == AppState::RUNNING && hasAnyFault()) {
    g_state_machine.requestFault();
    applySafeDisable();
  }
}

void effectiveThetaCmdBoundsRad(float& theta_min_rad, float& theta_max_rad) {
  float lower_deg = -kThetaCmdLimitDeg;
  float upper_deg = kThetaCmdLimitDeg;

  if (runtimeCalIsLimitsSet()) {
    if (runtimeCalThetaLowerLimitDeg() > lower_deg) {
      lower_deg = runtimeCalThetaLowerLimitDeg();
    }
    if (runtimeCalThetaUpperLimitDeg() < upper_deg) {
      upper_deg = runtimeCalThetaUpperLimitDeg();
    }
  }

  lower_deg = clampf(lower_deg, -kThetaHardLimitDeg, kThetaHardLimitDeg);
  upper_deg = clampf(upper_deg, -kThetaHardLimitDeg, kThetaHardLimitDeg);

  if (lower_deg > upper_deg) {
    lower_deg = -kThetaCmdLimitDeg;
    upper_deg = kThetaCmdLimitDeg;
  }

  theta_min_rad = lower_deg * kDegToRad;
  theta_max_rad = upper_deg * kDegToRad;
}

void maybeStopJogAtLimits() {
  const float rate = g_stepper.targetSignedStepRate();
  if (rate == 0.0f) {
    return;
  }

  if (g_angle_src != 0) {
    const int32_t steps = g_stepper.positionSteps();
    const float hw_dir_ref = rate * static_cast<float>(runtimeCalStepperDirSign());
    if (hw_dir_ref >= 0.0f) {
      if (steps >= kStepperPosLimitSteps) {
        g_stepper.stop();
        Serial.println(F("WARN,jog_stopped_upper_limit"));
      }
      return;
    }
    if (steps <= -kStepperPosLimitSteps) {
      g_stepper.stop();
      Serial.println(F("WARN,jog_stopped_lower_limit"));
    }
    return;
  }

  if (!runtimeCalIsLimitsSet() || !g_sensor.valid_angle) {
    return;
  }

  const float theta_deg = g_sensor.beam_angle_deg;

  if (rate > 0.0f && theta_deg >= (runtimeCalThetaUpperLimitDeg() - kLimitStopMarginDeg)) {
    g_stepper.stop();
    Serial.println(F("WARN,jog_stopped_upper_limit"));
    return;
  }

  if (rate < 0.0f && theta_deg <= (runtimeCalThetaLowerLimitDeg() + kLimitStopMarginDeg)) {
    g_stepper.stop();
    Serial.println(F("WARN,jog_stopped_lower_limit"));
  }
}

void printHelp() {
  Serial.println(F("HELP,keys,stepper:y p e1 r"));
}

void printStatus() {
  const uint32_t now_ms = millis();

  Serial.print(F("STATE,"));
  Serial.println(stateToString(g_state_machine.state()));

  Serial.print(F("SENSORS,angle="));
  Serial.print(g_sensor.valid_angle ? F("ok") : F("bad"));
  Serial.print(F(",pos="));
  Serial.println(g_sensor.valid_pos ? F("ok") : F("bad"));

  Serial.print(F("MEAS,theta_deg="));
  Serial.print(g_sensor.beam_angle_deg, 4);
  Serial.print(F(",x_cm="));
  Serial.print(g_sensor.ball_pos_cm, 4);
  Serial.print(F(",x_filt_cm="));
  Serial.println(g_sensor.ball_pos_filt_cm, 4);

  Serial.print(F("AGE,angle_ms="));
  Serial.print(static_cast<uint32_t>(now_ms - g_last_valid_angle_ms));
  Serial.print(F(",pos_ms="));
  Serial.println(static_cast<uint32_t>(now_ms - g_last_valid_pos_ms));

  Serial.print(F("FAULTS,bits="));
  Serial.println(packFaultFlags(g_fault_flags));

  Serial.print(F("CAL,zero_calibrated="));
  Serial.print(runtimeCalIsZeroSet() ? F("yes") : F("no"));
  Serial.print(F(",limits_calibrated="));
  Serial.print(runtimeCalIsLimitsSet() ? F("yes") : F("no"));
  Serial.print(F(",sign_calibrated="));
  Serial.println(runtimeCalIsSignSet() ? F("yes") : F("no"));

  Serial.print(F("CAL_STORE,status="));
  Serial.println(runtimeCalLoadStatusName(runtimeCalLoadStatus()));

  Serial.print(F("TEL,enabled="));
  Serial.println(g_telemetry_enabled ? F("1") : F("0"));
}

void emitTelemetry(uint32_t now_ms) {
  if (!g_telemetry_enabled) {
    return;
  }

  Serial.print(F("TEL,"));
  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(stateToString(g_state_machine.state()));
  Serial.print(',');
  Serial.print(g_sensor.ball_pos_cm, 4);
  Serial.print(',');
  Serial.print(g_sensor.ball_pos_filt_cm, 4);
  Serial.print(',');
  Serial.print(g_sensor.beam_angle_deg, 4);
  Serial.print(',');
  Serial.print(g_controller.lastThetaCmdDeg(), 4);
  Serial.print(',');
  Serial.print(g_stepper.targetSignedStepRate(), 2);
  Serial.print(',');
  Serial.println(packFaultFlags(g_fault_flags));
}

void runJogBlocking(uint32_t timeout_ms) {
  const uint32_t start_ms = millis();
  while (g_stepper.jogActive()) {
    const uint32_t now_us = micros();
    const uint32_t now_ms = millis();

    g_stepper.processIsrFlags();
    g_hcsr04.service(now_us, now_ms);
    // Enforce limit stops while blocking (used by sign calibration jog).
    serviceControl(now_ms);

    if (static_cast<uint32_t>(now_ms - start_ms) > timeout_ms) {
      g_stepper.stop();
      break;
    }
  }

  g_stepper.processIsrFlags();
}

bool handleCalSignBegin() {
  if (g_state_machine.state() == AppState::FAULT) {
    Serial.println(F("ERR,fault_active"));
    printFaultInfo();
    Serial.println(F("GUIDE,do=t->a->p->f->b"));
    return false;
  }

  float theta_raw_deg = 0.0f;
  if (!captureAs5600CalRawDeg(theta_raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
    printAs5600Diag();
    return false;
  }

  if (!g_state_machine.startSignCalibration()) {
    Serial.println(F("ERR,cannot_enter_calib_sign"));
    return false;
  }

  g_sign_cal.active = true;
  g_sign_cal.near_captured = true;
  g_sign_cal.complete = false;
  g_sign_cal.suggested_stepper_sign = runtimeCalStepperDirSign();
  g_sign_cal.suggested_as5600_sign = runtimeCalAs5600Sign();
  g_sign_cal.suggested_sonar_sign = runtimeCalSonarPosSign();

  float sonar_dist_cm = 0.0f;
  g_sign_cal.near_captured = captureSonarCalDistanceCm(sonar_dist_cm);
  g_sign_cal.near_distance_cm = sonar_dist_cm;
  g_sign_cal.theta_before_raw_deg = theta_raw_deg;

  g_stepper.enable(true);
  g_stepper.requestJogSteps(SIGN_CAL_JOG_STEPS, SIGN_CAL_JOG_RATE_SPS);
  runJogBlocking(3000);

  delay(50);
  if (!captureAs5600CalRawDeg(g_sign_cal.theta_after_raw_deg)) {
    Serial.println(F("ERR,angle_not_ready_after_jog"));
    g_state_machine.finishSignCalibration(false);
    return false;
  }
  g_sign_cal.theta_delta_raw_deg =
      wrapAngleDeltaDeg(g_sign_cal.theta_after_raw_deg - g_sign_cal.theta_before_raw_deg);

  g_sign_cal.suggested_stepper_sign =
      (g_sign_cal.theta_delta_raw_deg >= 0.0f) ? runtimeCalStepperDirSign()
                                               : static_cast<int8_t>(-runtimeCalStepperDirSign());

  g_sign_cal.suggested_as5600_sign = (g_sign_cal.theta_delta_raw_deg >= 0.0f) ? 1 : -1;

  runtimeCalSetStepperDirSign(g_sign_cal.suggested_stepper_sign);
  runtimeCalSetAs5600Sign(g_sign_cal.suggested_as5600_sign);
  runtimeCalSetSignCaptured(true);
  g_runtime_cal_dirty = true;
  resetAs5600Filter();

  g_state_machine.finishSignCalibration(true);

  if (g_sign_cal.near_captured) {
    Serial.println(F("INFO,sonar_near=1"));
  } else {
    Serial.println(F("WARN,sonar_near=0"));
  }

  if (!g_manual_enable) {
    applySafeDisable();
  }
  return true;
}

bool handleCalSignSave() {
  if (!g_sign_cal.active) {
    Serial.println(F("ERR,cal_sign_not_started"));
    return false;
  }
  if (!g_sign_cal.near_captured) {
    Serial.println(F("ERR,sign_sonar_near_missing"));
    Serial.println(F("GUIDE,do=sonar sign 1|-1 or rerun b"));
    return false;
  }

  float dist_cm = 0.0f;
  if (!captureSonarCalDistanceCm(dist_cm)) {
    Serial.println(F("ERR,sonar_not_ready"));
    printSonarDiag();
    return false;
  }

  g_sign_cal.far_distance_cm = dist_cm;
  g_sign_cal.suggested_sonar_sign =
      (g_sign_cal.far_distance_cm >= g_sign_cal.near_distance_cm) ? 1 : -1;

  g_sign_cal.complete = true;
  g_sign_cal.active = false;

  runtimeCalSetSonarPosSign(g_sign_cal.suggested_sonar_sign);
  if (g_state_machine.state() == AppState::CALIB_SIGN) {
    g_state_machine.finishSignCalibration(true);
  }

  Serial.print(F("SIGN_CAL_RESULT,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(g_sign_cal.theta_delta_raw_deg, 5);
  Serial.print(',');
  Serial.print(g_sign_cal.suggested_stepper_sign);
  Serial.print(',');
  Serial.print(g_sign_cal.suggested_as5600_sign);
  Serial.print(',');
  Serial.print(g_sign_cal.near_distance_cm, 4);
  Serial.print(',');
  Serial.print(g_sign_cal.far_distance_cm, 4);
  Serial.print(',');
  Serial.println(g_sign_cal.suggested_sonar_sign);

  Serial.println(F("INFO,signs_applied_runtime"));
  g_runtime_cal_dirty = true;
  return true;
}

AppConditions currentConditions() {
  AppConditions cond;
  if (g_angle_src != 0) {
    // Reference-style: bypass AS5600 calibration dependencies when using step counts.
    cond.sign_calibrated = true;
    cond.limits_calibrated = true;
    cond.zero_calibrated = runtimeCalIsZeroPosCaptured();
  } else {
    cond.sign_calibrated = runtimeCalIsSignSet();
    cond.zero_calibrated = runtimeCalIsZeroSet();
    cond.limits_calibrated = runtimeCalIsLimitsSet();
  }
  cond.sensors_ok = g_sensor.valid_angle && g_sensor.valid_pos;
  cond.faults_active = hasAnyFault();
  cond.inner_loop_stable = g_inner_loop_stable;
  return cond;
}

void printCalZero() {
  Serial.print(F("CAL_ZERO,as5600_zero_deg="));
  Serial.print(runtimeCalAs5600ZeroDeg(), 6);
  Serial.print(F(",sonar_center_cm="));
  Serial.print(runtimeCalSonarCenterCm(), 6);
  Serial.print(F(",zero_calibrated="));
  Serial.println(runtimeCalIsZeroSet() ? F("yes") : F("no"));
}

void printCalLimits() {
  Serial.print(F("CAL_LIMITS,lower_deg="));
  Serial.print(runtimeCalThetaLowerLimitDeg(), 6);
  Serial.print(F(",upper_deg="));
  Serial.print(runtimeCalThetaUpperLimitDeg(), 6);
  Serial.print(F(",limits_calibrated="));
  Serial.println(runtimeCalIsLimitsSet() ? F("yes") : F("no"));
}

void printFaultInfo() {
  const uint8_t bits = packFaultFlags(g_fault_flags);
  Serial.print(F("FAULT_INFO,bits="));
  Serial.println(bits);
  Serial.print(F("FAULT_INFO,mode="));
  Serial.println(g_state_machine.state() == AppState::RUNNING ? F("run_hard_fault") : F("bringup_warning"));
  if (bits == 0) {
    Serial.println(F("FAULT_INFO,none"));
    return;
  }
  if (g_fault_flags.sonar_timeout) {
    Serial.println(F("FAULT_INFO,sonar_timeout=1"));
  }
  if (g_fault_flags.i2c_error) {
    Serial.println(F("FAULT_INFO,i2c_error=1"));
  }
  if (g_fault_flags.angle_oob) {
    Serial.println(F("FAULT_INFO,angle_oob=1"));
  }
  if (g_fault_flags.pos_oob) {
    Serial.println(F("FAULT_INFO,pos_oob=1"));
  }
  if (g_state_machine.state() == AppState::FAULT) {
    Serial.println(F("FAULT_INFO,recover=f"));
  }
}

void printSonarDiag() {
  SonarDiag diag;
  g_hcsr04.getDiag(millis(), diag);
  Serial.print(F("SONAR_DIAG,has_sample="));
  Serial.print(diag.has_sample ? F("1") : F("0"));
  Serial.print(F(",fresh="));
  Serial.print(diag.fresh ? F("1") : F("0"));
  Serial.print(F(",timeout="));
  Serial.print(diag.timeout ? F("1") : F("0"));
  Serial.print(F(",age_ms="));
  Serial.print(diag.age_ms);
  Serial.print(F(",raw_cm="));
  Serial.print(diag.raw_cm, 4);
  Serial.print(F(",filt_cm="));
  Serial.print(diag.filt_cm, 4);
  Serial.print(F(",valid_streak="));
  Serial.print(diag.valid_streak);
  Serial.print(F(",timeout_cnt="));
  Serial.print(diag.timeout_count);
  Serial.print(F(",jump_reject_cnt="));
  Serial.println(diag.jump_reject_count);
}

void printAs5600Diag() {
  const uint32_t now_ms = millis();
  float theta_deg = g_sensor.beam_angle_deg;
  float raw_deg = g_sensor.beam_angle_raw_deg;
  const bool ok = sampleAngleNow(now_ms, &theta_deg, &raw_deg);
  float read_hz = 0.0f;
  if (g_last_angle_read_dt_ms > 0 && g_last_angle_read_dt_ms < 1000) {
    read_hz = 1000.0f / static_cast<float>(g_last_angle_read_dt_ms);
  }

  Serial.print(F("AS5600_DIAG,"));
  Serial.print(ok ? F("1") : F("0"));
  Serial.print(',');
  Serial.print(raw_deg, 4);
  Serial.print(',');
  Serial.print(theta_deg, 4);
  Serial.print(',');
  Serial.print(g_as5600.errorCount());
  Serial.print(',');
  Serial.println(read_hz, 2);

  const AS5600Status mag = g_as5600.readStatus();
  Serial.print(F("AS5600_MAG,ok="));
  Serial.print(mag.read_ok ? F("1") : F("0"));
  Serial.print(F(",md="));
  Serial.print(mag.magnet_detected ? F("1") : F("0"));
  Serial.print(F(",mh="));
  Serial.print(mag.too_strong ? F("1") : F("0"));
  Serial.print(F(",ml="));
  Serial.print(mag.too_weak ? F("1") : F("0"));
  Serial.print(F(",agc="));
  Serial.print(mag.agc);
  Serial.print(F(",mag="));
  Serial.println(mag.magnitude);
}

bool captureSonarCalDistanceCm(float& out_cm) {
  // Calibration needs a stable distance even when the target is a weak reflector.
  // Take multiple fresh filtered samples and smooth them again with EMA.
  constexpr uint8_t kNeedGood = 5;
  constexpr uint32_t kTimeoutMs = 900;
  constexpr uint32_t kGoodDelayMs = 45;
  constexpr uint32_t kBadDelayMs = 10;

  uint8_t good = 0;
  float ema = 0.0f;
  bool ema_init = false;

  const uint32_t start_ms = millis();
  while (good < kNeedGood && static_cast<uint32_t>(millis() - start_ms) < kTimeoutMs) {
    const uint32_t now_us = micros();
    const uint32_t now_ms = millis();

    g_stepper.processIsrFlags();
    g_hcsr04.service(now_us, now_ms);

    SonarDiag diag;
    g_hcsr04.getDiag(now_ms, diag);

    // Only accept true fresh samples (reject "held" values after a timeout).
    if (diag.fresh && diag.has_sample && !diag.timeout) {
      const float v = (diag.filt_cm > 0.0f) ? diag.filt_cm : diag.raw_cm;
      if (v > 0.0f) {
        if (!ema_init) {
          ema = v;
          ema_init = true;
        } else {
          ema = (kSonarEmaAlpha * v) + ((1.0f - kSonarEmaAlpha) * ema);
        }
        ++good;
        delay(kGoodDelayMs);
        continue;
      }
    }

    delay(kBadDelayMs);
  }

  if (!ema_init || good < kNeedGood) {
    return false;
  }
  out_cm = ema;
  return true;
}

bool readAs5600RawMedianCounts(uint16_t& out_counts) {
  // Median-of-3 raw samples with wrap handling at 0/4095 boundary.
  uint16_t a = 0, b = 0, c = 0;
  if (!g_as5600.readRaw(a)) {
    return false;
  }
  delayMicroseconds(1500);
  if (!g_as5600.readRaw(b)) {
    return false;
  }
  delayMicroseconds(1500);
  if (!g_as5600.readRaw(c)) {
    return false;
  }

  int16_t ai = static_cast<int16_t>(a);
  int16_t bi = static_cast<int16_t>(b);
  int16_t ci = static_cast<int16_t>(c);

  int16_t d = static_cast<int16_t>(bi - ai);
  if (d > 2048) { bi = static_cast<int16_t>(bi - 4096); }
  else if (d < -2048) { bi = static_cast<int16_t>(bi + 4096); }

  d = static_cast<int16_t>(ci - ai);
  if (d > 2048) { ci = static_cast<int16_t>(ci - 4096); }
  else if (d < -2048) { ci = static_cast<int16_t>(ci + 4096); }

  if (ai > bi) { int16_t t = ai; ai = bi; bi = t; }
  if (bi > ci) { int16_t t = bi; bi = ci; ci = t; }
  if (ai > bi) { int16_t t = ai; ai = bi; bi = t; }

  int16_t med = bi;
  if (med < 0) { med = static_cast<int16_t>(med + 4096); }
  else if (med >= 4096) { med = static_cast<int16_t>(med - 4096); }
  out_counts = static_cast<uint16_t>(med);
  return true;
}

bool readAs5600RawMedianDeg(float& out_raw_deg) {
  uint16_t counts = 0;
  if (!readAs5600RawMedianCounts(counts)) {
    return false;
  }
  out_raw_deg = (static_cast<float>(counts) * 360.0f) / 4096.0f;
  return true;
}

bool captureAs5600CalRawDeg(float& out_raw_deg) {
  // Calibration capture in raw counts domain to avoid 0/360° boundary corruption.
  // Uses unwrapped int32_t accumulator for EMA, then wraps back to [0°,360°) at end.
  constexpr uint8_t kNeedGood = AS5600_CAL_NEED_GOOD;
  constexpr uint32_t kTimeoutMs = AS5600_CAL_TIMEOUT_MS;
  constexpr uint8_t kDelayMs = 6;
  constexpr uint8_t kMaxConsecReject = 5;
  constexpr int16_t kMaxJumpCounts =
      static_cast<int16_t>(AS5600_CAL_MAX_JUMP_DEG * (4096.0f / 360.0f));

  uint8_t good = 0;
  int32_t ema_unwrap = 0;  // unwrapped counts, integer EMA
  bool ema_init = false;

  uint16_t last_counts = 0;
  int32_t last_unwrap = 0;
  bool last_init = false;
  uint8_t consec_reject = 0;

  const uint32_t start_ms = millis();
  while (good < kNeedGood && static_cast<uint32_t>(millis() - start_ms) < kTimeoutMs) {
    uint16_t counts = 0;
    if (readAs5600RawMedianCounts(counts)) {
      // Unwrap: compute shortest-path delta in counts domain.
      int16_t delta = static_cast<int16_t>(counts) - static_cast<int16_t>(last_counts);
      if (delta > 2048) { delta = static_cast<int16_t>(delta - 4096); }
      else if (delta < -2048) { delta = static_cast<int16_t>(delta + 4096); }

      if (last_init) {
        if (abs(delta) > kMaxJumpCounts) {
          ++consec_reject;
          if (consec_reject >= kMaxConsecReject) {
            last_counts = counts;
            last_unwrap = last_unwrap + static_cast<int32_t>(delta);
            consec_reject = 0;
          }
          delay(kDelayMs);
          continue;
        }
      }

      int32_t unwrap = last_init ? (last_unwrap + static_cast<int32_t>(delta)) : static_cast<int32_t>(counts);
      last_counts = counts;
      last_unwrap = unwrap;
      last_init = true;
      consec_reject = 0;

      // Integer EMA: ema = alpha * sample + (1-alpha) * ema, using fixed-point (alpha=0.3 ≈ 77/256).
      if (!ema_init) {
        ema_unwrap = unwrap;
        ema_init = true;
      } else {
        // Use float for simplicity (only runs during calibration, not in control loop).
        ema_unwrap = static_cast<int32_t>(
            kAs5600EmaAlpha * static_cast<float>(unwrap) +
            (1.0f - kAs5600EmaAlpha) * static_cast<float>(ema_unwrap));
      }
      ++good;
    }
    delay(kDelayMs);
  }

  if (!ema_init || good < kNeedGood) {
    return false;
  }

  // Wrap EMA back to [0, 4096) counts, then convert to degrees.
  int32_t wrapped = ema_unwrap % 4096;
  if (wrapped < 0) { wrapped += 4096; }
  out_raw_deg = (static_cast<float>(wrapped) * 360.0f) / 4096.0f;
  return true;
}

bool captureZeroAngle() {
  float raw_deg = 0.0f;
  if (!captureAs5600CalRawDeg(raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
    printAs5600Diag();
    return false;
  }
  runtimeCalSetAs5600ZeroDeg(raw_deg);
  runtimeCalMarkZeroAngleCaptured(true);
  resetAs5600Filter();
  sampleAngleNow(millis(), nullptr, nullptr);
  g_runtime_cal_dirty = true;
  Serial.print(F("OK,cal_zero_angle_set="));
  Serial.println(raw_deg, 6);
  printCalZero();
  return true;
}

bool captureZeroPosition() {
  float dist_cm = 0.0f;
  if (!captureSonarCalDistanceCm(dist_cm)) {
    Serial.println(F("ERR,sonar_not_ready"));
    printSonarDiag();
    return false;
  }
  runtimeCalSetSonarCenterCm(dist_cm);
  runtimeCalMarkZeroPosCaptured(true);
  sampleSonarNow(millis(), nullptr, nullptr, nullptr);
  g_runtime_cal_dirty = true;
  Serial.print(F("OK,cal_zero_position_set="));
  Serial.println(dist_cm, 6);
  printCalZero();
  return true;
}

bool captureLowerLimit() {
  float raw_deg = 0.0f;
  if (!captureAs5600CalRawDeg(raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
    printAs5600Diag();
    return false;
  }
  const float theta_deg = runtimeMapThetaDeg(raw_deg);

  runtimeCalSetThetaLowerLimitDeg(theta_deg);
  runtimeCalMarkLowerLimitCaptured(true);
  g_runtime_cal_dirty = true;
  if (!runtimeCalHasValidLimitSpan()) {
    Serial.println(F("ERR,invalid_limit_span"));
  }
  Serial.print(F("OK,cal_limit_lower_set="));
  Serial.println(theta_deg, 6);
  printCalLimits();
  return true;
}

bool captureUpperLimit() {
  float raw_deg = 0.0f;
  if (!captureAs5600CalRawDeg(raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
    printAs5600Diag();
    return false;
  }
  const float theta_deg = runtimeMapThetaDeg(raw_deg);

  runtimeCalSetThetaUpperLimitDeg(theta_deg);
  runtimeCalMarkUpperLimitCaptured(true);
  g_runtime_cal_dirty = true;
  if (!runtimeCalHasValidLimitSpan()) {
    Serial.println(F("ERR,invalid_limit_span"));
  }
  Serial.print(F("OK,cal_limit_upper_set="));
  Serial.println(theta_deg, 6);
  printCalLimits();
  return true;
}

bool finalizeNormalizedLimitSpan() {
  if (!runtimeCalIsLowerLimitCaptured() || !runtimeCalIsUpperLimitCaptured()) {
    return false;
  }

  const float a_deg = runtimeCalThetaLowerLimitDeg();
  const float b_deg = runtimeCalThetaUpperLimitDeg();
  const float lo_deg = fminf(a_deg, b_deg);
  const float hi_deg = fmaxf(a_deg, b_deg);
  const float span_deg = hi_deg - lo_deg;
  if (span_deg < kLimitMinSpanDeg) {
    runtimeCalMarkLowerLimitCaptured(false);
    runtimeCalMarkUpperLimitCaptured(false);
    Serial.println(F("ERR,limit_span_too_small"));
    return false;
  }

  runtimeCalSetThetaLowerLimitDeg(lo_deg);
  runtimeCalSetThetaUpperLimitDeg(hi_deg);
  return true;
}

bool captureDownLimit() {
  float raw_deg = 0.0f;
  if (!captureAs5600CalRawDeg(raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
    printAs5600Diag();
    return false;
  }
  const float theta_deg = runtimeMapThetaDeg(raw_deg);

  runtimeCalSetThetaLowerLimitDeg(theta_deg);
  runtimeCalMarkLowerLimitCaptured(true);
  g_runtime_cal_dirty = true;

  Serial.print(F("OK,cal_limit_down_set="));
  Serial.println(theta_deg, 6);
  if (runtimeCalIsUpperLimitCaptured()) {
    if (!finalizeNormalizedLimitSpan()) {
      return false;
    }
  }
  printCalLimits();
  return true;
}

bool captureUpLimit() {
  float raw_deg = 0.0f;
  if (!captureAs5600CalRawDeg(raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
    printAs5600Diag();
    return false;
  }
  const float theta_deg = runtimeMapThetaDeg(raw_deg);

  runtimeCalSetThetaUpperLimitDeg(theta_deg);
  runtimeCalMarkUpperLimitCaptured(true);
  g_runtime_cal_dirty = true;

  Serial.print(F("OK,cal_limit_up_set="));
  Serial.println(theta_deg, 6);
  if (runtimeCalIsLowerLimitCaptured()) {
    if (!finalizeNormalizedLimitSpan()) {
      return false;
    }
  }
  printCalLimits();
  return true;
}

bool suggestZeroAngle() {
  float raw_deg = 0.0f;
  if (!captureAs5600CalRawDeg(raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
    printAs5600Diag();
    return false;
  }
  Serial.print(F("CAL_SUGGEST,AS5600_ZERO_DEG="));
  Serial.println(raw_deg, 6);
  return true;
}

bool suggestZeroPosition() {
  float dist_cm = 0.0f;
  if (!captureSonarCalDistanceCm(dist_cm)) {
    Serial.println(F("ERR,sonar_not_ready"));
    printSonarDiag();
    return false;
  }
  Serial.print(F("CAL_SUGGEST,SONAR_CENTER_CM="));
  Serial.println(dist_cm, 6);
  return true;
}

void printGuideNextAction() {
  // Deprecated: use host-side logger (/bringup) for workflow.
}

void printRunBlockedDetails(const AppConditions& cond) {
  Serial.println(F("ERR,run_blocked"));
  if (!cond.sign_calibrated) {
    Serial.println(F("BLOCK,sign,try=a"));
  }
  if (!cond.zero_calibrated) {
    Serial.println(F("BLOCK,zero,try=z"));
  }
  if (!cond.limits_calibrated) {
    Serial.println(F("BLOCK,limits,try=l/u"));
  }
  if (!cond.sensors_ok) {
    if (g_fault_flags.sonar_timeout) {
      Serial.println(F("BLOCK,sonar_timeout"));
    }
    if (g_fault_flags.i2c_error) {
      Serial.println(F("BLOCK,i2c_err"));
    }
    if (!g_sensor.valid_pos) {
      Serial.println(F("BLOCK,pos,try=p"));
    }
    if (!g_sensor.valid_angle && g_angle_src == 0) {
      Serial.println(F("BLOCK,angle,try=y"));
    }
  }
  if (cond.faults_active) {
    if (g_fault_flags.angle_oob) {
      Serial.println(F("BLOCK,angle_oob"));
    }
    if (g_fault_flags.pos_oob) {
      Serial.println(F("BLOCK,pos_oob"));
    }
    printFaultInfo();
  }
  if (!cond.inner_loop_stable) {
    Serial.println(F("BLOCK,inner_loop"));
  }
}

void handleCommand(char* line) {
  char* saveptr = nullptr;
  char* token = strtok_r(line, " ", &saveptr);
  if (token == nullptr) {
    return;
  }

  if (strcmp(token, "?") == 0 || strlen(token) == 1) {
    const char key = token[0];
    switch (key) {
      case '?':
      case 'h':
        printHelp();
        return;
      case 'i':
      case 's':
        printStatus();
        printFaultInfo();
        return;
      case 't': {
        char* arg = strtok_r(nullptr, " ", &saveptr);
        if (arg == nullptr) {
          g_telemetry_enabled = !g_telemetry_enabled;
        } else {
          g_telemetry_enabled = (atoi(arg) != 0);
        }
        Serial.print(F("OK,telemetry="));
        Serial.println(g_telemetry_enabled ? F("1") : F("0"));
        return;
      }
      case 'y': {
        if (g_state_machine.state() == AppState::RUNNING || g_stepper.jogActive()) {
          Serial.println(F("ERR,stop_running_before_jog"));
          return;
        }
        g_angle_src = (g_angle_src == 0) ? 1 : 0;
        if (g_angle_src != 0) {
          applySafeDisable();
          g_stepper.resetPositionSteps(0);
          g_fault_flags.i2c_error = false;
          g_fault_flags.angle_oob = false;
          g_sensor.beam_angle_deg = 0.0f;
          g_sensor.beam_angle_rad = 0.0f;
          g_sensor.beam_angle_raw_deg = 0.0f;
          g_sensor.valid_angle = true;
          g_last_valid_angle_ms = millis();
        } else {
          resetAs5600Filter();
        }
        Serial.print(F("OK,angle_src="));
        Serial.println(static_cast<int>(g_angle_src));
        return;
      }
      case 'e': {
        char* arg = strtok_r(nullptr, " ", &saveptr);
        if (arg == nullptr) {
          Serial.println(F("ERR,usage:e"));
          return;
        }
        const int value = atoi(arg);
        g_manual_enable = (value != 0);
        const bool manual_ok = g_sensor.valid_angle && !g_fault_flags.i2c_error && !g_fault_flags.angle_oob;
        if (g_manual_enable && manual_ok) {
          g_stepper.enable(true);
          Serial.println(F("OK,driver_enabled"));
        } else {
          applySafeDisable();
          Serial.println(F("OK,driver_disabled"));
        }
        return;
      }
      case 'j': {
        char* steps_arg = strtok_r(nullptr, " ", &saveptr);
        char* rate_arg = strtok_r(nullptr, " ", &saveptr);
        if (steps_arg == nullptr || rate_arg == nullptr) {
          Serial.println(F("ERR,usage:j"));
          return;
        }

        const long steps = atol(steps_arg);
        const float rate = parseFloatFast(rate_arg);
        if (steps == 0 || rate <= 0.0f) {
          Serial.println(F("ERR,invalid_jog_args"));
          return;
        }
        if (g_state_machine.state() == AppState::RUNNING) {
          Serial.println(F("ERR,stop_running_before_jog"));
          return;
        }

        if (g_angle_src != 0) {
          const int32_t pos_steps = g_stepper.positionSteps();
          const float requested_rate = (steps > 0) ? rate : -rate;
          const float hw_dir_ref = requested_rate * static_cast<float>(runtimeCalStepperDirSign());
          if (hw_dir_ref >= 0.0f) {
            if (pos_steps >= kStepperPosLimitSteps) {
              Serial.println(F("ERR,jog_blocked_upper_limit"));
              return;
            }
          } else {
            if (pos_steps <= -kStepperPosLimitSteps) {
              Serial.println(F("ERR,jog_blocked_lower_limit"));
              return;
            }
          }
        } else {
          if (!runtimeCalIsLimitsSet()) {
            Serial.println(F("ERR,jog_requires_limits"));
            return;
          }
          float theta_now_deg = 0.0f;
          if (!sampleAngleNow(millis(), &theta_now_deg, nullptr)) {
            Serial.println(F("ERR,angle_not_ready"));
            return;
          }
          if (steps > 0 && theta_now_deg >= (runtimeCalThetaUpperLimitDeg() - kLimitStopMarginDeg)) {
            Serial.println(F("ERR,jog_blocked_upper_limit"));
            return;
          }
          if (steps < 0 && theta_now_deg <= (runtimeCalThetaLowerLimitDeg() + kLimitStopMarginDeg)) {
            Serial.println(F("ERR,jog_blocked_lower_limit"));
            return;
          }
        }

        g_stepper.enable(true);
        g_stepper.requestJogSteps(steps, rate);
        Serial.println(F("OK,jog_started"));
        return;
      }
      case 'a':
        if (captureZeroAngle()) {
          printGuideNextAction();
        }
        return;
      case 'p':
        if (captureZeroPosition()) {
          printGuideNextAction();
        }
        return;
      case 'z':
        printCalZero();
        printGuideNextAction();
        return;
      case '[':
      case '{':
      case 'l':
      case '1':
        if (captureDownLimit()) {
          printGuideNextAction();
        }
        return;
      case ']':
      case '}':
      case 'u':
      case '2':
        if (captureUpLimit()) {
          printGuideNextAction();
        }
        return;
      case 'm':
        printCalLimits();
        printGuideNextAction();
        return;
      case 'b':
        if (handleCalSignBegin()) {
          printGuideNextAction();
        }
        return;
      case 'g':
        if (handleCalSignSave()) {
          printGuideNextAction();
        }
        return;
      case 'v':
        if (runtimeCalSave()) {
          g_runtime_cal_dirty = false;
          Serial.println(F("OK,cal_saved"));
        } else {
          Serial.println(F("ERR,cal_save_failed"));
        }
        printGuideNextAction();
        return;
      case 'o': {
        const bool loaded = runtimeCalLoad();
        resetAs5600Filter();
        g_runtime_cal_dirty = false;
        Serial.print(F("OK,cal_load,"));
        Serial.println(loaded ? F("loaded") : F("defaults"));
        printGuideNextAction();
        return;
      }
      case 'd':
        runtimeCalResetDefaults();
        resetAs5600Filter();
        g_runtime_cal_dirty = true;
        Serial.println(F("OK,cal_defaults_applied"));
        printGuideNextAction();
        return;
      case 'r': {
        readSensors(millis());
        updateFaultState(millis());
        const AppConditions cond = currentConditions();
        if (g_state_machine.requestRun(cond)) {
          g_controller.reset();
          g_setpoint.ball_pos_cm_target = kDefaultBallSetpointCm;
          g_setpoint.ball_pos_m_target = g_setpoint.ball_pos_cm_target * kCmToM;
          Serial.println(F("OK,running"));
        } else {
          printRunBlockedDetails(cond);
        }
        return;
      }
      case 'k':
        g_state_machine.requestStop();
        applySafeDisable();
        Serial.println(F("OK,stopped"));
        printGuideNextAction();
        return;
      case 'f':
        g_fault_flags = FaultFlags{};
        if (g_state_machine.clearFault(currentConditions())) {
          Serial.println(F("OK,fault_cleared"));
        } else {
          Serial.println(F("ERR,no_fault_state"));
        }
        printFaultInfo();
        printGuideNextAction();
        return;
      case 'x':
        printFaultInfo();
        printSonarDiag();
        printGuideNextAction();
        return;
      default:
        break;
    }
  }

  if (strcmp(token, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(token, "status") == 0) {
    printStatus();
    printFaultInfo();
    printGuideNextAction();
    return;
  }

  if (strcmp(token, "telemetry") == 0) {
    char* arg = strtok_r(nullptr, " ", &saveptr);
    if (arg == nullptr) {
      Serial.println(F("ERR,usage:telemetry"));
      return;
    }
    const int value = atoi(arg);
    g_telemetry_enabled = (value != 0);
    Serial.print(F("OK,telemetry="));
    Serial.println(g_telemetry_enabled ? F("1") : F("0"));
    return;
  }

  if (strcmp(token, "en") == 0) {
    char* arg = strtok_r(nullptr, " ", &saveptr);
    if (arg == nullptr) {
      Serial.println(F("ERR,en_requires_0_or_1"));
      return;
    }

    const int value = atoi(arg);
    g_manual_enable = (value != 0);
    const bool manual_ok = g_sensor.valid_angle && !g_fault_flags.i2c_error && !g_fault_flags.angle_oob;
    if (g_manual_enable && manual_ok) {
      g_stepper.enable(true);
      Serial.println(F("OK,driver_enabled"));
    } else {
      applySafeDisable();
      Serial.println(F("OK,driver_disabled"));
    }
    return;
  }

  if (strcmp(token, "jog") == 0) {
    char* steps_arg = strtok_r(nullptr, " ", &saveptr);
    char* rate_arg = strtok_r(nullptr, " ", &saveptr);
    if (steps_arg == nullptr || rate_arg == nullptr) {
      Serial.println(F("ERR,usage:jog"));
      return;
    }

    const long steps = atol(steps_arg);
    const float rate = parseFloatFast(rate_arg);
    if (steps == 0 || rate <= 0.0f) {
      Serial.println(F("ERR,invalid_jog_args"));
      return;
    }

    if (g_state_machine.state() == AppState::RUNNING) {
      Serial.println(F("ERR,stop_running_before_jog"));
      return;
    }

    if (g_angle_src != 0) {
      const int32_t pos_steps = g_stepper.positionSteps();
      const float requested_rate = (steps > 0) ? rate : -rate;
      const float hw_dir_ref = requested_rate * static_cast<float>(runtimeCalStepperDirSign());
      if (hw_dir_ref >= 0.0f) {
        if (pos_steps >= kStepperPosLimitSteps) {
          Serial.println(F("ERR,jog_blocked_upper_limit"));
          return;
        }
      } else {
        if (pos_steps <= -kStepperPosLimitSteps) {
          Serial.println(F("ERR,jog_blocked_lower_limit"));
          return;
        }
      }
    } else {
      float theta_now_deg = 0.0f;
      if (!runtimeCalIsLimitsSet()) {
        Serial.println(F("ERR,jog_requires_limits"));
        return;
      }
      if (!sampleAngleNow(millis(), &theta_now_deg, nullptr)) {
        Serial.println(F("ERR,angle_not_ready"));
        return;
      }
      if (steps > 0 && theta_now_deg >= (runtimeCalThetaUpperLimitDeg() - kLimitStopMarginDeg)) {
        Serial.println(F("ERR,jog_blocked_upper_limit"));
        return;
      }
      if (steps < 0 && theta_now_deg <= (runtimeCalThetaLowerLimitDeg() + kLimitStopMarginDeg)) {
        Serial.println(F("ERR,jog_blocked_lower_limit"));
        return;
      }
    }

    g_stepper.enable(true);
    g_stepper.requestJogSteps(steps, rate);
    Serial.println(F("OK,jog_started"));
    return;
  }

  if (strcmp(token, "cal_sign") == 0) {
    char* sub = strtok_r(nullptr, " ", &saveptr);
    if (sub == nullptr) {
      Serial.println(F("ERR,usage:cal_sign"));
      return;
    }
    if (strcmp(sub, "begin") == 0) {
      if (handleCalSignBegin()) {
        printGuideNextAction();
      }
      return;
    }
    if (strcmp(sub, "save") == 0) {
      if (handleCalSignSave()) {
        printGuideNextAction();
      }
      return;
    }
    Serial.println(F("ERR,usage:cal_sign"));
    return;
  }

  if (strcmp(token, "sonar") == 0) {
    char* sub = strtok_r(nullptr, " ", &saveptr);
    if (sub == nullptr) {
      Serial.println(F("ERR,usage:sonar"));
      return;
    }
    if (strcmp(sub, "diag") == 0) {
      printSonarDiag();
      printGuideNextAction();
      return;
    }
    if (strcmp(sub, "sign") == 0) {
      char* arg = strtok_r(nullptr, " ", &saveptr);
      if (arg == nullptr) {
        Serial.println(F("ERR,usage:sonar_sign"));
        return;
      }
      const int value = atoi(arg);
      if (value != -1 && value != 1) {
        Serial.println(F("ERR,usage:sonar_sign"));
        return;
      }
      runtimeCalSetSonarPosSign(static_cast<int8_t>(value));
      g_runtime_cal_dirty = true;
      Serial.print(F("OK,sonar_sign_set="));
      Serial.println(value);
      if (g_sign_cal.active) {
        g_sign_cal.complete = true;
        g_sign_cal.active = false;
      }
      printGuideNextAction();
      return;
    }
    Serial.println(F("ERR,usage:sonar"));
    return;
  }

  if (strcmp(token, "as5600") == 0) {
    printAs5600Diag();
    printGuideNextAction();
    return;
  }

  if (strcmp(token, "cal_zero") == 0) {
    char* sub = strtok_r(nullptr, " ", &saveptr);
    if (sub == nullptr) {
      Serial.println(F("ERR,usage:cal_zero"));
      return;
    }

    if (strcmp(sub, "show") == 0) {
      printCalZero();
      printGuideNextAction();
      return;
    }

    if (strcmp(sub, "angle") == 0) {
      suggestZeroAngle();
      return;
    }

    if (strcmp(sub, "position") == 0) {
      suggestZeroPosition();
      return;
    }

    if (strcmp(sub, "set") == 0) {
      char* target = strtok_r(nullptr, " ", &saveptr);
      if (target == nullptr) {
        Serial.println(F("ERR,usage:cal_zero_set"));
        return;
      }

      if (strcmp(target, "angle") == 0) {
        if (captureZeroAngle()) {
          printGuideNextAction();
        }
        return;
      }

      if (strcmp(target, "position") == 0) {
        if (captureZeroPosition()) {
          printGuideNextAction();
        }
        return;
      }

      Serial.println(F("ERR,usage:cal_zero_set"));
      return;
    }

    Serial.println(F("ERR,usage:cal_zero"));
    return;
  }

  if (strcmp(token, "cal_limits") == 0) {
    char* sub = strtok_r(nullptr, " ", &saveptr);
    if (sub == nullptr) {
      Serial.println(F("ERR,usage:cal_limits"));
      return;
    }

    if (strcmp(sub, "show") == 0) {
      printCalLimits();
      printGuideNextAction();
      return;
    }

    if (strcmp(sub, "set") == 0) {
      char* edge = strtok_r(nullptr, " ", &saveptr);
      if (edge == nullptr) {
        Serial.println(F("ERR,usage:cal_limits_set"));
        return;
      }

      if (strcmp(edge, "down") == 0) {
        if (captureDownLimit()) {
          printGuideNextAction();
        }
        return;
      }

      if (strcmp(edge, "up") == 0) {
        if (captureUpLimit()) {
          printGuideNextAction();
        }
        return;
      }

      if (strcmp(edge, "lower") == 0) {
        if (captureLowerLimit()) {
          printGuideNextAction();
        }
        return;
      }

      if (strcmp(edge, "upper") == 0) {
        if (captureUpperLimit()) {
          printGuideNextAction();
        }
        return;
      }

      Serial.println(F("ERR,usage:cal_limits_set"));
      return;
    }

    Serial.println(F("ERR,usage:cal_limits"));
    return;
  }

  if (strcmp(token, "cal_save") == 0) {
    if (runtimeCalSave()) {
      g_runtime_cal_dirty = false;
      Serial.println(F("OK,cal_saved"));
    } else {
      Serial.println(F("ERR,cal_save_failed"));
    }
    printGuideNextAction();
    return;
  }

  if (strcmp(token, "cal_load") == 0) {
    const bool loaded = runtimeCalLoad();
    resetAs5600Filter();
    g_runtime_cal_dirty = false;
    Serial.print(F("OK,cal_load,"));
    Serial.println(loaded ? F("loaded") : F("defaults"));
    printGuideNextAction();
    return;
  }

  if (strcmp(token, "cal_reset") == 0) {
    char* sub = strtok_r(nullptr, " ", &saveptr);
    if (sub == nullptr || strcmp(sub, "defaults") != 0) {
      Serial.println(F("ERR,usage:cal_reset"));
      return;
    }
    runtimeCalResetDefaults();
    resetAs5600Filter();
    g_runtime_cal_dirty = true;
    Serial.println(F("OK,cal_defaults_applied"));
    printGuideNextAction();
    return;
  }

  if (strcmp(token, "run") == 0) {
    readSensors(millis());
    updateFaultState(millis());
    const AppConditions cond = currentConditions();
    if (g_state_machine.requestRun(cond)) {
      g_controller.reset();
      g_setpoint.ball_pos_cm_target = kDefaultBallSetpointCm;
      g_setpoint.ball_pos_m_target = g_setpoint.ball_pos_cm_target * kCmToM;
      Serial.println(F("OK,running"));
    } else {
      printRunBlockedDetails(cond);
    }
    return;
  }

  if (strcmp(token, "stop") == 0) {
    g_state_machine.requestStop();
    applySafeDisable();
    Serial.println(F("OK,stopped"));
    printGuideNextAction();
    return;
  }

  if (strcmp(token, "fault_reset") == 0) {
    g_fault_flags = FaultFlags{};
    if (g_state_machine.clearFault(currentConditions())) {
      Serial.println(F("OK,fault_cleared"));
    } else {
      Serial.println(F("ERR,no_fault_state"));
    }
    printFaultInfo();
    printGuideNextAction();
    return;
  }

  if (strcmp(token, "faults") == 0) {
    printFaultInfo();
    printSonarDiag();
    printGuideNextAction();
    return;
  }

  if (strcmp(token, "guide") == 0) {
    printStatus();
    printFaultInfo();
    return;
  }

  Serial.println(F("ERR,unknown_command,try=h"));
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      g_cmd_buffer[g_cmd_len] = '\0';
      handleCommand(g_cmd_buffer);
      g_cmd_len = 0;
      continue;
    }
    if (g_cmd_len + 1 < sizeof(g_cmd_buffer)) {
      g_cmd_buffer[g_cmd_len++] = c;
    }
  }
}

void serviceControl(uint32_t now_ms) {
  readSensors(now_ms);
  updateFaultState(now_ms);

  if (g_state_machine.state() == AppState::RUNNING) {
    if (hasAnyFault()) {
      applySafeDisable();
      return;
    }

    g_setpoint.ball_pos_m_target = g_setpoint.ball_pos_cm_target * kCmToM;

    float theta_min_rad = -kThetaCmdLimitRad;
    float theta_max_rad = kThetaCmdLimitRad;
    effectiveThetaCmdBoundsRad(theta_min_rad, theta_max_rad);

    const ActuatorCmd cmd = g_controller.update(
        g_sensor, g_setpoint, kControlDtSec, theta_min_rad, theta_max_rad);
    if (cmd.enable) {
      g_stepper.enable(true);
      g_stepper.setSignedStepRate(cmd.signed_step_rate_sps);
    } else {
      applySafeDisable();
    }
    return;
  }

  if (g_stepper.jogActive()) {
    maybeStopJogAtLimits();
    if (g_stepper.jogActive()) {
      g_stepper.enable(true);
      return;
    }
  }

  if (g_manual_enable) {
    const bool manual_ok = g_sensor.valid_angle && !g_fault_flags.i2c_error && !g_fault_flags.angle_oob;
    if (manual_ok) {
      g_stepper.enable(true);
    } else {
      applySafeDisable();
    }
  } else {
    applySafeDisable();
  }
}

void handleEchoPcintIsr() {
  if (g_echo_pin_reg == nullptr || g_echo_pin_mask == 0) {
    return;
  }

  const bool level_high = ((*g_echo_pin_reg) & g_echo_pin_mask) != 0;
  g_hcsr04.handleEchoEdgeIsr(micros(), level_high);
}

}  // namespace

ISR(TIMER1_COMPA_vect) {
  g_stepper.handleTimerCompareIsr();
}

ISR(PCINT0_vect) {
  handleEchoPcintIsr();
}

ISR(PCINT1_vect) {
  handleEchoPcintIsr();
}

ISR(PCINT2_vect) {
  handleEchoPcintIsr();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(250);

  Serial.println(F("BALL_BEAM_BOOT"));

  runtimeCalInit();

  g_stepper.begin();
  g_stepper.beginScheduler();
  applySafeDisable();

  Wire.begin();
  const bool i2c_ok = g_as5600.begin(Wire, AS5600_I2C_ADDR);
  g_hcsr04.begin();
  setupEchoPcint();

  g_fault_flags = FaultFlags{};
  if (!i2c_ok) {
    g_fault_flags.i2c_error = true;
  }

  g_setpoint.ball_pos_cm_target = kDefaultBallSetpointCm;
  g_setpoint.ball_pos_m_target = g_setpoint.ball_pos_cm_target * kCmToM;

  g_last_control_ms = millis();
  g_last_telemetry_ms = g_last_control_ms;
  g_last_valid_angle_ms = g_last_control_ms;
  g_last_valid_pos_ms = g_last_control_ms;

  printHelp();
  printStatus();
  printFaultInfo();
  printGuideNextAction();
}

void loop() {
  const uint32_t now_us = micros();
  const uint32_t now_ms = millis();

  g_stepper.processIsrFlags();
  g_hcsr04.service(now_us, now_ms);
  pollSerial();

  if (static_cast<uint32_t>(now_ms - g_last_control_ms) >= kControlPeriodMs) {
    g_last_control_ms = now_ms;
    serviceControl(now_ms);
  }

  if (static_cast<uint32_t>(now_ms - g_last_telemetry_ms) >= kTelemetryPeriodMs) {
    g_last_telemetry_ms = now_ms;
    emitTelemetry(now_ms);
  }
}
