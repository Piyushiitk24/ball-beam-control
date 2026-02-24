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

uint32_t g_last_control_ms = 0;
uint32_t g_last_telemetry_ms = 0;
uint32_t g_last_valid_angle_ms = 0;
uint32_t g_last_valid_pos_ms = 0;

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
  if (!g_as5600.readBeamThetaDeg(theta_deg, theta_raw_deg)) {
    g_sensor.valid_angle = false;
    g_fault_flags.i2c_error = true;
    return false;
  }

  g_sensor.beam_angle_deg = theta_deg;
  g_sensor.beam_angle_rad = theta_deg * kDegToRad;
  g_sensor.beam_angle_raw_deg = theta_raw_deg;
  g_sensor.valid_angle = true;
  g_last_valid_angle_ms = now_ms;
  g_fault_flags.i2c_error = false;
  g_fault_flags.angle_oob = fabsf(g_sensor.beam_angle_rad) > kThetaHardLimitRad;

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
    if (g_hcsr04.hasTimeout()) {
      g_fault_flags.sonar_timeout = true;
    }
    return false;
  }

  g_sensor.ball_pos_cm = x_cm;
  g_sensor.ball_pos_m = x_cm * kCmToM;
  g_sensor.ball_pos_filt_cm = x_filt_cm;
  g_sensor.ball_pos_filt_m = x_filt_cm * kCmToM;
  g_sensor.sonar_distance_raw_cm = dist_cm;

  const bool fresh = g_hcsr04.hasFreshSample(now_ms);
  const bool timeout = g_hcsr04.hasTimeout();
  g_sensor.valid_pos = fresh && !timeout;

  if (g_sensor.valid_pos) {
    g_last_valid_pos_ms = now_ms;
    g_fault_flags.sonar_timeout = false;
  } else if (timeout) {
    g_fault_flags.sonar_timeout = true;
  }

  g_fault_flags.pos_oob = fabsf(g_sensor.ball_pos_filt_m) > kBallPosHardLimitM;

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
  const bool angle_ok = sampleAngleNow(now_ms);
  const bool pos_ok = sampleSonarNow(now_ms);
  return angle_ok && pos_ok;
}

void updateFaultState(uint32_t now_ms) {
  const uint32_t stale_threshold_ms = activeStaleThresholdMs();

  if (static_cast<uint32_t>(now_ms - g_last_valid_angle_ms) > stale_threshold_ms) {
    g_fault_flags.i2c_error = true;
  }

  if (static_cast<uint32_t>(now_ms - g_last_valid_pos_ms) > stale_threshold_ms) {
    g_fault_flags.sonar_timeout = true;
  }

  if (g_hcsr04.hasTimeout()) {
    g_fault_flags.sonar_timeout = true;
  }

  if (hasAnyFault()) {
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
  if (!runtimeCalIsLimitsSet() || !g_sensor.valid_angle) {
    return;
  }

  const float theta_deg = g_sensor.beam_angle_deg;
  const float rate = g_stepper.targetSignedStepRate();

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
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  status"));
  Serial.println(F("  telemetry 0|1"));
  Serial.println(F("  en 0|1"));
  Serial.println(F("  jog <signed_steps> <rate>"));
  Serial.println(F("  cal_zero set angle"));
  Serial.println(F("  cal_zero set position"));
  Serial.println(F("  cal_zero show"));
  Serial.println(F("  cal_zero angle      (compat suggest)"));
  Serial.println(F("  cal_zero position   (compat suggest)"));
  Serial.println(F("  cal_limits set lower"));
  Serial.println(F("  cal_limits set upper"));
  Serial.println(F("  cal_limits show"));
  Serial.println(F("  cal_sign begin"));
  Serial.println(F("  cal_sign save"));
  Serial.println(F("  cal_save"));
  Serial.println(F("  cal_load"));
  Serial.println(F("  cal_reset defaults"));
  Serial.println(F("  run"));
  Serial.println(F("  stop"));
  Serial.println(F("  fault_reset"));
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

  Serial.print(F("SIGNS,current_stepper="));
  Serial.print(runtimeCalStepperDirSign());
  Serial.print(F(",current_as5600="));
  Serial.print(runtimeCalAs5600Sign());
  Serial.print(F(",current_sonar="));
  Serial.println(runtimeCalSonarPosSign());

  Serial.print(F("CAL,zero_calibrated="));
  Serial.print(runtimeCalIsZeroSet() ? F("yes") : F("no"));
  Serial.print(F(",limits_calibrated="));
  Serial.print(runtimeCalIsLimitsSet() ? F("yes") : F("no"));
  Serial.print(F(",sign_calibrated="));
  Serial.println(runtimeCalIsSignSet() ? F("yes") : F("no"));

  Serial.print(F("CAL_ZERO,as5600_zero_deg="));
  Serial.print(runtimeCalAs5600ZeroDeg(), 6);
  Serial.print(F(",sonar_center_cm="));
  Serial.println(runtimeCalSonarCenterCm(), 6);

  Serial.print(F("CAL_LIMITS,lower_deg="));
  Serial.print(runtimeCalThetaLowerLimitDeg(), 5);
  Serial.print(F(",upper_deg="));
  Serial.print(runtimeCalThetaUpperLimitDeg(), 5);
  Serial.print(F(",valid="));
  Serial.println(runtimeCalIsLimitsSet() ? F("yes") : F("no"));

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

    if (static_cast<uint32_t>(now_ms - start_ms) > timeout_ms) {
      g_stepper.stop();
      break;
    }
  }

  g_stepper.processIsrFlags();
}

void handleCalSignBegin() {
  if (g_state_machine.state() == AppState::FAULT) {
    Serial.println(F("ERR,fault_active"));
    return;
  }

  if (!readSensors(millis())) {
    Serial.println(F("ERR,sensors_not_ready"));
    return;
  }

  if (!g_state_machine.startSignCalibration()) {
    Serial.println(F("ERR,cannot_enter_calib_sign"));
    return;
  }

  g_sign_cal.active = true;
  g_sign_cal.near_captured = true;
  g_sign_cal.complete = false;
  g_sign_cal.suggested_stepper_sign = runtimeCalStepperDirSign();
  g_sign_cal.suggested_as5600_sign = runtimeCalAs5600Sign();
  g_sign_cal.suggested_sonar_sign = runtimeCalSonarPosSign();

  g_sign_cal.near_distance_cm = g_sensor.sonar_distance_raw_cm;
  g_sign_cal.theta_before_raw_deg = g_sensor.beam_angle_raw_deg;

  g_stepper.enable(true);
  g_stepper.requestJogSteps(SIGN_CAL_JOG_STEPS, SIGN_CAL_JOG_RATE_SPS);
  runJogBlocking(3000);

  delay(50);
  readSensors(millis());
  g_sign_cal.theta_after_raw_deg = g_sensor.beam_angle_raw_deg;
  g_sign_cal.theta_delta_raw_deg =
      wrapAngleDeltaDeg(g_sign_cal.theta_after_raw_deg - g_sign_cal.theta_before_raw_deg);

  const float mapped_delta =
      static_cast<float>(runtimeCalAs5600Sign()) * g_sign_cal.theta_delta_raw_deg;

  g_sign_cal.suggested_stepper_sign =
      (mapped_delta >= 0.0f) ? runtimeCalStepperDirSign() : static_cast<int8_t>(-runtimeCalStepperDirSign());

  g_sign_cal.suggested_as5600_sign = (g_sign_cal.theta_delta_raw_deg >= 0.0f) ? 1 : -1;

  Serial.println(F("INFO,cal_sign_begin_complete"));
  Serial.print(F("INFO,theta_raw_delta_deg="));
  Serial.println(g_sign_cal.theta_delta_raw_deg, 5);
  Serial.print(F("INFO,suggested_stepper_sign="));
  Serial.println(g_sign_cal.suggested_stepper_sign);
  Serial.print(F("INFO,suggested_as5600_sign="));
  Serial.println(g_sign_cal.suggested_as5600_sign);
  Serial.println(F("INFO,move ball to far end (+x), then run: cal_sign save"));

  if (!g_manual_enable) {
    applySafeDisable();
  }
}

void handleCalSignSave() {
  if (!g_sign_cal.active || !g_sign_cal.near_captured) {
    Serial.println(F("ERR,cal_sign_not_started"));
    return;
  }

  float x_cm = 0.0f;
  float x_filt_cm = 0.0f;
  float dist_cm = 0.0f;
  if (!sampleSonarNow(millis(), &x_cm, &x_filt_cm, &dist_cm)) {
    Serial.println(F("ERR,sonar_not_ready"));
    return;
  }

  g_sign_cal.far_distance_cm = dist_cm;
  g_sign_cal.suggested_sonar_sign =
      (g_sign_cal.far_distance_cm >= g_sign_cal.near_distance_cm) ? 1 : -1;

  g_sign_cal.complete = true;
  g_sign_cal.active = false;

  runtimeCalSetStepperDirSign(g_sign_cal.suggested_stepper_sign);
  runtimeCalSetAs5600Sign(g_sign_cal.suggested_as5600_sign);
  runtimeCalSetSonarPosSign(g_sign_cal.suggested_sonar_sign);
  runtimeCalSetSignCaptured(true);

  g_state_machine.finishSignCalibration(!hasAnyFault());

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

  Serial.println(F("INFO,signs_applied_runtime; run cal_save to persist"));
}

AppConditions currentConditions() {
  AppConditions cond;
  cond.sign_calibrated = runtimeCalIsSignSet();
  cond.zero_calibrated = runtimeCalIsZeroSet();
  cond.limits_calibrated = runtimeCalIsLimitsSet();
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

void handleCommand(char* line) {
  char* saveptr = nullptr;
  char* token = strtok_r(line, " ", &saveptr);
  if (token == nullptr) {
    return;
  }

  if (strcmp(token, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(token, "status") == 0) {
    printStatus();
    return;
  }

  if (strcmp(token, "telemetry") == 0) {
    char* arg = strtok_r(nullptr, " ", &saveptr);
    if (arg == nullptr) {
      Serial.println(F("ERR,usage:telemetry 0|1"));
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
    if (g_manual_enable && !hasAnyFault()) {
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
      Serial.println(F("ERR,usage:jog <signed_steps> <rate>"));
      return;
    }

    const long steps = atol(steps_arg);
    const float rate = atof(rate_arg);
    if (steps == 0 || rate <= 0.0f) {
      Serial.println(F("ERR,invalid_jog_args"));
      return;
    }

    if (g_state_machine.state() == AppState::RUNNING) {
      Serial.println(F("ERR,stop_running_before_jog"));
      return;
    }

    float theta_now_deg = 0.0f;
    if (runtimeCalIsLimitsSet() && sampleAngleNow(millis(), &theta_now_deg, nullptr)) {
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
      Serial.println(F("ERR,usage:cal_sign begin|save"));
      return;
    }
    if (strcmp(sub, "begin") == 0) {
      handleCalSignBegin();
      return;
    }
    if (strcmp(sub, "save") == 0) {
      handleCalSignSave();
      return;
    }
    Serial.println(F("ERR,usage:cal_sign begin|save"));
    return;
  }

  if (strcmp(token, "cal_zero") == 0) {
    char* sub = strtok_r(nullptr, " ", &saveptr);
    if (sub == nullptr) {
      Serial.println(F("ERR,usage:cal_zero set angle|position | cal_zero show"));
      return;
    }

    if (strcmp(sub, "show") == 0) {
      printCalZero();
      return;
    }

    if (strcmp(sub, "angle") == 0) {
      float theta_deg = 0.0f;
      float raw_deg = 0.0f;
      if (!sampleAngleNow(millis(), &theta_deg, &raw_deg)) {
        Serial.println(F("ERR,angle_not_ready"));
        return;
      }
      Serial.print(F("CAL_SUGGEST,AS5600_ZERO_DEG="));
      Serial.println(raw_deg, 6);
      return;
    }

    if (strcmp(sub, "position") == 0) {
      float x_cm = 0.0f;
      float x_filt_cm = 0.0f;
      float dist_cm = 0.0f;
      if (!sampleSonarNow(millis(), &x_cm, &x_filt_cm, &dist_cm)) {
        Serial.println(F("ERR,sonar_not_ready"));
        return;
      }
      Serial.print(F("CAL_SUGGEST,SONAR_CENTER_CM="));
      Serial.println(dist_cm, 6);
      return;
    }

    if (strcmp(sub, "set") == 0) {
      char* target = strtok_r(nullptr, " ", &saveptr);
      if (target == nullptr) {
        Serial.println(F("ERR,usage:cal_zero set angle|position"));
        return;
      }

      if (strcmp(target, "angle") == 0) {
        float theta_deg = 0.0f;
        float raw_deg = 0.0f;
        if (!sampleAngleNow(millis(), &theta_deg, &raw_deg)) {
          Serial.println(F("ERR,angle_not_ready"));
          return;
        }
        runtimeCalSetAs5600ZeroDeg(raw_deg);
        runtimeCalMarkZeroAngleCaptured(true);
        sampleAngleNow(millis(), nullptr, nullptr);
        Serial.print(F("OK,cal_zero_angle_set="));
        Serial.println(raw_deg, 6);
        printCalZero();
        return;
      }

      if (strcmp(target, "position") == 0) {
        float x_cm = 0.0f;
        float x_filt_cm = 0.0f;
        float dist_cm = 0.0f;
        if (!sampleSonarNow(millis(), &x_cm, &x_filt_cm, &dist_cm)) {
          Serial.println(F("ERR,sonar_not_ready"));
          return;
        }
        runtimeCalSetSonarCenterCm(dist_cm);
        runtimeCalMarkZeroPosCaptured(true);
        sampleSonarNow(millis(), nullptr, nullptr, nullptr);
        Serial.print(F("OK,cal_zero_position_set="));
        Serial.println(dist_cm, 6);
        printCalZero();
        return;
      }

      Serial.println(F("ERR,usage:cal_zero set angle|position"));
      return;
    }

    Serial.println(F("ERR,usage:cal_zero set angle|position | cal_zero show"));
    return;
  }

  if (strcmp(token, "cal_limits") == 0) {
    char* sub = strtok_r(nullptr, " ", &saveptr);
    if (sub == nullptr) {
      Serial.println(F("ERR,usage:cal_limits set lower|upper | cal_limits show"));
      return;
    }

    if (strcmp(sub, "show") == 0) {
      printCalLimits();
      return;
    }

    if (strcmp(sub, "set") == 0) {
      char* edge = strtok_r(nullptr, " ", &saveptr);
      if (edge == nullptr) {
        Serial.println(F("ERR,usage:cal_limits set lower|upper"));
        return;
      }

      float theta_deg = 0.0f;
      if (!sampleAngleNow(millis(), &theta_deg, nullptr)) {
        Serial.println(F("ERR,angle_not_ready"));
        return;
      }

      if (strcmp(edge, "lower") == 0) {
        runtimeCalSetThetaLowerLimitDeg(theta_deg);
        runtimeCalMarkLowerLimitCaptured(true);
        if (!runtimeCalHasValidLimitSpan()) {
          Serial.println(F("ERR,invalid_limit_span"));
        }
        Serial.print(F("OK,cal_limit_lower_set="));
        Serial.println(theta_deg, 6);
        printCalLimits();
        return;
      }

      if (strcmp(edge, "upper") == 0) {
        runtimeCalSetThetaUpperLimitDeg(theta_deg);
        runtimeCalMarkUpperLimitCaptured(true);
        if (!runtimeCalHasValidLimitSpan()) {
          Serial.println(F("ERR,invalid_limit_span"));
        }
        Serial.print(F("OK,cal_limit_upper_set="));
        Serial.println(theta_deg, 6);
        printCalLimits();
        return;
      }

      Serial.println(F("ERR,usage:cal_limits set lower|upper"));
      return;
    }

    Serial.println(F("ERR,usage:cal_limits set lower|upper | cal_limits show"));
    return;
  }

  if (strcmp(token, "cal_save") == 0) {
    if (runtimeCalSave()) {
      Serial.println(F("OK,cal_saved"));
    } else {
      Serial.println(F("ERR,cal_save_failed"));
    }
    return;
  }

  if (strcmp(token, "cal_load") == 0) {
    const bool loaded = runtimeCalLoad();
    Serial.print(F("OK,cal_load,"));
    Serial.println(loaded ? F("loaded") : F("defaults"));
    return;
  }

  if (strcmp(token, "cal_reset") == 0) {
    char* sub = strtok_r(nullptr, " ", &saveptr);
    if (sub == nullptr || strcmp(sub, "defaults") != 0) {
      Serial.println(F("ERR,usage:cal_reset defaults"));
      return;
    }
    runtimeCalResetDefaults();
    Serial.println(F("OK,cal_defaults_applied"));
    return;
  }

  if (strcmp(token, "run") == 0) {
    readSensors(millis());
    updateFaultState(millis());

    if (g_state_machine.requestRun(currentConditions())) {
      g_controller.reset();
      g_setpoint.ball_pos_cm_target = kDefaultBallSetpointCm;
      g_setpoint.ball_pos_m_target = g_setpoint.ball_pos_cm_target * kCmToM;
      Serial.println(F("OK,running"));
    } else {
      Serial.println(F("ERR,run_blocked_check_status"));
    }
    return;
  }

  if (strcmp(token, "stop") == 0) {
    g_state_machine.requestStop();
    applySafeDisable();
    Serial.println(F("OK,stopped"));
    return;
  }

  if (strcmp(token, "fault_reset") == 0) {
    g_fault_flags = FaultFlags{};
    if (g_state_machine.clearFault(currentConditions())) {
      Serial.println(F("OK,fault_cleared"));
    } else {
      Serial.println(F("ERR,no_fault_state"));
    }
    return;
  }

  Serial.println(F("ERR,unknown_command"));
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

  if (g_manual_enable && !hasAnyFault()) {
    g_stepper.enable(true);
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
