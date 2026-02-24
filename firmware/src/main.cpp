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

struct GuidedWizardSession {
  bool active = false;
  uint8_t step_index = 0;
  bool telemetry_prev = true;
  bool awaiting_save_confirm = false;
};

GuidedWizardSession g_wizard;
bool g_runtime_cal_dirty = false;

enum WizardStep : uint8_t {
  kWizardStepZeroAngle = 0,
  kWizardStepZeroPosition = 1,
  kWizardStepLowerLimit = 2,
  kWizardStepUpperLimit = 3,
  kWizardStepSignBegin = 4,
  kWizardStepSignSave = 5,
  kWizardStepSaveConfirm = 6
};

void printFaultInfo();
void printBringupMenu();
void printSonarDiag();

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
    g_fault_flags.pos_oob = false;
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
    g_fault_flags.pos_oob = fabsf(g_sensor.ball_pos_filt_m) > kBallPosHardLimitM;
  } else if (timeout) {
    g_fault_flags.sonar_timeout = true;
    g_fault_flags.pos_oob = false;
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
  Serial.println(F("Quick Keys:"));
  Serial.println(F("  ?|h help, s status, t toggle telemetry"));
  Serial.println(F("  e 0|1 enable driver, j <steps> <rate> jog"));
  Serial.println(F("  a zero angle, p zero position, z show zero"));
  Serial.println(F("  [|l|1 DOWN limit, ]|u|2 UP limit, m show limits"));
  Serial.println(F("  b sign begin, g sign save, v save EEPROM"));
  Serial.println(F("  o load EEPROM, d reset defaults, i guide"));
  Serial.println(F("  r run, k stop, f fault reset, x fault info"));
  Serial.println(F("Wizard Keys:"));
  Serial.println(F("  w start guided calibration, n next step"));
  Serial.println(F("  c confirm final save, q abort wizard"));
  Serial.println(F("Full Commands (compat):"));
  Serial.println(F("  help"));
  Serial.println(F("  status"));
  Serial.println(F("  telemetry 0|1"));
  Serial.println(F("  en 0|1"));
  Serial.println(F("  jog <signed_steps> <rate>"));
  Serial.println(F("  cal_zero set angle|position"));
  Serial.println(F("  cal_zero show|angle|position"));
  Serial.println(F("  cal_limits set down|up|lower|upper"));
  Serial.println(F("  cal_limits show"));
  Serial.println(F("  cal_sign begin|save"));
  Serial.println(F("  sonar diag | sonar sign 1|-1"));
  Serial.println(F("  cal_save | cal_load | cal_reset defaults"));
  Serial.println(F("  run | stop | fault_reset | faults | guide"));
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

bool handleCalSignBegin() {
  if (g_state_machine.state() == AppState::FAULT) {
    Serial.println(F("ERR,fault_active"));
    printFaultInfo();
    Serial.println(F("GUIDE,do=Move neutral then run a,p,f,b"));
    return false;
  }

  const uint32_t now_ms = millis();
  float theta_now_deg = 0.0f;
  float theta_raw_deg = 0.0f;
  if (!sampleAngleNow(now_ms, &theta_now_deg, &theta_raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
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

  float sonar_x_cm = 0.0f;
  float sonar_x_filt_cm = 0.0f;
  float sonar_dist_cm = 0.0f;
  g_sign_cal.near_captured = sampleSonarNow(now_ms, &sonar_x_cm, &sonar_x_filt_cm, &sonar_dist_cm);
  g_sign_cal.near_distance_cm = sonar_dist_cm;
  g_sign_cal.theta_before_raw_deg = theta_raw_deg;

  g_stepper.enable(true);
  g_stepper.requestJogSteps(SIGN_CAL_JOG_STEPS, SIGN_CAL_JOG_RATE_SPS);
  runJogBlocking(3000);

  delay(50);
  if (!sampleAngleNow(millis(), nullptr, &g_sign_cal.theta_after_raw_deg)) {
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

  g_state_machine.finishSignCalibration(true);

  Serial.println(F("INFO,cal_sign_begin_complete"));
  Serial.print(F("INFO,theta_raw_delta_deg="));
  Serial.println(g_sign_cal.theta_delta_raw_deg, 5);
  Serial.print(F("INFO,suggested_stepper_sign="));
  Serial.println(g_sign_cal.suggested_stepper_sign);
  Serial.print(F("INFO,suggested_as5600_sign="));
  Serial.println(g_sign_cal.suggested_as5600_sign);
  if (g_sign_cal.near_captured) {
    Serial.println(F("INFO,sonar_near_captured=yes; move ball far +x then run g"));
  } else {
    Serial.println(F("WARN,sonar_near_captured=no; run sonar sign +/-1 or rerun b when sonar stable"));
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
    Serial.println(F("GUIDE,do=Run sonar sign 1|-1 or rerun b after sonar stabilizes"));
    return false;
  }

  float x_cm = 0.0f;
  float x_filt_cm = 0.0f;
  float dist_cm = 0.0f;
  if (!sampleSonarNow(millis(), &x_cm, &x_filt_cm, &dist_cm)) {
    Serial.println(F("ERR,sonar_not_ready"));
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

  Serial.println(F("INFO,signs_applied_runtime; run cal_save to persist"));
  g_runtime_cal_dirty = true;
  return true;
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
    Serial.println(F("FAULT_INFO,sonar_timeout=1,do=Fix HC-SR04 visibility/wiring"));
  }
  if (g_fault_flags.i2c_error) {
    Serial.println(F("FAULT_INFO,i2c_error=1,do=Check AS5600 wiring/power"));
  }
  if (g_fault_flags.angle_oob) {
    Serial.println(F("FAULT_INFO,angle_oob=1,do=Level beam then run a"));
  }
  if (g_fault_flags.pos_oob) {
    Serial.println(F("FAULT_INFO,pos_oob=1,do=Center target then run p"));
  }
  Serial.println(F("FAULT_INFO,recover=neutral->a->p->f->s->x"));
}

void printBringupMenu() {
  Serial.println(F("MENU,bringup"));
  Serial.print(F("MENU,state="));
  Serial.print(stateToString(g_state_machine.state()));
  Serial.print(F(",zero="));
  Serial.print(runtimeCalIsZeroSet() ? F("yes") : F("no"));
  Serial.print(F(",limits="));
  Serial.print(runtimeCalIsLimitsSet() ? F("yes") : F("no"));
  Serial.print(F(",sign="));
  Serial.print(runtimeCalIsSignSet() ? F("yes") : F("no"));
  Serial.print(F(",angle_ok="));
  Serial.print(g_sensor.valid_angle ? F("yes") : F("no"));
  Serial.print(F(",sonar_ok="));
  Serial.println(g_sensor.valid_pos ? F("yes") : F("no"));
  Serial.println(F("STEP,flow,t->a->p->[down]->](up)->b->(g|sonar sign)->v->r"));
  Serial.println(F("STEP,down_up,do=Capture physical DOWN then physical UP; numeric order auto-normalized"));
  if (!g_sensor.valid_pos) {
    Serial.println(F("STEP,sonar,do=Use flat target and run sonar diag"));
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

bool captureZeroAngle() {
  float theta_deg = 0.0f;
  float raw_deg = 0.0f;
  if (!sampleAngleNow(millis(), &theta_deg, &raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
    return false;
  }
  runtimeCalSetAs5600ZeroDeg(raw_deg);
  runtimeCalMarkZeroAngleCaptured(true);
  sampleAngleNow(millis(), nullptr, nullptr);
  g_runtime_cal_dirty = true;
  Serial.print(F("OK,cal_zero_angle_set="));
  Serial.println(raw_deg, 6);
  printCalZero();
  return true;
}

bool captureZeroPosition() {
  float x_cm = 0.0f;
  float x_filt_cm = 0.0f;
  float dist_cm = 0.0f;
  if (!sampleSonarNow(millis(), &x_cm, &x_filt_cm, &dist_cm)) {
    Serial.println(F("ERR,sonar_not_ready"));
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
  float theta_deg = 0.0f;
  if (!sampleAngleNow(millis(), &theta_deg, nullptr)) {
    Serial.println(F("ERR,angle_not_ready"));
    return false;
  }

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
  float theta_deg = 0.0f;
  if (!sampleAngleNow(millis(), &theta_deg, nullptr)) {
    Serial.println(F("ERR,angle_not_ready"));
    return false;
  }

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
  float theta_deg = 0.0f;
  if (!sampleAngleNow(millis(), &theta_deg, nullptr)) {
    Serial.println(F("ERR,angle_not_ready"));
    return false;
  }

  runtimeCalSetThetaLowerLimitDeg(theta_deg);
  runtimeCalMarkLowerLimitCaptured(true);
  g_runtime_cal_dirty = true;

  Serial.print(F("OK,cal_limit_down_set="));
  Serial.println(theta_deg, 6);
  if (runtimeCalIsUpperLimitCaptured()) {
    if (!finalizeNormalizedLimitSpan()) {
      return false;
    }
    Serial.println(F("INFO,limits_normalized_from_down_up"));
  }
  printCalLimits();
  return true;
}

bool captureUpLimit() {
  float theta_deg = 0.0f;
  if (!sampleAngleNow(millis(), &theta_deg, nullptr)) {
    Serial.println(F("ERR,angle_not_ready"));
    return false;
  }

  runtimeCalSetThetaUpperLimitDeg(theta_deg);
  runtimeCalMarkUpperLimitCaptured(true);
  g_runtime_cal_dirty = true;

  Serial.print(F("OK,cal_limit_up_set="));
  Serial.println(theta_deg, 6);
  if (runtimeCalIsLowerLimitCaptured()) {
    if (!finalizeNormalizedLimitSpan()) {
      return false;
    }
    Serial.println(F("INFO,limits_normalized_from_down_up"));
  }
  printCalLimits();
  return true;
}

bool suggestZeroAngle() {
  float theta_deg = 0.0f;
  float raw_deg = 0.0f;
  if (!sampleAngleNow(millis(), &theta_deg, &raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
    return false;
  }
  Serial.print(F("CAL_SUGGEST,AS5600_ZERO_DEG="));
  Serial.println(raw_deg, 6);
  return true;
}

bool suggestZeroPosition() {
  float x_cm = 0.0f;
  float x_filt_cm = 0.0f;
  float dist_cm = 0.0f;
  if (!sampleSonarNow(millis(), &x_cm, &x_filt_cm, &dist_cm)) {
    Serial.println(F("ERR,sonar_not_ready"));
    return false;
  }
  Serial.print(F("CAL_SUGGEST,SONAR_CENTER_CM="));
  Serial.println(dist_cm, 6);
  return true;
}

void printGuideNextAction() {
  if (g_wizard.active) {
    return;
  }

  if (!runtimeCalIsZeroAngleCaptured()) {
    Serial.println(F("GUIDE,step=zero_angle,do=Level beam parallel, then press a (or cal_zero set angle)"));
    return;
  }
  if (!runtimeCalIsZeroPosCaptured()) {
    Serial.println(F("GUIDE,step=zero_position,do=Place ball at center, then press p (or cal_zero set position)"));
    return;
  }
  if (!runtimeCalIsLowerLimitCaptured()) {
    Serial.println(F("GUIDE,step=down_limit,do=Move beam to physical DOWN stop, then press [ (or l)"));
    return;
  }
  if (!runtimeCalIsUpperLimitCaptured()) {
    Serial.println(F("GUIDE,step=up_limit,do=Move beam to physical UP stop, then press ] (or u)"));
    return;
  }
  if (!runtimeCalHasValidLimitSpan()) {
    Serial.println(F("GUIDE,step=limits_fix,do=Recapture physical DOWN/UP using [ and ]"));
    return;
  }
  if (!runtimeCalIsSignSet()) {
    Serial.println(F("GUIDE,step=sign_begin,do=Run b to calibrate stepper+AS5600 signs"));
    return;
  }
  if (g_sign_cal.active && !g_sign_cal.complete) {
    if (g_sign_cal.near_captured) {
      Serial.println(F("GUIDE,step=sonar_sign_auto,do=Move target far +x then run g"));
    } else {
      Serial.println(F("GUIDE,step=sonar_sign_manual,do=Run sonar sign 1|-1 or rerun b when sonar stable"));
    }
    return;
  }
  if (g_state_machine.state() == AppState::FAULT) {
    Serial.println(F("GUIDE,step=fault_reset,do=Fix issue then run f"));
    return;
  }
  if (!g_sensor.valid_angle) {
    Serial.println(F("GUIDE,step=angle_sensor,do=Check AS5600 and rerun s"));
    return;
  }
  if (!g_sensor.valid_pos) {
    Serial.println(F("GUIDE,step=sonar_sensor,do=Use flat target and run sonar diag"));
    return;
  }
  if (!g_inner_loop_stable) {
    Serial.println(F("GUIDE,step=inner_loop,do=Inner loop unstable; do not run until stabilized"));
    return;
  }
  if (g_runtime_cal_dirty) {
    Serial.println(F("GUIDE,step=save,do=Persist calibration with v (or cal_save)"));
    return;
  }
  Serial.println(F("GUIDE,step=run,do=Press r (or run) to start balancing"));
}

void printRunBlockedDetails(const AppConditions& cond) {
  Serial.println(F("ERR,run_blocked"));
  if (!cond.zero_calibrated) {
    Serial.println(F("BLOCK,zero"));
  }
  if (!cond.limits_calibrated) {
    Serial.println(F("BLOCK,limits"));
  }
  if (!cond.sign_calibrated) {
    Serial.println(F("BLOCK,sign"));
  }
  if (!cond.sensors_ok) {
    Serial.println(F("BLOCK,sensors"));
  }
  if (cond.faults_active) {
    Serial.println(F("BLOCK,faults"));
  }
  if (!cond.inner_loop_stable) {
    Serial.println(F("BLOCK,inner_loop"));
  }
  printGuideNextAction();
}

void printWizardStepPrompt() {
  switch (g_wizard.step_index) {
    case kWizardStepZeroAngle:
      Serial.println(F("WIZ,step=1,do=Set beam parallel to ground; then press n"));
      Serial.println(F("WIZ,hint=This captures angle zero (same as a)"));
      return;
    case kWizardStepZeroPosition:
      Serial.println(F("WIZ,step=2,do=Place ball at center; then press n"));
      Serial.println(F("WIZ,hint=This captures sonar center (same as p)"));
      return;
    case kWizardStepLowerLimit:
      Serial.println(F("WIZ,step=3,do=Move beam to physical DOWN stop; then press n"));
      Serial.println(F("WIZ,hint=This captures DOWN limit (same as [)"));
      return;
    case kWizardStepUpperLimit:
      Serial.println(F("WIZ,step=4,do=Move beam to physical UP stop; then press n"));
      Serial.println(F("WIZ,hint=This captures UP limit (same as ])"));
      return;
    case kWizardStepSignBegin:
      Serial.println(F("WIZ,step=5,do=Keep ball at near reference end; then press n"));
      Serial.println(F("WIZ,hint=This runs sign-begin jog (same as b)"));
      return;
    case kWizardStepSignSave:
      Serial.println(F("WIZ,step=6,do=Move ball to far +x end; then press n"));
      Serial.println(F("WIZ,hint=This saves sign direction (same as g)"));
      return;
    case kWizardStepSaveConfirm:
      Serial.print(F("WIZ,summary,zero="));
      Serial.print(runtimeCalIsZeroSet() ? F("yes") : F("no"));
      Serial.print(F(",limits="));
      Serial.print(runtimeCalIsLimitsSet() ? F("yes") : F("no"));
      Serial.print(F(",sign="));
      Serial.print(runtimeCalIsSignSet() ? F("yes") : F("no"));
      Serial.print(F(",dirty="));
      Serial.println(g_runtime_cal_dirty ? F("yes") : F("no"));
      Serial.println(F("WIZ,step=7,do=Press c to save to EEPROM, or q to exit without save"));
      return;
    default:
      return;
  }
}

void stopWizard(const char* reason) {
  if (!g_wizard.active) {
    return;
  }
  if (g_state_machine.state() == AppState::CALIB_SIGN) {
    g_state_machine.requestStop();
  }
  g_sign_cal.active = false;
  const bool telemetry_restore = g_wizard.telemetry_prev;
  g_wizard = GuidedWizardSession{};
  g_telemetry_enabled = telemetry_restore;
  Serial.print(F("WIZ,exit,reason="));
  Serial.print(reason);
  Serial.print(F(",telemetry_restored="));
  Serial.println(g_telemetry_enabled ? F("1") : F("0"));
}

void startWizard() {
  if (g_wizard.active) {
    Serial.println(F("ERR,wizard_already_active"));
    printWizardStepPrompt();
    return;
  }
  if (g_state_machine.state() == AppState::RUNNING) {
    Serial.println(F("ERR,stop_running_before_wizard"));
    return;
  }
  if (g_state_machine.state() == AppState::FAULT) {
    Serial.println(F("ERR,fault_active"));
    printFaultInfo();
    Serial.println(F("WIZ,blocked,do=Move neutral then run a,p,f,s,w"));
    return;
  }

  g_wizard.active = true;
  g_wizard.step_index = kWizardStepZeroAngle;
  g_wizard.awaiting_save_confirm = false;
  g_wizard.telemetry_prev = g_telemetry_enabled;
  g_telemetry_enabled = false;

  Serial.println(F("OK,wizard_started"));
  Serial.print(F("WIZ,telemetry_auto_disabled,prev="));
  Serial.println(g_wizard.telemetry_prev ? F("1") : F("0"));
  Serial.println(F("WIZ,controls=n(next),c(confirm save),q(quit)"));
  printWizardStepPrompt();
}

void wizardNext() {
  if (!g_wizard.active) {
    Serial.println(F("ERR,wizard_not_active"));
    return;
  }

  bool success = false;
  switch (g_wizard.step_index) {
    case kWizardStepZeroAngle:
      success = captureZeroAngle();
      break;
    case kWizardStepZeroPosition:
      success = captureZeroPosition();
      break;
    case kWizardStepLowerLimit:
      success = captureDownLimit();
      break;
    case kWizardStepUpperLimit:
      success = captureUpLimit();
      break;
    case kWizardStepSignBegin:
      success = handleCalSignBegin();
      break;
    case kWizardStepSignSave:
      success = handleCalSignSave();
      break;
    case kWizardStepSaveConfirm:
      Serial.println(F("WIZ,awaiting_confirm,press=c_to_save_or_q_to_exit"));
      return;
    default:
      Serial.println(F("ERR,wizard_invalid_step"));
      return;
  }

  if (!success) {
    Serial.println(F("WIZ,step_failed,fix_and_press_n_again"));
    return;
  }

  if (g_wizard.step_index < kWizardStepSaveConfirm) {
    ++g_wizard.step_index;
  }
  if (g_wizard.step_index >= kWizardStepSaveConfirm) {
    g_wizard.step_index = kWizardStepSaveConfirm;
    g_wizard.awaiting_save_confirm = true;
  }
  printWizardStepPrompt();
}

void wizardConfirmSave() {
  if (!g_wizard.active) {
    Serial.println(F("ERR,wizard_not_active"));
    return;
  }
  if (!g_wizard.awaiting_save_confirm || g_wizard.step_index != kWizardStepSaveConfirm) {
    Serial.println(F("ERR,wizard_not_at_save_step"));
    return;
  }

  if (runtimeCalSave()) {
    g_runtime_cal_dirty = false;
    Serial.println(F("OK,wizard_saved"));
    stopWizard("saved");
    printGuideNextAction();
  } else {
    Serial.println(F("ERR,cal_save_failed"));
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
        printBringupMenu();
        printGuideNextAction();
        return;
      case 's':
        printStatus();
        printFaultInfo();
        printGuideNextAction();
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
      case 'e': {
        char* arg = strtok_r(nullptr, " ", &saveptr);
        if (arg == nullptr) {
          Serial.println(F("ERR,usage:e 0|1"));
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
      case 'j': {
        char* steps_arg = strtok_r(nullptr, " ", &saveptr);
        char* rate_arg = strtok_r(nullptr, " ", &saveptr);
        if (steps_arg == nullptr || rate_arg == nullptr) {
          Serial.println(F("ERR,usage:j <signed_steps> <rate>"));
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
      case 'l':
      case '1':
        if (captureDownLimit()) {
          printGuideNextAction();
        }
        return;
      case ']':
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
        g_runtime_cal_dirty = false;
        Serial.print(F("OK,cal_load,"));
        Serial.println(loaded ? F("loaded") : F("defaults"));
        printGuideNextAction();
        return;
      }
      case 'd':
        runtimeCalResetDefaults();
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
      case 'w':
        startWizard();
        return;
      case 'n':
        wizardNext();
        return;
      case 'q':
        if (!g_wizard.active) {
          Serial.println(F("ERR,wizard_not_active"));
          return;
        }
        stopWizard("aborted");
        printGuideNextAction();
        return;
      case 'c':
        wizardConfirmSave();
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
        Serial.println(F("ERR,usage:cal_zero set angle|position"));
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
      printGuideNextAction();
      return;
    }

    if (strcmp(sub, "set") == 0) {
      char* edge = strtok_r(nullptr, " ", &saveptr);
      if (edge == nullptr) {
        Serial.println(F("ERR,usage:cal_limits set lower|upper"));
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

      Serial.println(F("ERR,usage:cal_limits set lower|upper"));
      return;
    }

    Serial.println(F("ERR,usage:cal_limits set lower|upper | cal_limits show"));
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
    g_runtime_cal_dirty = false;
    Serial.print(F("OK,cal_load,"));
    Serial.println(loaded ? F("loaded") : F("defaults"));
    printGuideNextAction();
    return;
  }

  if (strcmp(token, "cal_reset") == 0) {
    char* sub = strtok_r(nullptr, " ", &saveptr);
    if (sub == nullptr || strcmp(sub, "defaults") != 0) {
      Serial.println(F("ERR,usage:cal_reset defaults"));
      return;
    }
    runtimeCalResetDefaults();
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
    printGuideNextAction();
    return;
  }

  if (strcmp(token, "guide") == 0) {
    printBringupMenu();
    printGuideNextAction();
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
  printBringupMenu();
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
