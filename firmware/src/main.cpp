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
constexpr float kRunSoftLimitBandDeg = 1.0f;
constexpr float kRunHardLimitMarginDeg = 2.0f;
constexpr uint32_t kLimitRecoveryHoldMs = 300UL;
constexpr float kLimitRecoveryMoveCm = 0.4f;
constexpr long kLimitRecoveryNudgeSteps = 120;
constexpr float kLimitRecoveryNudgeRateSps = 250.0f;
constexpr float kTrimLearnXWindowCm = 0.8f;
constexpr float kTrimLearnThetaWindowDeg = 0.5f;
constexpr float kTrimLearnMaxStepRateSps = 80.0f;
constexpr uint32_t kTrimLearnStableMs = 500UL;
constexpr float kTrimLearnRateDegPerSec = 0.2f;
constexpr float kTrimGuardBandDeg = 1.0f;

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
float g_prev_step_rate_sps = 0.0f;  // slew-rate limiter state

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

// Raw AS5600 readings captured at physical lower/upper beam positions.
// Used by finalizeNormalizedLimitSpan() to auto-compute AS5600 sign.
float g_limit_lower_raw_deg = 0.0f;
float g_limit_upper_raw_deg = 0.0f;

// Sonar raw distance at each limit (for auto sonar-sign derivation).
float g_limit_lower_sonar_cm = 0.0f;
float g_limit_upper_sonar_cm = 0.0f;

bool g_runtime_cal_dirty = false;
bool g_limit_stuck_fault = false;

enum class LimitRecoveryPhase : uint8_t {
  kIdle = 0,
  kHold,
  kNudging,
  kPostNudgeHold
};

struct LimitRecoveryState {
  LimitRecoveryPhase phase = LimitRecoveryPhase::kIdle;
  int8_t side = 0;  // -1 = lower, +1 = upper
  bool nudge_used = false;
  uint32_t phase_start_ms = 0;
  float entry_abs_x_cm = 0.0f;
};

LimitRecoveryState g_limit_recovery;

struct TrimLearningState {
  uint32_t stable_since_ms = 0;
};

TrimLearningState g_trim_learning;

void printFaultInfo();
void printSonarDiag();
bool captureSonarCalDistanceCm(float& out_cm);
bool readAs5600RawMedianDeg(float& out_raw_deg);
bool captureAs5600CalRawDeg(float& out_raw_deg);
void applySafeDisable();
void maybeStopJogAtLimits();
void serviceControl(uint32_t now_ms);
void resetLimitRecovery();
void resetTrimLearning();
void syncLimitCaptureCacheFromRuntime();
void serviceTrimLearning(uint32_t now_ms, float signed_step_rate_sps);
bool serviceActiveLimitRecovery(uint32_t now_ms);
bool maybeStartLimitRecovery(uint32_t now_ms, float signed_step_rate_sps);

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

void resetLimitRecovery() { g_limit_recovery = LimitRecoveryState{}; }

void resetTrimLearning() { g_trim_learning = TrimLearningState{}; }

void syncLimitCaptureCacheFromRuntime() {
  g_limit_lower_raw_deg = runtimeCalAs5600LowerRawDeg();
  g_limit_upper_raw_deg = runtimeCalAs5600UpperRawDeg();
  g_limit_lower_sonar_cm = runtimeCalSonarLowerCm();
  g_limit_upper_sonar_cm = runtimeCalSonarUpperCm();
}

float currentActuatorDeg() {
  return g_sensor.beam_angle_deg + runtimeCalActiveActuatorTrimDeg();
}

float clampActuatorTrimDeg(float trim_deg) {
  const float span = runtimeCalActuatorSpanDeg();
  if (span <= 0.0f) {
    return 0.0f;
  }
  if (span <= (2.0f * kTrimGuardBandDeg)) {
    return 0.5f * span;
  }
  return clampf(trim_deg, kTrimGuardBandDeg, span - kTrimGuardBandDeg);
}

bool ballMovedCenterwardEnough() {
  if (!g_sensor.valid_pos) {
    return false;
  }
  return (g_limit_recovery.entry_abs_x_cm - fabsf(g_sensor.ball_pos_filt_cm)) >=
         kLimitRecoveryMoveCm;
}

void triggerLimitStuckFault() {
  g_limit_stuck_fault = true;
  g_prev_step_rate_sps = 0.0f;
  resetLimitRecovery();
  g_state_machine.requestFault();
  applySafeDisable();
}

bool serviceActiveLimitRecovery(uint32_t now_ms) {
  if (g_limit_recovery.phase == LimitRecoveryPhase::kIdle) {
    return false;
  }

  if (ballMovedCenterwardEnough()) {
    resetLimitRecovery();
    return false;
  }

  if (g_limit_recovery.phase == LimitRecoveryPhase::kNudging) {
    maybeStopJogAtLimits();
    if (g_stepper.jogActive()) {
      g_stepper.enable(true);
      return true;
    }
    g_limit_recovery.phase = LimitRecoveryPhase::kPostNudgeHold;
    g_limit_recovery.phase_start_ms = now_ms;
  }

  g_prev_step_rate_sps = 0.0f;
  g_stepper.enable(true);
  g_stepper.setSignedStepRate(0.0f);

  if (static_cast<uint32_t>(now_ms - g_limit_recovery.phase_start_ms) < kLimitRecoveryHoldMs) {
    return true;
  }

  if (!g_limit_recovery.nudge_used) {
    g_limit_recovery.nudge_used = true;
    g_limit_recovery.phase = LimitRecoveryPhase::kNudging;
    g_limit_recovery.phase_start_ms = now_ms;
    g_controller.reset();
    g_prev_step_rate_sps = 0.0f;
    g_stepper.enable(true);
    g_stepper.requestJogSteps(
        (g_limit_recovery.side > 0) ? -kLimitRecoveryNudgeSteps : kLimitRecoveryNudgeSteps,
        kLimitRecoveryNudgeRateSps);
    return true;
  }

  triggerLimitStuckFault();
  return true;
}

bool maybeStartLimitRecovery(uint32_t now_ms, float signed_step_rate_sps) {
  if (g_limit_recovery.phase != LimitRecoveryPhase::kIdle ||
      !runtimeCalIsLimitsSet() || !g_sensor.valid_angle) {
    return false;
  }

  int8_t side = 0;
  const float theta_deg = g_sensor.beam_angle_deg;
  if (signed_step_rate_sps > 0.0f &&
      theta_deg >= (runtimeCalThetaUpperLimitDeg() - kRunSoftLimitBandDeg)) {
    side = 1;
  } else if (signed_step_rate_sps < 0.0f &&
             theta_deg <= (runtimeCalThetaLowerLimitDeg() + kRunSoftLimitBandDeg)) {
    side = -1;
  }

  if (side == 0) {
    return false;
  }

  g_limit_recovery.phase = LimitRecoveryPhase::kHold;
  g_limit_recovery.side = side;
  g_limit_recovery.nudge_used = false;
  g_limit_recovery.phase_start_ms = now_ms;
  g_limit_recovery.entry_abs_x_cm = fabsf(g_sensor.ball_pos_filt_cm);
  return serviceActiveLimitRecovery(now_ms);
}

void serviceTrimLearning(uint32_t now_ms, float signed_step_rate_sps) {
  if (!runtimeCalIsLimitsSet() || !g_sensor.valid_angle || !g_sensor.valid_pos) {
    resetTrimLearning();
    return;
  }

  if (fabsf(g_sensor.ball_pos_filt_cm) > kTrimLearnXWindowCm ||
      fabsf(g_controller.lastThetaCmdDeg() - g_sensor.beam_angle_deg) > kTrimLearnThetaWindowDeg ||
      fabsf(signed_step_rate_sps) > kTrimLearnMaxStepRateSps) {
    resetTrimLearning();
    return;
  }

  if (g_trim_learning.stable_since_ms == 0u) {
    g_trim_learning.stable_since_ms = now_ms;
    return;
  }

  if (static_cast<uint32_t>(now_ms - g_trim_learning.stable_since_ms) < kTrimLearnStableMs) {
    return;
  }

  const float act_deg = currentActuatorDeg();
  const float trim_now = runtimeCalActiveActuatorTrimDeg();
  const float max_step = kTrimLearnRateDegPerSec * kControlDtSec;
  float delta = act_deg - trim_now;
  if (delta > max_step) {
    delta = max_step;
  } else if (delta < -max_step) {
    delta = -max_step;
  }

  if (fabsf(delta) < 1.0e-4f) {
    return;
  }

  runtimeCalSetActuatorTrimDeg(clampActuatorTrimDeg(trim_now + delta));
  runtimeCalSetActuatorTrimValid(true);
  runtimeCalSetActuatorTrimSource(ActuatorTrimSource::kLearned);
  runtimeCalMarkZeroAngleCaptured(true);
  g_runtime_cal_dirty = true;
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
  // Wider running threshold for sensor dropout tolerance.
  return (g_state_machine.state() == AppState::RUNNING)
             ? 1500UL
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

// Forward declarations for tryStartRun.
bool readSensors(uint32_t now_ms);
void updateFaultState(uint32_t now_ms);
AppConditions currentConditions();
void printRunBlockedDetails(const AppConditions& cond);

bool tryStartRun() {
  readSensors(millis());
  updateFaultState(millis());

  if (runtimeCalIsLimitsSet() && g_sensor.valid_angle) {
    const float theta_deg = g_sensor.beam_angle_deg;
    if (theta_deg < runtimeCalThetaLowerLimitDeg() || theta_deg > runtimeCalThetaUpperLimitDeg()) {
      Serial.print(F("ERR,act_range,theta="));
      Serial.println(theta_deg, 1);
      return false;
    }
  }

  if (g_fault_flags.angle_oob) {
    Serial.println(F("ERR,angle_oob"));
    return false;
  }

  const AppConditions cond = currentConditions();
  if (g_state_machine.requestRun(cond)) {
    g_controller.reset();
    g_prev_step_rate_sps = 0.0f;
    g_limit_stuck_fault = false;
    resetLimitRecovery();
    resetTrimLearning();
    g_telemetry_enabled = true;
    g_setpoint.ball_pos_cm_target = kDefaultBallSetpointCm;
    g_setpoint.ball_pos_m_target = g_setpoint.ball_pos_cm_target * kCmToM;
    Serial.println(F("OK,running"));
    return true;
  }
  printRunBlockedDetails(cond);
  return false;
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
  if (runtimeCalIsLimitsSet()) {
    g_fault_flags.angle_oob =
        (theta_deg < (runtimeCalThetaLowerLimitDeg() - kRunHardLimitMarginDeg)) ||
        (theta_deg > (runtimeCalThetaUpperLimitDeg() + kRunHardLimitMarginDeg));
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
  sampleAngleNow(now_ms);
  // Hold last-good AS5600 value until it becomes stale (matches sonar policy).
  g_sensor.valid_angle =
      g_as5600_last_initialized &&
      (static_cast<uint32_t>(now_ms - g_last_valid_angle_ms) <= kPosSampleFreshMs);

  sampleSonarNow(now_ms);
  return g_sensor.valid_angle && g_sensor.valid_pos;
}

void updateFaultState(uint32_t now_ms) {
  const uint32_t stale_threshold_ms = activeStaleThresholdMs();

  if (static_cast<uint32_t>(now_ms - g_last_valid_angle_ms) > stale_threshold_ms) {
    g_fault_flags.i2c_error = true;
  }

  if (static_cast<uint32_t>(now_ms - g_last_valid_pos_ms) > stale_threshold_ms) {
    g_fault_flags.sonar_timeout = true;
  }

  if (g_state_machine.state() == AppState::RUNNING && hasAnyFault()) {
    resetLimitRecovery();
    resetTrimLearning();
    g_state_machine.requestFault();
    g_prev_step_rate_sps = 0.0f;
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

  if (!runtimeCalIsLimitsSet() || !g_sensor.valid_angle) {
    return;
  }

  const float theta_deg = g_sensor.beam_angle_deg;

  if (rate > 0.0f && theta_deg >= (runtimeCalThetaUpperLimitDeg() - kLimitStopMarginDeg)) {
    g_stepper.stop();
    Serial.println(F("WARN,jog_upper"));
    return;
  }

  if (rate < 0.0f && theta_deg <= (runtimeCalThetaLowerLimitDeg() + kLimitStopMarginDeg)) {
    g_stepper.stop();
    Serial.println(F("WARN,jog_lower"));
  }
}

void printHelp() {
  Serial.println(F("HELP,keys"));
  // Keep on-device help minimal to save flash; use the host-side serial logger
  // (analysis/serial_logger.py) for run logs and copy/paste workflows.
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

  Serial.print(F("SONAR_CFG,s="));
  Serial.print(runtimeCalSonarPosSign());
  Serial.print(F(",m="));
  Serial.print(runtimeCalSonarSignModeName(runtimeCalSonarSignMode()));
  Serial.print(F(",u="));
  Serial.println(runtimeCalUpperLimitNearSensor() ? F("1") : F("0"));

  Serial.print(F("ACT_CFG,t="));
  Serial.print(runtimeCalActiveActuatorTrimDeg(), 4);
  Serial.print(F(",s="));
  Serial.print(runtimeCalActuatorTrimSourceName(runtimeCalActuatorTrimSource()));
  Serial.print(F(",c="));
  Serial.println(runtimeCalSonarCenterSourceName(runtimeCalSonarCenterSource()));

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
    const uint32_t now_ms = millis();

    g_stepper.processIsrFlags();
    const uint32_t jog_now_us = micros();
    g_hcsr04.service(jog_now_us, now_ms);
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
  // Requires limits already captured (which auto-set AS5600 sign).
  if (!runtimeCalIsLimitsSet()) {
    Serial.println(F("ERR,capture_l_u_first"));
    return false;
  }

  float raw_before = 0.0f;
  if (!captureAs5600CalRawDeg(raw_before)) {
    Serial.println(F("ERR,angle_not_ready"));
    return false;
  }

  if (!g_state_machine.startSignCalibration()) {
    Serial.println(F("ERR,cannot_enter_calib_sign"));
    return false;
  }

  // Initialise sign-cal session for later g (save) command.
  g_sign_cal.active = true;
  g_sign_cal.near_captured = false;
  g_sign_cal.complete = false;
  g_sign_cal.suggested_stepper_sign = runtimeCalStepperDirSign();
  g_sign_cal.suggested_as5600_sign = runtimeCalAs5600Sign();
  g_sign_cal.suggested_sonar_sign = runtimeCalOrientationSonarSign();
  g_sign_cal.theta_before_raw_deg = raw_before;

  // Capture near sonar distance (ball at near end before jog).
  float sonar_dist_cm = 0.0f;
  g_sign_cal.near_captured = captureSonarCalDistanceCm(sonar_dist_cm);
  g_sign_cal.near_distance_cm = sonar_dist_cm;

  // Temporarily clear limits so test jog isn't blocked.
  runtimeCalMarkLowerLimitCaptured(false);
  runtimeCalMarkUpperLimitCaptured(false);

  // Test jog: 200 usteps at 200 sps (~3.3° beam — well above noise).
  g_stepper.enable(true);
  g_stepper.requestJogSteps(200, 200.0f);
  runJogBlocking(3000);
  delay(100);

  float raw_after = 0.0f;
  const bool got_after = captureAs5600CalRawDeg(raw_after);

  // Jog back to original position.
  g_stepper.requestJogSteps(-200, 200.0f);
  runJogBlocking(3000);

  // Restore limits.
  runtimeCalMarkLowerLimitCaptured(true);
  runtimeCalMarkUpperLimitCaptured(true);

  if (!got_after) {
    Serial.println(F("ERR,angle_not_ready"));
    g_sign_cal.active = false;
    g_state_machine.finishSignCalibration(false);
    if (!g_manual_enable) { applySafeDisable(); }
    return false;
  }

  g_sign_cal.theta_after_raw_deg = raw_after;

  // Positive jog should increase theta (motor up = positive).
  const float theta_before = runtimeMapThetaDeg(raw_before);
  const float theta_after = runtimeMapThetaDeg(raw_after);
  const float delta_deg = theta_after - theta_before;
  g_sign_cal.theta_delta_raw_deg = delta_deg;

  if (delta_deg < 0.0f) {
    runtimeCalSetStepperDirSign(static_cast<int8_t>(-runtimeCalStepperDirSign()));
  }
  g_sign_cal.suggested_stepper_sign = runtimeCalStepperDirSign();
  g_sign_cal.suggested_as5600_sign = runtimeCalAs5600Sign();

  runtimeCalSetSignCaptured(true);
  g_runtime_cal_dirty = true;
  resetAs5600Filter();

  g_state_machine.finishSignCalibration(true);

  Serial.print(F("OK,stepper_sign="));
  Serial.print(runtimeCalStepperDirSign());
  Serial.print(F(",delta="));
  Serial.println(delta_deg, 4);

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
  const float delta_cm = g_sign_cal.far_distance_cm - g_sign_cal.near_distance_cm;
  g_sign_cal.suggested_sonar_sign = (delta_cm >= 0.0f) ? 1 : -1;

  g_sign_cal.complete = true;
  g_sign_cal.active = false;

  bool applied = false;
  if (fabsf(delta_cm) < 1.0f) {
    Serial.println(F("WARN,sign_sonar_delta"));
  } else if (g_sign_cal.suggested_sonar_sign != runtimeCalOrientationSonarSign()) {
    Serial.println(F("WARN,sign_flip"));
  } else {
    runtimeCalApplyOrientationSonarSign();
    applied = true;
  }

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
  Serial.println(runtimeCalSonarPosSign());

  Serial.println(applied ? F("INFO,signs_applied_runtime")
                         : F("INFO,sign_keep"));
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
  Serial.print(runtimeCalActiveActuatorTrimDeg(), 6);
  Serial.print(F(",act_trim_src="));
  Serial.print(runtimeCalActuatorTrimSourceName(runtimeCalActuatorTrimSource()));
  Serial.print(F(",sonar_center_cm="));
  Serial.print(runtimeCalActiveSonarCenterCm(), 6);
  Serial.print(F(",sonar_center_src="));
  Serial.print(runtimeCalSonarCenterSourceName(runtimeCalSonarCenterSource()));
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
  if (bits == 0 && !g_limit_stuck_fault) {
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
  if (g_limit_stuck_fault) {
    Serial.println(F("FAULT_INFO,stuck=1"));
  }
  if (g_state_machine.state() == AppState::FAULT) {
    Serial.println(F("FAULT_INFO,recover=f"));
  }
}

void printSonarDiag() {
  SonarDiag diag;
  g_hcsr04.getDiag(millis(), diag);
  Serial.print(F("SONAR_DIAG,fresh="));
  Serial.print(diag.fresh ? F("1") : F("0"));
  Serial.print(F(",age_ms="));
  Serial.print(diag.age_ms);
  Serial.print(F(",raw_cm="));
  Serial.print(diag.raw_cm, 4);
  Serial.print(F(",filt_cm="));
  Serial.print(diag.filt_cm, 4);
  Serial.print(F(",jump_reject_cnt="));
  Serial.println(diag.jump_reject_count);
}

bool captureSonarCalDistanceCm(float& out_cm) {
  // Calibration needs a stable distance even when the target is a weak reflector.
  // Take multiple fresh filtered samples and smooth them again with EMA.
  constexpr uint8_t kNeedGood = 8;
  constexpr uint32_t kTimeoutMs = 1500;
  constexpr uint32_t kGoodDelayMs = 45;
  constexpr uint32_t kBadDelayMs = 10;
  constexpr float kMaxSpanCm = 0.8f;

  uint8_t good = 0;
  float ema = 0.0f;
  bool ema_init = false;
  float min_v = 0.0f;
  float max_v = 0.0f;

  const uint32_t start_ms = millis();
  while (good < kNeedGood && static_cast<uint32_t>(millis() - start_ms) < kTimeoutMs) {
    const uint32_t now_ms = millis();

    g_stepper.processIsrFlags();
    const uint32_t cal_now_us = micros();
    g_hcsr04.service(cal_now_us, now_ms);

    SonarDiag diag;
    g_hcsr04.getDiag(now_ms, diag);

    // Only accept true fresh samples (reject "held" values after a timeout).
    if (diag.fresh && diag.has_sample && !diag.timeout) {
      const float v = (diag.filt_cm > 0.0f) ? diag.filt_cm : diag.raw_cm;
      if (v > 0.0f) {
        if (!ema_init) {
          ema = v;
          ema_init = true;
          min_v = v;
          max_v = v;
        } else {
          ema = (kSonarEmaAlpha * v) + ((1.0f - kSonarEmaAlpha) * ema);
          if (v < min_v) {
            min_v = v;
          }
          if (v > max_v) {
            max_v = v;
          }
        }
        ++good;
        delay(kGoodDelayMs);
        continue;
      }
    }

    delay(kBadDelayMs);
  }

  if (!ema_init || good < kNeedGood || ((max_v - min_v) > kMaxSpanCm)) {
    return false;
  }
  out_cm = ema;
  return true;
}

bool readAs5600RawMedianDeg(float& out_raw_deg) {
  // Cheap outlier rejection for calibration: median-of-3 raw samples, with wrap handling.
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

  // Unwrap around 'a' so values near 0/4095 don't break the median.
  int16_t ai = static_cast<int16_t>(a);
  int16_t bi = static_cast<int16_t>(b);
  int16_t ci = static_cast<int16_t>(c);

  int16_t d = static_cast<int16_t>(bi - ai);
  if (d > 2048) {
    bi = static_cast<int16_t>(bi - 4096);
  } else if (d < -2048) {
    bi = static_cast<int16_t>(bi + 4096);
  }

  d = static_cast<int16_t>(ci - ai);
  if (d > 2048) {
    ci = static_cast<int16_t>(ci - 4096);
  } else if (d < -2048) {
    ci = static_cast<int16_t>(ci + 4096);
  }

  // Median-of-3 on int16.
  if (ai > bi) {
    const int16_t t = ai;
    ai = bi;
    bi = t;
  }
  if (bi > ci) {
    const int16_t t = bi;
    bi = ci;
    ci = t;
  }
  if (ai > bi) {
    const int16_t t = ai;
    ai = bi;
    bi = t;
  }

  int16_t med = bi;
  if (med < 0) {
    med = static_cast<int16_t>(med + 4096);
  } else if (med >= 4096) {
    med = static_cast<int16_t>(med - 4096);
  }

  out_raw_deg = (static_cast<float>(med) * 360.0f) / 4096.0f;
  return true;
}

bool captureAs5600CalRawDeg(float& out_raw_deg) {
  // Calibration capture needs to be stable: take multiple median-filtered samples,
  // ignore large jumps, and smooth with a light EMA.
  constexpr uint8_t kNeedGood = AS5600_CAL_NEED_GOOD;
  constexpr uint32_t kTimeoutMs = AS5600_CAL_TIMEOUT_MS;
  constexpr uint8_t kDelayMs = 6;

  uint8_t good = 0;
  float ema = 0.0f;
  bool ema_init = false;

  float last = 0.0f;
  bool last_init = false;

  const uint32_t start_ms = millis();
  while (good < kNeedGood && static_cast<uint32_t>(millis() - start_ms) < kTimeoutMs) {
    float v = 0.0f;
    if (readAs5600RawMedianDeg(v)) {
      if (last_init) {
        const float delta = wrapAngleDeltaDeg(v - last);
        if (fabsf(delta) > AS5600_CAL_MAX_JUMP_DEG) {
          delay(kDelayMs);
          continue;
        }
      }
      last = v;
      last_init = true;

      if (!ema_init) {
        ema = v;
        ema_init = true;
      } else {
        ema = (kAs5600EmaAlpha * v) + ((1.0f - kAs5600EmaAlpha) * ema);
      }
      ++good;
    }
    delay(kDelayMs);
  }

  if (!ema_init || good < kNeedGood) {
    return false;
  }
  out_raw_deg = ema;
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
  runtimeCalSetSonarCenterSource(CenterSource::kManual);
  runtimeCalMarkZeroPosCaptured(true);
  sampleSonarNow(millis(), nullptr, nullptr, nullptr);
  g_runtime_cal_dirty = true;
  Serial.print(F("OK,cal_zero_position_set="));
  Serial.println(dist_cm, 6);
  printCalZero();
  return true;
}

bool finalizeNormalizedLimitSpan() {
  if (!runtimeCalIsLowerLimitCaptured() || !runtimeCalIsUpperLimitCaptured()) {
    return false;
  }

  const float delta = wrapAngleDeltaDeg(g_limit_upper_raw_deg - g_limit_lower_raw_deg);
  const int8_t new_sign = (delta >= 0.0f) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
  runtimeCalSetAs5600Sign(new_sign);
  runtimeCalSetAs5600LowerRawDeg(g_limit_lower_raw_deg);
  runtimeCalSetAs5600UpperRawDeg(g_limit_upper_raw_deg);
  runtimeCalSetSonarLowerCm(g_limit_lower_sonar_cm);
  runtimeCalSetSonarUpperCm(g_limit_upper_sonar_cm);

  const float span_deg = runtimeCalActuatorSpanDeg();
  if (span_deg < kLimitMinSpanDeg) {
    runtimeCalMarkLowerLimitCaptured(false);
    runtimeCalMarkUpperLimitCaptured(false);
    runtimeCalMarkZeroAngleCaptured(false);
    Serial.println(F("ERR,limit_span_too_small"));
    return false;
  }

  runtimeCalSetActuatorTrimDeg(runtimeCalActuatorMidpointDeg());
  runtimeCalSetActuatorTrimValid(true);
  runtimeCalSetActuatorTrimSource(ActuatorTrimSource::kMidpoint);
  runtimeCalMarkZeroAngleCaptured(true);

  if (runtimeCalSonarCenterSource() == CenterSource::kAuto) {
    runtimeCalSetSonarCenterCm(runtimeCalDerivedSonarCenterCm());
  }
  runtimeCalMarkZeroPosCaptured(true);
  runtimeCalSetUpperLimitNearSensor(true);
  runtimeCalApplyOrientationSonarSign();
  resetAs5600Filter();

  Serial.print(F("OK,as5600_sign="));
  Serial.print(new_sign);
  Serial.print(F(",sonar_sign="));
  Serial.print(runtimeCalSonarPosSign());
  Serial.println();

  if (g_limit_lower_sonar_cm > 1.0f && g_limit_upper_sonar_cm > 1.0f) {
    const float dd = g_limit_upper_sonar_cm - g_limit_lower_sonar_cm;
    const int8_t measured_sign = (dd >= 0.0f) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
    if (fabsf(dd) >= 1.0f && measured_sign == runtimeCalOrientationSonarSign()) {
      Serial.println(F("INFO,limits_sonar_ok"));
    } else {
      Serial.println(F("WARN,limits_sonar_bad"));
    }
  } else {
    Serial.println(F("WARN,limits_sonar_bad"));
  }
  return true;
}

bool captureDownLimit() {
  float raw_deg = 0.0f;
  if (!captureAs5600CalRawDeg(raw_deg)) {
    Serial.println(F("ERR,angle_not_ready"));
    return false;
  }
  float dist_cm = 0.0f;
  if (!captureSonarCalDistanceCm(dist_cm)) {
    Serial.println(F("ERR,sonar_not_ready"));
    printSonarDiag();
    return false;
  }

  g_limit_lower_raw_deg = raw_deg;
  g_limit_lower_sonar_cm = dist_cm;
  runtimeCalSetAs5600LowerRawDeg(raw_deg);
  runtimeCalSetSonarLowerCm(dist_cm);
  runtimeCalMarkLowerLimitCaptured(true);
  g_runtime_cal_dirty = true;

  Serial.print(F("OK,cal_limit_down_set="));
  Serial.println(0.0f, 6);
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
    return false;
  }
  float dist_cm = 0.0f;
  if (!captureSonarCalDistanceCm(dist_cm)) {
    Serial.println(F("ERR,sonar_not_ready"));
    printSonarDiag();
    return false;
  }

  g_limit_upper_raw_deg = raw_deg;
  g_limit_upper_sonar_cm = dist_cm;
  runtimeCalSetAs5600UpperRawDeg(raw_deg);
  runtimeCalSetSonarUpperCm(dist_cm);
  runtimeCalMarkUpperLimitCaptured(true);
  g_runtime_cal_dirty = true;

  float span_hint_deg = 0.0f;
  if (runtimeCalIsLowerLimitCaptured()) {
    span_hint_deg = fabsf(wrapAngleDeltaDeg(g_limit_upper_raw_deg - g_limit_lower_raw_deg));
  }
  Serial.print(F("OK,cal_limit_up_set="));
  Serial.println(span_hint_deg, 6);
  if (runtimeCalIsLowerLimitCaptured()) {
    if (!finalizeNormalizedLimitSpan()) {
      return false;
    }
  }
  printCalLimits();
  return true;
}

void printGuideNextAction() {
  // Deprecated: use host-side logger (/bringup) for workflow.
}

void setManualDriverEnabled(bool enabled) {
  g_manual_enable = enabled;
  const bool manual_ok = g_sensor.valid_angle && !g_fault_flags.i2c_error && !g_fault_flags.angle_oob;
  if (g_manual_enable && manual_ok) {
    g_stepper.enable(true);
    Serial.println(F("OK,driver_enabled"));
  } else {
    applySafeDisable();
    Serial.println(F("OK,driver_disabled"));
  }
}

bool startJogCommand(long steps, float rate, bool require_limits) {
  if (steps == 0 || rate <= 0.0f) {
    Serial.println(F("ERR,jog_args"));
    return false;
  }
  if (g_state_machine.state() == AppState::RUNNING) {
    Serial.println(F("ERR,stop_first"));
    return false;
  }

  float theta_now_deg = 0.0f;
  if (require_limits && !runtimeCalIsLimitsSet()) {
    Serial.println(F("ERR,jog_requires_limits"));
    return false;
  }
  const float upper = runtimeCalIsLimitsSet()
                          ? runtimeCalThetaUpperLimitDeg()
                          : kThetaHardLimitDeg;
  const float lower = runtimeCalIsLimitsSet()
                          ? runtimeCalThetaLowerLimitDeg()
                          : -kThetaHardLimitDeg;
  if (!sampleAngleNow(millis(), &theta_now_deg, nullptr)) {
    Serial.println(F("ERR,angle_not_ready"));
    return false;
  }
  if (steps > 0 && theta_now_deg >= (upper - kLimitStopMarginDeg)) {
    Serial.println(F("ERR,jog_hi"));
    return false;
  }
  if (steps < 0 && theta_now_deg <= (lower + kLimitStopMarginDeg)) {
    Serial.println(F("ERR,jog_lo"));
    return false;
  }

  g_stepper.enable(true);
  g_stepper.requestJogSteps(steps, rate);
  Serial.println(F("OK,jog_started"));
  return true;
}

__attribute__((noinline)) void loadCalibrationCommand() {
  const bool loaded = runtimeCalLoad();
  syncLimitCaptureCacheFromRuntime();
  resetAs5600Filter();
  resetLimitRecovery();
  resetTrimLearning();
  g_limit_stuck_fault = false;
  g_runtime_cal_dirty = false;
  Serial.print(F("OK,cal_load,"));
  Serial.println(loaded ? F("loaded") : F("defaults"));
}

void stopRunCommand() {
  g_state_machine.requestStop();
  resetLimitRecovery();
  resetTrimLearning();
  applySafeDisable();
  Serial.println(F("OK,stopped"));
}

void clearFaultCommand() {
  g_fault_flags = FaultFlags{};
  g_limit_stuck_fault = false;
  resetLimitRecovery();
  resetTrimLearning();
  if (g_state_machine.clearFault(currentConditions())) {
    Serial.println(F("OK,fault_cleared"));
  } else {
    Serial.println(F("ERR,no_fault_state"));
  }
  printFaultInfo();
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
  if (cond.faults_active) {
    printFaultInfo();
  }
  if (!cond.sensors_ok) {
    printSonarDiag();
  }
  printGuideNextAction();
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
      case 'e': {
        char* arg = strtok_r(nullptr, " ", &saveptr);
        if (arg == nullptr) {
          Serial.println(F("ERR,usage:e"));
          return;
        }
        setManualDriverEnabled(atoi(arg) != 0);
        return;
      }
      case 'j': {
        char* steps_arg = strtok_r(nullptr, " ", &saveptr);
        char* rate_arg = strtok_r(nullptr, " ", &saveptr);
        if (steps_arg == nullptr) {
          Serial.println(F("ERR,usage:j"));
          return;
        }

        const long steps = atol(steps_arg);
        const float rate = (rate_arg != nullptr) ? parseFloatFast(rate_arg) : 500.0f;
        startJogCommand(steps, rate, false);
        return;
      }
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
        loadCalibrationCommand();
        printGuideNextAction();
        return;
      }
      case 'd':
        runtimeCalResetDefaults();
        runtimeCalSave();
        syncLimitCaptureCacheFromRuntime();
        resetAs5600Filter();
        resetLimitRecovery();
        resetTrimLearning();
        g_limit_stuck_fault = false;
        g_runtime_cal_dirty = false;
        Serial.println(F("OK,cal_reset_saved"));
        printGuideNextAction();
        return;
      case 'r':
        tryStartRun();
        return;
      case 'k':
        stopRunCommand();
        printGuideNextAction();
        return;
      case 'f':
        clearFaultCommand();
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
      resetTrimLearning();
      applySafeDisable();
      return;
    }

    // Sonar dropout: ramp motor toward zero instead of holding last command.
    if (!g_sensor.valid_pos) {
      resetTrimLearning();
      float target = 0.0f;
      float delta = target - g_prev_step_rate_sps;
      if (delta > kMaxStepRateChangeSpsPerTick) delta = kMaxStepRateChangeSpsPerTick;
      if (delta < -kMaxStepRateChangeSpsPerTick) delta = -kMaxStepRateChangeSpsPerTick;
      target = g_prev_step_rate_sps + delta;
      g_prev_step_rate_sps = target;
      g_stepper.setSignedStepRate(target);
      return;
    }

    if (serviceActiveLimitRecovery(now_ms)) {
      resetTrimLearning();
      return;
    }

    g_setpoint.ball_pos_m_target = g_setpoint.ball_pos_cm_target * kCmToM;

    float theta_min_rad = -kThetaCmdLimitRad;
    float theta_max_rad = kThetaCmdLimitRad;
    effectiveThetaCmdBoundsRad(theta_min_rad, theta_max_rad);

    const ActuatorCmd cmd = g_controller.update(
        g_sensor, g_setpoint, kControlDtSec, theta_min_rad, theta_max_rad);
    if (cmd.enable) {
      if (maybeStartLimitRecovery(now_ms, cmd.signed_step_rate_sps)) {
        resetTrimLearning();
        return;
      }

      // Slew-rate limit step rate to prevent step skipping on sudden accel.
      float target = cmd.signed_step_rate_sps;
      float delta = target - g_prev_step_rate_sps;
      if (delta > kMaxStepRateChangeSpsPerTick) delta = kMaxStepRateChangeSpsPerTick;
      if (delta < -kMaxStepRateChangeSpsPerTick) delta = -kMaxStepRateChangeSpsPerTick;
      target = g_prev_step_rate_sps + delta;
      g_prev_step_rate_sps = target;

      g_stepper.enable(true);
      g_stepper.setSignedStepRate(target);
      serviceTrimLearning(now_ms, target);
    } else {
      resetTrimLearning();
      g_prev_step_rate_sps = 0.0f;
      applySafeDisable();
    }
    return;
  }

  resetTrimLearning();

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
  syncLimitCaptureCacheFromRuntime();

  // Init HC-SR04 distance sensor + PCINT echo capture.
  g_hcsr04.begin();
  setupEchoPcint();

  g_stepper.begin();
  g_stepper.beginScheduler();
  applySafeDisable();

  Wire.begin();
  const bool i2c_ok = g_as5600.begin(Wire, AS5600_I2C_ADDR);

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
