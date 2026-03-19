#include <AccelStepper.h>
#include <Arduino.h>
#include <SimpleKalmanFilter.h>

#include "calibration.h"
#include "sensors/as5600_sensor.h"

namespace {

constexpr int STEP_PIN = 2;
constexpr int DIR_PIN = 3;
constexpr int ENABLE_PIN = 4;
constexpr int MICROSTEPS = 16;
constexpr double STEPS_PER_REV = 200.0 * MICROSTEPS;

// Sharp GP2Y0A21YK0F on A0. Wiring: Vo->A0, VCC->5V, GND->GND.
constexpr uint8_t SHARP_PIN = A0;
// Sharp sensor internal cycle is ~38 ms; do not sample faster.
constexpr unsigned long SENSOR_SAMPLE_MS = 40UL;
constexpr unsigned long AS5600_SAMPLE_MS = 10UL;
constexpr unsigned long AS5600_STALE_FAULT_MS = 250UL;
constexpr int AS5600_ZERO_SAMPLES = 20;
constexpr unsigned long AS5600_ZERO_SAMPLE_DELAY_MS = 8UL;
constexpr unsigned long AS5600_ZERO_TIMEOUT_MS = 1200UL;
constexpr double AS5600_ZERO_MAX_SPAN_DEG = 1.5;
constexpr double AS5600_EMA_ALPHA = 0.3;
constexpr double AS5600_MAX_JUMP_DEG = 25.0;

// Kalman filter noise parameters.
// MEASUREMENT_ERROR = 2.0: actual inter-sample noise from the Sharp GP2Y0A21YK0F
// is ±2-4 cm (stepper EMI + sensor shot noise). With M=2.0, V=1.0, the Kalman
// gain K ≈ 0.33, so a 4 cm spike only shifts the estimate by ~1.3 cm.
// With M=0.3 (previous), K ≈ 0.77 — the filter chased every spike and the PID
// oscillated wildly.
constexpr double MEASUREMENT_ERROR = 2.0;
constexpr double VARIANCE = 1.0;

// The stepper has no homing reference, so control must stay purely relative
// within a run. Recent logs show the "ball too far, come back toward near"
// branch is the unstable one: far tracking is acceptable, but center and near
// fall into a far-side limit cycle once that branch takes over. Keep the
// away-from-sensor branch stronger, and make the toward-near branch softer and
// more damped. Recent closed-loop logs show the remaining failure is not raw
// authority at the far target, but the inability to retain enough nonzero hold
// command once center/near are reached: the ball gets close, P collapses toward
// zero, and the bench's static trim/friction drifts it away again. The
// within-run bias term therefore needs to adapt fast enough to matter inside a
// 30 s phase, while still unwinding quickly when its sign becomes wrong.
constexpr double REGULATION_TOWARD_FAR_KP = 18.0;
constexpr double REGULATION_TOWARD_NEAR_KP = 17.0;
constexpr double ACQUIRE_TOWARD_FAR_KP = 28.0;
constexpr double ACQUIRE_TOWARD_NEAR_KP = 25.0;
constexpr double REGULATION_TOWARD_FAR_KI = 0.8;
constexpr double REGULATION_TOWARD_NEAR_KI = 0.7;
constexpr double ACQUIRE_TOWARD_FAR_KI = 0.2;
constexpr double ACQUIRE_TOWARD_NEAR_KI = 0.2;
constexpr double REGULATION_TOWARD_FAR_KD = 7.0;
constexpr double REGULATION_TOWARD_NEAR_KD = 8.0;
constexpr double ACQUIRE_TOWARD_FAR_KD = 4.5;
constexpr double ACQUIRE_TOWARD_NEAR_KD = 5.5;
constexpr double SETPOINT_HOLD_POS_TOL_CM = 0.35;
constexpr double SETPOINT_HOLD_RATE_TOL_CM_S = 1.2;
constexpr int REGULATION_TOWARD_FAR_LIMIT_FULL_STEPS = 6;
constexpr int REGULATION_TOWARD_NEAR_LIMIT_FULL_STEPS = 6;
constexpr int ACQUIRE_TOWARD_FAR_LIMIT_FULL_STEPS = 14;
constexpr int ACQUIRE_TOWARD_NEAR_LIMIT_FULL_STEPS = 12;
constexpr double REGULATION_BAND_CM = 2.0;
constexpr double INTEGRAL_ACTIVE_BAND_CM = 2.0;
constexpr double INTEGRAL_ACTIVE_RATE_TOL_CM_S = 2.5;
constexpr double REGULATION_INTEGRAL_LIMIT_STEPS = 32.0;
constexpr double ACQUIRE_INTEGRAL_LIMIT_STEPS = 32.0;
constexpr double INTEGRAL_UNWIND_TAU_S = 0.30;
constexpr double INTEGRAL_LEAK_TAU_S = 4.0;
constexpr double BIAS_ADAPT_RATE_STEPS_PER_CM_S = 4.0;
constexpr double BIAS_ACTIVE_POS_TOL_CM = 0.5;
constexpr double BIAS_ACTIVE_RATE_TOL_CM_S = 1.2;
constexpr double BIAS_LIMIT_STEPS = 96.0;
constexpr double BIAS_UNWIND_TAU_S = 0.40;
constexpr double BIAS_LEAK_TAU_S = 12.0;
constexpr double REGULATION_TOWARD_FAR_OUTPUT_SLEW_STEPS_PER_TICK = 40.0;
constexpr double REGULATION_TOWARD_NEAR_OUTPUT_SLEW_STEPS_PER_TICK = 34.0;
constexpr double ACQUIRE_TOWARD_FAR_OUTPUT_SLEW_STEPS_PER_TICK = 72.0;
constexpr double ACQUIRE_TOWARD_NEAR_OUTPUT_SLEW_STEPS_PER_TICK = 56.0;
// Band-limit the derivative term so Sharp velocity spikes do not dominate P
// right as the ball approaches the target.
constexpr double VELOCITY_FILTER_TAU_S = 0.25;
// Keep the migrated actuator inside a conservative envelope around the
// run-start neutral until measured-angle control is proven on hardware.
constexpr double REGULATION_THETA_LIMIT_DEG = 2.5;
constexpr double ACQUIRE_THETA_LIMIT_DEG = 3.5;
constexpr double HARD_THETA_LIMIT_DEG = 4.0;
constexpr double HARD_THETA_STOP_MARGIN_DEG = 0.20;
constexpr double HARD_THETA_FAULT_MARGIN_DEG = 0.35;
constexpr double INNER_MAX_STEP_RATE_SPS = 500.0;
constexpr double INNER_ACCEL_SPS2 = 2500.0;
constexpr double INNER_DEADBAND_STEPS = 2.0;
constexpr double STEPPER_MAX_ACCEL_SPS2 = 15000.0;
constexpr double START_VERIFY_THETA_CMD_DEG = 0.9;
constexpr double START_VERIFY_MIN_RESPONSE_DEG = 0.20;
constexpr double START_VERIFY_MAX_ABS_THETA_DEG = 1.5;
constexpr double START_VERIFY_STEP_RATE_SPS = 140.0;
constexpr double START_VERIFY_ACCEL_SPS2 = 800.0;
constexpr unsigned long START_VERIFY_TIMEOUT_MS = 1200UL;
constexpr unsigned long DRIVER_ENABLE_SETTLE_MS = 20UL;

// Sharp is mounted at the pivot end, so the measured distance is SMALL near the
// sensor and LARGE toward the motor end. The near stop now physically blocks
// the ball from entering the Sharp dead zone below about 8 cm, leaving roughly
// 13 cm of runner travel. For a 40 mm ball, the sensed near-surface travel is
// runner_length - ball_diameter ≈ 9 cm.
constexpr double NEAR_STOP_DISTANCE_CM = 8.5;
constexpr double USABLE_RUNNER_LENGTH_CM = 13.0;
constexpr double BALL_DIAMETER_CM = 4.0;
constexpr double SENSED_TRAVEL_CM = USABLE_RUNNER_LENGTH_CM - BALL_DIAMETER_CM;
constexpr double FAR_STOP_DISTANCE_CM = NEAR_STOP_DISTANCE_CM + SENSED_TRAVEL_CM;
constexpr double CENTER_DISTANCE_CM = 0.5 * (NEAR_STOP_DISTANCE_CM + FAR_STOP_DISTANCE_CM);
constexpr double ENDPOINT_TARGET_MARGIN_CM = 0.5;
constexpr double NEAR_TARGET_DISTANCE_CM = NEAR_STOP_DISTANCE_CM + ENDPOINT_TARGET_MARGIN_CM;
constexpr double FAR_TARGET_DISTANCE_CM = FAR_STOP_DISTANCE_CM - ENDPOINT_TARGET_MARGIN_CM;

// Tighten the valid window to the shortened runner geometry. Allow a small
// margin around the physical stops so noise at the new near stop does not
// immediately force grace-mode hold.
// The obstruction keeps the ball out of the true Sharp dead zone, but the raw
// sensor still throws occasional 7.5-8.0 cm dips when the ball is parked near
// the lower stop. Accept those noisy edge samples so the controller does not
// keep dropping into grace-hold right where near-end regulation matters most.
constexpr double MIN_VALID_DISTANCE_CM = 7.5;
constexpr double MAX_VALID_DISTANCE_CM = 19.5;

// Grace period extended and applies to ALL invalid reading types.
constexpr unsigned long INVALID_GRACE_MS = 600UL;
constexpr int INVALID_GRACE_READS = 15;

enum class ReferenceProfile : uint8_t {
  kCenterHold,
  kCenterFarNear90s,
};

enum class ResetMode : uint8_t {
  kAcquireTarget,
  kBumplessResume,
};

// Keep the working controller unchanged and only move the reference. Endpoint
// targets stay 0.5 cm inside the physical stops to avoid chatter against the
// obstruction and Sharp fold-back region.
constexpr ReferenceProfile ACTIVE_REFERENCE_PROFILE = ReferenceProfile::kCenterFarNear90s;
constexpr unsigned long REFERENCE_PHASE_MS = 30000UL;

constexpr unsigned long kRefSerialBaud = 115200UL;
constexpr int PID_SAMPLE_TIME_MS = 100;
constexpr unsigned long TELEMETRY_INTERVAL_MS = 100UL;
constexpr int WARMUP_SAMPLES = 10;
constexpr unsigned long WARMUP_DELAY_MS = 40UL;  // one sensor cycle per warmup sample

double raw_distance_cm = 0.0;
double ball_position = 0.0;
double targetDistanceCm = CENTER_DISTANCE_CM;
double input = 0.0;
double output = 0.0;
double radiansPerStep = 0.0;
double degreesPerStep = 0.0;
double ballVelocityCmS = 0.0;
double filteredBallVelocityCmS = 0.0;
double integralOutputSteps = 0.0;
double biasOutputSteps = 0.0;
bool invalidFallbackActive = false;
bool controlRunActive = false;
unsigned long invalidSequenceStartMs = 0UL;
unsigned long lastControlComputeMs = 0UL;
unsigned long profileStartMs = 0UL;
unsigned long prevValidBallPositionMs = 0UL;
int invalidReadStreak = 0;
int activeReferencePhase = -1;
long graceHoldTargetSteps = 0;
double graceHoldThetaDeg = 0.0;
double prevValidBallPositionCm = 0.0;
bool hasPrevValidBallPosition = false;
double thetaMeasuredDeg = 0.0;
double thetaMeasuredRawDeg = 0.0;
double thetaCommandDeg = 0.0;
double innerRateCommandSps = 0.0;
double as5600ZeroRawDeg = 0.0;
int8_t encoderThetaSign = bb::AS5600_SIGN;
bool as5600BootOk = false;
bool as5600ZeroCaptured = false;
bool as5600MeasurementInitialized = false;
bool as5600FaultActive = false;
bool actuatorSafetyLatched = false;
unsigned long lastAs5600ValidMs = 0UL;
unsigned long lastInnerControlMs = 0UL;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
SimpleKalmanFilter filter(MEASUREMENT_ERROR, MEASUREMENT_ERROR, VARIANCE);
bb::AS5600Sensor as5600;

// GP2Y0A21YK0F empirical power-law: distance_cm = 29.988 * voltage^(-1.173)
float sharpAdcToCm(uint16_t adc) {
  if (adc < 10) return 0.0f;
  const float v = static_cast<float>(adc) * (5.0f / 1024.0f);
  if (v < 0.05f) return 0.0f;
  return 29.988f * powf(v, -1.173f);
}

double slewToward(double current, double target, double maxDelta) {
  if (target > current + maxDelta) return current + maxDelta;
  if (target < current - maxDelta) return current - maxDelta;
  return target;
}

double clampAbs(double value, double limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

double wrapAngleDeltaDeg(double deltaDeg) {
  while (deltaDeg > 180.0) deltaDeg -= 360.0;
  while (deltaDeg < -180.0) deltaDeg += 360.0;
  return deltaDeg;
}

double controlLimitSteps(bool regulateNow, bool demandTowardNear) {
  (void)demandTowardNear;
  const double thetaLimitDeg = regulateNow ? REGULATION_THETA_LIMIT_DEG : ACQUIRE_THETA_LIMIT_DEG;
  return thetaLimitDeg / degreesPerStep;
}

double outputSlewLimitSteps(bool regulateNow, bool demandTowardNear) {
  return regulateNow
             ? (demandTowardNear ? REGULATION_TOWARD_NEAR_OUTPUT_SLEW_STEPS_PER_TICK
                                 : REGULATION_TOWARD_FAR_OUTPUT_SLEW_STEPS_PER_TICK)
             : (demandTowardNear ? ACQUIRE_TOWARD_NEAR_OUTPUT_SLEW_STEPS_PER_TICK
                                 : ACQUIRE_TOWARD_FAR_OUTPUT_SLEW_STEPS_PER_TICK);
}

const __FlashStringHelper* referenceProfileName() {
  switch (ACTIVE_REFERENCE_PROFILE) {
    case ReferenceProfile::kCenterHold:
      return F("center_hold");
    case ReferenceProfile::kCenterFarNear90s:
      return F("center_far_near_90s");
  }
  return F("unknown");
}

int referencePhaseForElapsedMs(unsigned long elapsedMs) {
  switch (ACTIVE_REFERENCE_PROFILE) {
    case ReferenceProfile::kCenterHold:
      return 0;
    case ReferenceProfile::kCenterFarNear90s:
      if (elapsedMs < REFERENCE_PHASE_MS) return 0;
      if (elapsedMs < (2UL * REFERENCE_PHASE_MS)) return 1;
      return 2;
  }
  return 0;
}

double referenceTargetCmForPhase(int phase) {
  switch (ACTIVE_REFERENCE_PROFILE) {
    case ReferenceProfile::kCenterHold:
      return CENTER_DISTANCE_CM;
    case ReferenceProfile::kCenterFarNear90s:
      if (phase <= 0) return CENTER_DISTANCE_CM;
      if (phase == 1) return FAR_TARGET_DISTANCE_CM;
      return NEAR_TARGET_DISTANCE_CM;
  }
  return CENTER_DISTANCE_CM;
}

const __FlashStringHelper* referencePhaseLabel(int phase) {
  switch (ACTIVE_REFERENCE_PROFILE) {
    case ReferenceProfile::kCenterHold:
      return F("hold_center");
    case ReferenceProfile::kCenterFarNear90s:
      if (phase <= 0) return F("hold_center");
      if (phase == 1) return F("hold_far");
      return F("hold_near");
  }
  return F("hold_center");
}

void emitReferenceConfig() {
  Serial.print(F("REF_CFG,profile="));
  Serial.print(referenceProfileName());
  Serial.print(F(",center_cm="));
  Serial.print(CENTER_DISTANCE_CM, 4);
  Serial.print(F(",near_cm="));
  Serial.print(NEAR_TARGET_DISTANCE_CM, 4);
  Serial.print(F(",far_cm="));
  Serial.print(FAR_TARGET_DISTANCE_CM, 4);
  Serial.print(F(",segment_ms="));
  Serial.println(REFERENCE_PHASE_MS);
}

void emitHostStartReady() { Serial.println(F("HOST_START_READY")); }

bool consumeHostStartRequest() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == '\n' || c == '\r') {
      return true;
    }
  }
  return false;
}

void emitReferenceChange(unsigned long nowMs, int phase) {
  Serial.print(F("REF,"));
  Serial.print(nowMs);
  Serial.print(',');
  Serial.print(targetDistanceCm, 4);
  Serial.print(',');
  Serial.println(referencePhaseLabel(phase));
}

void emitAs5600BootStatus() {
  Serial.print(F("AS5600_BOOT,ok="));
  Serial.print(as5600BootOk ? 1 : 0);
  if (as5600BootOk) {
    const bb::AS5600Status status = as5600.readStatus();
    Serial.print(F(",status_ok="));
    Serial.print(status.read_ok ? 1 : 0);
    Serial.print(F(",md="));
    Serial.print(status.magnet_detected ? 1 : 0);
    Serial.print(F(",mh="));
    Serial.print(status.too_strong ? 1 : 0);
    Serial.print(F(",ml="));
    Serial.print(status.too_weak ? 1 : 0);
    Serial.print(F(",agc="));
    Serial.print(status.agc);
    Serial.print(F(",mag="));
    Serial.print(status.magnitude);
  }
  Serial.println();
}

void emitAs5600Zero(double rawZeroDeg) {
  Serial.print(F("AS5600_ZERO,raw_deg="));
  Serial.println(rawZeroDeg, 4);
}

void armDriver() { digitalWrite(ENABLE_PIN, LOW); }

bool as5600Ok(unsigned long nowMs) {
  return as5600ZeroCaptured && (lastAs5600ValidMs != 0UL) &&
         ((nowMs - lastAs5600ValidMs) <= AS5600_STALE_FAULT_MS);
}

bool sampleAs5600(unsigned long nowMs, bool force = false) {
  static unsigned long lastSampleMs = 0UL;
  if (!force && (nowMs - lastSampleMs) < AS5600_SAMPLE_MS) {
    return as5600Ok(nowMs);
  }
  lastSampleMs = nowMs;

  float rawAngleDeg = 0.0f;
  if (!as5600BootOk || !as5600.readRawAngleDeg(rawAngleDeg)) {
    return as5600Ok(nowMs);
  }

  thetaMeasuredRawDeg = rawAngleDeg;
  if (!as5600ZeroCaptured) {
    return false;
  }

  double thetaDeg = static_cast<double>(encoderThetaSign) *
                    wrapAngleDeltaDeg(static_cast<double>(rawAngleDeg) - as5600ZeroRawDeg);
  if (!as5600MeasurementInitialized) {
    thetaMeasuredDeg = thetaDeg;
    as5600MeasurementInitialized = true;
  } else {
    double deltaDeg = wrapAngleDeltaDeg(thetaDeg - thetaMeasuredDeg);
    if (fabs(deltaDeg) > AS5600_MAX_JUMP_DEG) {
      deltaDeg = (deltaDeg >= 0.0) ? AS5600_MAX_JUMP_DEG : -AS5600_MAX_JUMP_DEG;
      thetaDeg = thetaMeasuredDeg + deltaDeg;
    }
    thetaMeasuredDeg += AS5600_EMA_ALPHA * (thetaDeg - thetaMeasuredDeg);
  }

  lastAs5600ValidMs = nowMs;
  return true;
}

bool captureAs5600Neutral() {
  if (!as5600BootOk) return false;

  const unsigned long startMs = millis();
  bool haveRaw = false;
  double lastRawDeg = 0.0;
  double unwrappedDeg = 0.0;
  double sumDeg = 0.0;
  double minDeg = 0.0;
  double maxDeg = 0.0;
  int good = 0;

  while (good < AS5600_ZERO_SAMPLES && (millis() - startMs) < AS5600_ZERO_TIMEOUT_MS) {
    float rawAngleDeg = 0.0f;
    if (!as5600.readRawAngleDeg(rawAngleDeg)) {
      delay(AS5600_ZERO_SAMPLE_DELAY_MS);
      continue;
    }

    const double rawDeg = static_cast<double>(rawAngleDeg);
    if (!haveRaw) {
      lastRawDeg = rawDeg;
      unwrappedDeg = rawDeg;
      minDeg = rawDeg;
      maxDeg = rawDeg;
      haveRaw = true;
    } else {
      const double deltaDeg = wrapAngleDeltaDeg(rawDeg - lastRawDeg);
      if (fabs(deltaDeg) > AS5600_MAX_JUMP_DEG) {
        lastRawDeg = rawDeg;
        delay(AS5600_ZERO_SAMPLE_DELAY_MS);
        continue;
      }
      unwrappedDeg += deltaDeg;
      lastRawDeg = rawDeg;
      if (unwrappedDeg < minDeg) minDeg = unwrappedDeg;
      if (unwrappedDeg > maxDeg) maxDeg = unwrappedDeg;
    }

    sumDeg += unwrappedDeg;
    ++good;
    delay(AS5600_ZERO_SAMPLE_DELAY_MS);
  }

  if (good < AS5600_ZERO_SAMPLES) return false;
  if ((maxDeg - minDeg) > AS5600_ZERO_MAX_SPAN_DEG) return false;

  as5600ZeroRawDeg = fmod(sumDeg / static_cast<double>(good), 360.0);
  if (as5600ZeroRawDeg < 0.0) as5600ZeroRawDeg += 360.0;
  as5600ZeroCaptured = true;
  as5600MeasurementInitialized = false;
  thetaMeasuredDeg = 0.0;
  thetaMeasuredRawDeg = as5600ZeroRawDeg;
  lastAs5600ValidMs = 0UL;
  sampleAs5600(millis(), true);
  emitAs5600Zero(as5600ZeroRawDeg);
  return as5600Ok(millis());
}

void stopInnerLoop() {
  innerRateCommandSps = 0.0;
  stepper.setSpeed(0.0);
}

void applySafeDisable() {
  stopInnerLoop();
  digitalWrite(ENABLE_PIN, HIGH);
}

bool runOpenLoopStepMove(long targetSteps,
                         double maxSpeedSps,
                         double accelSps2,
                         unsigned long timeoutMs,
                         const __FlashStringHelper** failReason) {
  if (failReason != nullptr) *failReason = nullptr;
  stepper.setMaxSpeed(maxSpeedSps);
  stepper.setAcceleration(accelSps2);
  stepper.moveTo(targetSteps);
  const unsigned long startMs = millis();
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    const unsigned long nowMs = millis();
    sampleAs5600(nowMs, true);
    if (fabs(thetaMeasuredDeg) > START_VERIFY_MAX_ABS_THETA_DEG) {
      stepper.stop();
      while (stepper.distanceToGo() != 0 && (millis() - startMs) < timeoutMs) {
        stepper.run();
      }
      stepper.setCurrentPosition(stepper.currentPosition());
      stepper.moveTo(stepper.currentPosition());
      stepper.setMaxSpeed(INNER_MAX_STEP_RATE_SPS);
      stepper.setAcceleration(STEPPER_MAX_ACCEL_SPS2);
      if (failReason != nullptr) *failReason = F("verify_theta_excursion");
      return false;
    }
    if ((millis() - startMs) > timeoutMs) {
      stepper.stop();
      while (stepper.distanceToGo() != 0 && (millis() - startMs) < (timeoutMs + 250UL)) {
        stepper.run();
      }
      stepper.setCurrentPosition(stepper.currentPosition());
      stepper.moveTo(stepper.currentPosition());
      stepper.setMaxSpeed(INNER_MAX_STEP_RATE_SPS);
      stepper.setAcceleration(STEPPER_MAX_ACCEL_SPS2);
      if (failReason != nullptr) *failReason = F("verify_move_timeout");
      return false;
    }
  }
  stepper.setMaxSpeed(INNER_MAX_STEP_RATE_SPS);
  stepper.setAcceleration(STEPPER_MAX_ACCEL_SPS2);
  return true;
}

bool verifyActuatorDirection() {
  if (!as5600Ok(millis())) {
    Serial.println(F("AS5600_VERIFY_FAIL,reason=encoder_not_ready"));
    return false;
  }

  const long verifySteps = lround(-START_VERIFY_THETA_CMD_DEG / degreesPerStep);
  if (verifySteps == 0L) {
    Serial.println(F("AS5600_VERIFY_FAIL,reason=zero_verify_steps"));
    return false;
  }

  armDriver();
  delay(DRIVER_ENABLE_SETTLE_MS);
  stopInnerLoop();
  stepper.setCurrentPosition(0);
  stepper.moveTo(0);
  sampleAs5600(millis(), true);
  const double thetaBeforeDeg = thetaMeasuredDeg;
  const double rawBeforeDeg = thetaMeasuredRawDeg;

  const __FlashStringHelper* failReason = nullptr;
  if (!runOpenLoopStepMove(verifySteps, START_VERIFY_STEP_RATE_SPS, START_VERIFY_ACCEL_SPS2,
                           START_VERIFY_TIMEOUT_MS, &failReason)) {
    applySafeDisable();
    Serial.print(F("AS5600_VERIFY_FAIL,reason="));
    Serial.println(failReason != nullptr ? failReason : F("verify_move_failed"));
    return false;
  }

  sampleAs5600(millis(), true);
  const double rawAfterDeg = thetaMeasuredRawDeg;
  const double rawDeltaDeg = wrapAngleDeltaDeg(rawAfterDeg - rawBeforeDeg);
  if (fabs(rawDeltaDeg) < START_VERIFY_MIN_RESPONSE_DEG) {
    applySafeDisable();
    Serial.print(F("AS5600_VERIFY_FAIL,raw_delta_deg="));
    Serial.println(rawDeltaDeg, 4);
    return false;
  }

  // A negative-step jog is defined by the existing Sharp control convention to
  // be positive logical theta (motor side up). Use that bounded jog to infer
  // the encoder sign at run start instead of trusting the compile-time guess.
  encoderThetaSign = (rawDeltaDeg >= 0.0) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
  as5600MeasurementInitialized = false;
  sampleAs5600(millis(), true);
  const double thetaAfterDeg = thetaMeasuredDeg;
  const double thetaDeltaDeg = thetaAfterDeg - thetaBeforeDeg;

  const bool ok = thetaDeltaDeg >= START_VERIFY_MIN_RESPONSE_DEG &&
                  thetaAfterDeg >= START_VERIFY_MIN_RESPONSE_DEG &&
                  fabs(thetaAfterDeg) <= START_VERIFY_MAX_ABS_THETA_DEG;

  const __FlashStringHelper* returnFailReason = nullptr;
  if (!runOpenLoopStepMove(0L, START_VERIFY_STEP_RATE_SPS, START_VERIFY_ACCEL_SPS2,
                           START_VERIFY_TIMEOUT_MS, &returnFailReason)) {
    Serial.print(F("AS5600_VERIFY_WARN,reason="));
    Serial.println(returnFailReason != nullptr ? returnFailReason : F("verify_return_failed"));
  }
  sampleAs5600(millis(), true);
  stepper.setCurrentPosition(0);
  stepper.moveTo(0);
  applySafeDisable();

  if (!ok) {
    Serial.print(F("AS5600_VERIFY_FAIL,theta_before_deg="));
    Serial.print(thetaBeforeDeg, 4);
    Serial.print(F(",theta_after_deg="));
    Serial.print(thetaAfterDeg, 4);
    Serial.print(F(",theta_delta_deg="));
    Serial.print(thetaDeltaDeg, 4);
    Serial.print(F(",raw_delta_deg="));
    Serial.print(rawDeltaDeg, 4);
    Serial.print(F(",theta_sign="));
    Serial.println(encoderThetaSign);
    return false;
  }

  Serial.print(F("AS5600_VERIFY_OK,theta_delta_deg="));
  Serial.print(thetaDeltaDeg, 4);
  Serial.print(F(",raw_delta_deg="));
  Serial.print(rawDeltaDeg, 4);
  Serial.print(F(",theta_sign="));
  Serial.println(encoderThetaSign);
  return true;
}

void updateReferenceProfile() {
  if (profileStartMs == 0UL) return;
  const unsigned long nowMs = millis();
  const unsigned long elapsedMs = nowMs - profileStartMs;
  const int phase = referencePhaseForElapsedMs(elapsedMs);
  const double newTargetCm = referenceTargetCmForPhase(phase);
  if (phase == activeReferencePhase && fabs(newTargetCm - targetDistanceCm) <= 1.0e-6) {
    return;
  }
  targetDistanceCm = newTargetCm;
  activeReferencePhase = phase;
  input = targetDistanceCm - ball_position;
  integralOutputSteps = 0.0;
  biasOutputSteps = 0.0;
  filteredBallVelocityCmS = ballVelocityCmS;
  output = 0.0;
  thetaCommandDeg = 0.0;
  lastControlComputeMs = 0UL;
  emitReferenceChange(nowMs, phase);
}

bool readSensor() {
  static unsigned long last_sample_ms = 0;
  const unsigned long now_ms = millis();
  if (now_ms - last_sample_ms < SENSOR_SAMPLE_MS) {
    // Too soon; re-report previous reading without counting as a miss.
    return (raw_distance_cm > MIN_VALID_DISTANCE_CM && raw_distance_cm < MAX_VALID_DISTANCE_CM);
  }
  last_sample_ms = now_ms;

  const float dist = sharpAdcToCm(static_cast<uint16_t>(analogRead(SHARP_PIN)));
  raw_distance_cm = dist;
  if (dist > MIN_VALID_DISTANCE_CM && dist < MAX_VALID_DISTANCE_CM) {
    ball_position = filter.updateEstimate(dist);
    if (hasPrevValidBallPosition) {
      const unsigned long delta_ms = now_ms - prevValidBallPositionMs;
      if (delta_ms > 0UL) {
        ballVelocityCmS =
            (ball_position - prevValidBallPositionCm) * (1000.0 / static_cast<double>(delta_ms));
      }
    } else {
      ballVelocityCmS = 0.0;
      hasPrevValidBallPosition = true;
    }
    prevValidBallPositionCm = ball_position;
    prevValidBallPositionMs = now_ms;
    return true;
  }
  return false;
}

void emitTelemetry() {
  static unsigned long last_emit_ms = 0UL;
  const unsigned long now_ms = millis();
  if (now_ms - last_emit_ms < TELEMETRY_INTERVAL_MS) {
    return;
  }
  last_emit_ms = now_ms;

  const long current_steps = stepper.currentPosition();
  const double beam_angle_rad = -static_cast<double>(current_steps) * radiansPerStep;
  const bool encoderOk = as5600Ok(now_ms);

  Serial.print(F("TEL,"));
  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(raw_distance_cm, 4);
  Serial.print(',');
  Serial.print(ball_position, 4);
  Serial.print(',');
  Serial.print(input, 4);
  Serial.print(',');
  Serial.print(output, 4);
  Serial.print(',');
  Serial.print(current_steps);
  Serial.print(',');
  Serial.print(beam_angle_rad, 6);
  Serial.print(',');
  Serial.print(filteredBallVelocityCmS, 4);
  Serial.print(',');
  Serial.print(integralOutputSteps, 4);
  Serial.print(',');
  Serial.print(biasOutputSteps, 4);
  Serial.print(',');
  Serial.print(thetaMeasuredDeg, 4);
  Serial.print(',');
  Serial.print(thetaCommandDeg, 4);
  Serial.print(',');
  Serial.print(thetaCommandDeg - thetaMeasuredDeg, 4);
  Serial.print(',');
  Serial.print(innerRateCommandSps, 4);
  Serial.print(',');
  Serial.println(encoderOk ? 1 : 0);
}

void holdGracePosition() { thetaCommandDeg = graceHoldThetaDeg; }

void latchSafetyStop(const __FlashStringHelper* reason) {
  if (!actuatorSafetyLatched) {
    Serial.print(F("SAFETY_STOP,reason="));
    Serial.print(reason);
    Serial.print(F(",theta_meas_deg="));
    Serial.print(thetaMeasuredDeg, 4);
    Serial.print(F(",theta_cmd_deg="));
    Serial.println(thetaCommandDeg, 4);
  }
  actuatorSafetyLatched = true;
  controlRunActive = false;
  invalidFallbackActive = false;
  as5600FaultActive = false;
  input = 0.0;
  output = 0.0;
  integralOutputSteps = 0.0;
  biasOutputSteps = 0.0;
  thetaCommandDeg = 0.0;
  profileStartMs = 0UL;
  activeReferencePhase = -1;
  lastControlComputeMs = 0UL;
  lastInnerControlMs = 0UL;
  applySafeDisable();
  emitHostStartReady();
}

void updateInnerLoop(unsigned long nowMs) {
  if (!as5600Ok(nowMs)) {
    applySafeDisable();
    return;
  }

  thetaCommandDeg = clampAbs(thetaCommandDeg, HARD_THETA_LIMIT_DEG);
  if (fabs(thetaMeasuredDeg) > (HARD_THETA_LIMIT_DEG + HARD_THETA_FAULT_MARGIN_DEG)) {
    latchSafetyStop(F("theta_oob"));
    return;
  }
  if (thetaMeasuredDeg >= (HARD_THETA_LIMIT_DEG - HARD_THETA_STOP_MARGIN_DEG) &&
      thetaCommandDeg > thetaMeasuredDeg) {
    latchSafetyStop(F("upper_limit"));
    return;
  }
  if (thetaMeasuredDeg <= -(HARD_THETA_LIMIT_DEG - HARD_THETA_STOP_MARGIN_DEG) &&
      thetaCommandDeg < thetaMeasuredDeg) {
    latchSafetyStop(F("lower_limit"));
    return;
  }

  if (lastInnerControlMs == 0UL) {
    lastInnerControlMs = nowMs;
  }

  const unsigned long deltaMs = nowMs - lastInnerControlMs;
  if (deltaMs == 0UL) {
    stepper.runSpeed();
    return;
  }
  lastInnerControlMs = nowMs;

  const double thetaCmdSteps = -thetaCommandDeg / degreesPerStep;
  const double thetaMeasSteps = -thetaMeasuredDeg / degreesPerStep;
  const double stepError = thetaCmdSteps - thetaMeasSteps;
  double desiredRateSps = 0.0;
  if (fabs(stepError) > INNER_DEADBAND_STEPS) {
    const double magnitude =
        sqrt(2.0 * INNER_ACCEL_SPS2 * fabs(stepError));
    desiredRateSps = (stepError >= 0.0 ? 1.0 : -1.0) *
                     clampAbs(magnitude, INNER_MAX_STEP_RATE_SPS);
  }

  const double dt_s = static_cast<double>(deltaMs) / 1000.0;
  innerRateCommandSps =
      slewToward(innerRateCommandSps, desiredRateSps, INNER_ACCEL_SPS2 * dt_s);
  stepper.setSpeed(innerRateCommandSps);
  stepper.runSpeed();
}

void resetControllerState(ResetMode mode) {
  input = targetDistanceCm - ball_position;
  filteredBallVelocityCmS = ballVelocityCmS;
  lastControlComputeMs = millis();
  lastInnerControlMs = 0UL;

  if (mode == ResetMode::kAcquireTarget) {
    output = 0.0;
    integralOutputSteps = 0.0;
    biasOutputSteps = 0.0;
    thetaCommandDeg = thetaMeasuredDeg;
    stopInnerLoop();
  }
}

void startControlRun() {
  if (controlRunActive) return;
  actuatorSafetyLatched = false;
  encoderThetaSign = bb::AS5600_SIGN;
  if (!captureAs5600Neutral()) {
    Serial.println(F("AS5600_START_FAIL"));
    applySafeDisable();
    emitHostStartReady();
    return;
  }
  if (!verifyActuatorDirection()) {
    applySafeDisable();
    emitHostStartReady();
    return;
  }
  invalidFallbackActive = false;
  as5600FaultActive = false;
  invalidReadStreak = 0;
  invalidSequenceStartMs = 0UL;
  // The operator manually places the beam at neutral before pressing Enter, so
  // treat that within-run pose as the stepper origin for all subsequent
  // diagnostic step estimates in this run.
  stepper.setCurrentPosition(0);
  stepper.moveTo(0);
  graceHoldTargetSteps = 0;
  graceHoldThetaDeg = 0.0;
  thetaCommandDeg = thetaMeasuredDeg;
  profileStartMs = millis();
  activeReferencePhase = -1;
  emitReferenceConfig();
  updateReferenceProfile();
  armDriver();
  resetControllerState(ResetMode::kAcquireTarget);
  controlRunActive = true;
}

}  // namespace

void setup() {
  Serial.begin(kRefSerialBaud);
  Serial.println(F("BALL_BEAM_REF_BOOT"));

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  // TMC2209 enable is active-low.
  pinMode(SHARP_PIN, INPUT);

  degreesPerStep = 360.0 / STEPS_PER_REV;
  radiansPerStep = degreesPerStep * (PI / 180.0);

  stepper.setMaxSpeed(INNER_MAX_STEP_RATE_SPS);
  stepper.setAcceleration(STEPPER_MAX_ACCEL_SPS2);
  stepper.setMinPulseWidth(5);
  // No DIR inversion. Logical theta-to-step direction is handled explicitly in
  // the measured-angle inner loop.

  as5600BootOk = as5600.begin();

  for (int i = 0; i < WARMUP_SAMPLES; ++i) {
    readSensor();
    delay(WARMUP_DELAY_MS);
  }

  emitAs5600BootStatus();
  emitHostStartReady();
}

void loop() {
  const unsigned long now_ms = millis();
  sampleAs5600(now_ms);

  if (!controlRunActive) {
    readSensor();
    if (consumeHostStartRequest()) {
      startControlRun();
    }
    stepper.runSpeed();
    return;
  }

  updateReferenceProfile();

  const bool encoderOk = as5600Ok(now_ms);
  if (!encoderOk) {
    if (!as5600FaultActive) {
      Serial.print(F("AS5600_FAULT,stale_ms="));
      Serial.println(lastAs5600ValidMs == 0UL ? -1L : static_cast<long>(now_ms - lastAs5600ValidMs));
      as5600FaultActive = true;
    }
    applySafeDisable();
    emitTelemetry();
    stepper.runSpeed();
    return;
  }
  if (as5600FaultActive) {
    Serial.println(F("AS5600_RECOVER"));
    resetControllerState(ResetMode::kBumplessResume);
    thetaCommandDeg = thetaMeasuredDeg;
    as5600FaultActive = false;
  }

  if (!readSensor()) {
    if (invalidReadStreak == 0) {
      invalidSequenceStartMs = now_ms;
      graceHoldTargetSteps = stepper.currentPosition();
      graceHoldThetaDeg = thetaMeasuredDeg;
    }
    ++invalidReadStreak;

    const unsigned long invalidDurationMs = now_ms - invalidSequenceStartMs;
    const bool graceActive =
        (invalidDurationMs < INVALID_GRACE_MS || invalidReadStreak <= INVALID_GRACE_READS);

    if (!invalidFallbackActive) {
      resetControllerState(ResetMode::kBumplessResume);
      invalidFallbackActive = true;
    }
    (void)graceActive;
    input = 0.0;
    output = 0.0;
    holdGracePosition();
    updateInnerLoop(now_ms);
    emitTelemetry();
    return;
  }

  invalidReadStreak = 0;
  invalidSequenceStartMs = 0UL;
  if (invalidFallbackActive) {
    resetControllerState(ResetMode::kBumplessResume);
    thetaCommandDeg = thetaMeasuredDeg;
  }
  invalidFallbackActive = false;

  if (lastControlComputeMs == 0UL || now_ms - lastControlComputeMs >= PID_SAMPLE_TIME_MS) {
    const unsigned long sampleIntervalMs =
        lastControlComputeMs == 0UL ? PID_SAMPLE_TIME_MS : (now_ms - lastControlComputeMs);
    lastControlComputeMs = now_ms;

    input = targetDistanceCm - ball_position;
    const bool regulateNow = fabs(input) <= REGULATION_BAND_CM;
    const bool demandTowardNear = input < 0.0;
    const double dt_s = static_cast<double>(sampleIntervalMs) / 1000.0;
    const double velocityAlpha = dt_s / (VELOCITY_FILTER_TAU_S + dt_s);
    filteredBallVelocityCmS += velocityAlpha * (ballVelocityCmS - filteredBallVelocityCmS);
    const bool holdWindow = fabs(input) <= SETPOINT_HOLD_POS_TOL_CM &&
                            fabs(filteredBallVelocityCmS) <= SETPOINT_HOLD_RATE_TOL_CM_S;
    const double desiredOutput =
        [&]() {
          const double kp =
              regulateNow ? (demandTowardNear ? REGULATION_TOWARD_NEAR_KP
                                              : REGULATION_TOWARD_FAR_KP)
                          : (demandTowardNear ? ACQUIRE_TOWARD_NEAR_KP : ACQUIRE_TOWARD_FAR_KP);
          const double ki =
              regulateNow ? (demandTowardNear ? REGULATION_TOWARD_NEAR_KI
                                              : REGULATION_TOWARD_FAR_KI)
                          : (demandTowardNear ? ACQUIRE_TOWARD_NEAR_KI : ACQUIRE_TOWARD_FAR_KI);
          const double kd =
              regulateNow ? (demandTowardNear ? REGULATION_TOWARD_NEAR_KD
                                              : REGULATION_TOWARD_FAR_KD)
                          : (demandTowardNear ? ACQUIRE_TOWARD_NEAR_KD : ACQUIRE_TOWARD_FAR_KD);
          const double controlLimit = controlLimitSteps(regulateNow, demandTowardNear);
          const double integralLimit =
              regulateNow ? REGULATION_INTEGRAL_LIMIT_STEPS : ACQUIRE_INTEGRAL_LIMIT_STEPS;
          const bool integralSignMismatch = input * integralOutputSteps > 0.0;
          const bool biasSignMismatch = input * biasOutputSteps > 0.0;
          const double unwindAlpha = dt_s / (INTEGRAL_UNWIND_TAU_S + dt_s);
          const double leakAlpha = dt_s / (INTEGRAL_LEAK_TAU_S + dt_s);
          const double biasUnwindAlpha = dt_s / (BIAS_UNWIND_TAU_S + dt_s);
          const double biasLeakAlpha = dt_s / (BIAS_LEAK_TAU_S + dt_s);
          const bool stalledOffTarget = fabs(input) >= BIAS_ACTIVE_POS_TOL_CM &&
                                        fabs(filteredBallVelocityCmS) <= BIAS_ACTIVE_RATE_TOL_CM_S;
          if (!holdWindow && integralSignMismatch) {
            integralOutputSteps *= (1.0 - unwindAlpha);
          } else if (!holdWindow && fabs(input) <= INTEGRAL_ACTIVE_BAND_CM &&
                     fabs(filteredBallVelocityCmS) <= INTEGRAL_ACTIVE_RATE_TOL_CM_S) {
            integralOutputSteps -= ki * input * dt_s;
          } else if (!holdWindow) {
            integralOutputSteps *= (1.0 - leakAlpha);
          }
          integralOutputSteps = clampAbs(integralOutputSteps, integralLimit);
          if (!holdWindow && biasSignMismatch) {
            biasOutputSteps *= (1.0 - biasUnwindAlpha);
          } else if (!holdWindow && stalledOffTarget) {
            biasOutputSteps -= BIAS_ADAPT_RATE_STEPS_PER_CM_S * input * dt_s;
          } else if (!holdWindow) {
            biasOutputSteps *= (1.0 - biasLeakAlpha);
          }
          biasOutputSteps = clampAbs(biasOutputSteps, BIAS_LIMIT_STEPS);
          const double pTermSteps = -kp * input;
          const double dTermSteps = kd * filteredBallVelocityCmS;
          if (holdWindow) {
            return clampAbs(integralOutputSteps + biasOutputSteps, controlLimit);
          }
          return clampAbs(pTermSteps + dTermSteps + integralOutputSteps + biasOutputSteps,
                          controlLimit);
        }();
    const double maxDelta = outputSlewLimitSteps(regulateNow, demandTowardNear);
    output = slewToward(output, desiredOutput, maxDelta);
    thetaCommandDeg = output * degreesPerStep;
  }
  emitTelemetry();
  updateInnerLoop(now_ms);
}
