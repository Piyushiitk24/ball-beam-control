#include <AccelStepper.h>
#include <Arduino.h>
#include <SimpleKalmanFilter.h>

namespace {

constexpr int STEP_PIN = 2;
constexpr int DIR_PIN = 3;
constexpr int ENABLE_PIN = 4;
constexpr int MICROSTEPS = 16;

// Sharp GP2Y0A21YK0F on A0. Wiring: Vo->A0, VCC->5V, GND->GND.
constexpr uint8_t SHARP_PIN = A0;
// Sharp sensor internal cycle is ~38 ms; do not sample faster.
constexpr unsigned long SENSOR_SAMPLE_MS = 40UL;

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
// more damped. The remaining missing piece is a slow within-run trim estimate:
// when the ball stalls far from target, plain P/D stops growing and never
// learns the static bias needed to pull it back.
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
constexpr double BIAS_ADAPT_RATE_STEPS_PER_CM_S = 0.45;
constexpr double BIAS_ACTIVE_POS_TOL_CM = 1.0;
constexpr double BIAS_ACTIVE_RATE_TOL_CM_S = 0.8;
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
double prevValidBallPositionCm = 0.0;
bool hasPrevValidBallPosition = false;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
SimpleKalmanFilter filter(MEASUREMENT_ERROR, MEASUREMENT_ERROR, VARIANCE);

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

double controlLimitSteps(bool regulateNow, bool demandTowardNear) {
  const int fullSteps =
      regulateNow ? (demandTowardNear ? REGULATION_TOWARD_NEAR_LIMIT_FULL_STEPS
                                      : REGULATION_TOWARD_FAR_LIMIT_FULL_STEPS)
                  : (demandTowardNear ? ACQUIRE_TOWARD_NEAR_LIMIT_FULL_STEPS
                                      : ACQUIRE_TOWARD_FAR_LIMIT_FULL_STEPS);
  return static_cast<double>(fullSteps * MICROSTEPS);
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
  filteredBallVelocityCmS = ballVelocityCmS;
  output = 0.0;
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
  const double beam_angle_rad = static_cast<double>(current_steps) * radiansPerStep;

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
  Serial.println(beam_angle_rad, 6);
}

void move() {
  // Positive AccelStepper steps move the ball away from the sensor on the live
  // Sharp setup, so negate the PID output at the stepper boundary:
  //   positive PID output (ball too far) -> negative steps -> ball toward sensor
  //   negative PID output (ball too close) -> positive steps -> ball away
  stepper.moveTo(lround(-output));
}

void holdGracePosition() { stepper.moveTo(graceHoldTargetSteps); }

void resetControllerState(ResetMode mode) {
  (void)mode;
  input = targetDistanceCm - ball_position;
  output = 0.0;
  filteredBallVelocityCmS = ballVelocityCmS;
  integralOutputSteps = 0.0;
  lastControlComputeMs = millis();
}

void startControlRun() {
  if (controlRunActive) return;
  invalidFallbackActive = false;
  invalidReadStreak = 0;
  invalidSequenceStartMs = 0UL;
  biasOutputSteps = 0.0;
  graceHoldTargetSteps = stepper.currentPosition();
  profileStartMs = millis();
  activeReferencePhase = -1;
  emitReferenceConfig();
  updateReferenceProfile();
  digitalWrite(ENABLE_PIN, LOW);
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

  const double degreesPerStep = 360.0 / (200.0 * MICROSTEPS);
  radiansPerStep = degreesPerStep * (PI / 180.0);

  stepper.setMaxSpeed(1000.0);
  stepper.setAcceleration(15000.0);
  stepper.setMinPulseWidth(5);
  // No DIR inversion. The stepper boundary mapping is handled in move().

  for (int i = 0; i < WARMUP_SAMPLES; ++i) {
    readSensor();
    delay(WARMUP_DELAY_MS);
  }

  emitHostStartReady();
}

void loop() {
  if (!controlRunActive) {
    readSensor();
    if (consumeHostStartRequest()) {
      startControlRun();
    }
    return;
  }

  stepper.run();
  updateReferenceProfile();

  if (!readSensor()) {
    const unsigned long now_ms = millis();
    if (invalidReadStreak == 0) {
      invalidSequenceStartMs = now_ms;
      graceHoldTargetSteps = stepper.currentPosition();
    }
    ++invalidReadStreak;

    const unsigned long invalidDurationMs = now_ms - invalidSequenceStartMs;
    const bool graceActive =
        (invalidDurationMs < INVALID_GRACE_MS || invalidReadStreak <= INVALID_GRACE_READS);

    if (graceActive) {
      holdGracePosition();
      stepper.run();
      emitTelemetry();
      return;
    }

    if (!invalidFallbackActive) {
      resetControllerState(ResetMode::kBumplessResume);
      invalidFallbackActive = true;
    }
    input = 0.0;
    output = 0.0;
    stepper.moveTo(0);
    stepper.run();
    emitTelemetry();
    return;
  }

  invalidReadStreak = 0;
  invalidSequenceStartMs = 0UL;
  if (invalidFallbackActive) {
    resetControllerState(ResetMode::kBumplessResume);
  }
  invalidFallbackActive = false;

  const unsigned long now_ms = millis();
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
  }
  emitTelemetry();
  move();
  stepper.run();
}
