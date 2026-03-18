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
// Ball-and-beam position is effectively a double-integrator plant, so a plain
// PD controller is the standard simple choice: proportional position error plus
// derivative damping from measured ball velocity. Avoid the recent rule-heavy
// capture logic that turned the loop into saturated bang-bang control.
constexpr double KP = 20.0;
constexpr double KI = 0.0;
constexpr double KD = 8.0;
constexpr double SETPOINT_HOLD_POS_TOL_CM = 0.35;
constexpr double SETPOINT_HOLD_RATE_TOL_CM_S = 1.2;
constexpr int CENTER_LIMIT_FULL_STEPS = 8;
constexpr int TRACKING_LIMIT_FULL_STEPS = 10;

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
constexpr double MIN_VALID_DISTANCE_CM = 8.0;
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

constexpr ReferenceProfile ACTIVE_REFERENCE_PROFILE = ReferenceProfile::kCenterFarNear90s;
constexpr unsigned long REFERENCE_PHASE_MS = 30000UL;
constexpr unsigned long kRefSerialBaud = 115200UL;
constexpr int PID_SAMPLE_TIME_MS = 100;
constexpr int WARMUP_SAMPLES = 10;
constexpr unsigned long WARMUP_DELAY_MS = 40UL;  // one sensor cycle per warmup sample

double raw_distance_cm = 0.0;
double ball_position = 0.0;
double targetDistanceCm = CENTER_DISTANCE_CM;
double input = 0.0;
double output = 0.0;
double radiansPerStep = 0.0;
double ballVelocityCmS = 0.0;
double integralOutputSteps = 0.0;
bool invalidFallbackActive = false;
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

double clampOutput(double value) {
  const bool targetIsCenterNow = fabs(targetDistanceCm - CENTER_DISTANCE_CM) <= 0.05;
  const int fullSteps = targetIsCenterNow ? CENTER_LIMIT_FULL_STEPS : TRACKING_LIMIT_FULL_STEPS;
  const double stepperMax = fullSteps * MICROSTEPS;
  const double stepperMin = -stepperMax;
  if (value > stepperMax) return stepperMax;
  if (value < stepperMin) return stepperMin;
  return value;
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
  const long current_steps = stepper.currentPosition();
  const double beam_angle_rad = static_cast<double>(current_steps) * radiansPerStep;

  Serial.print(F("TEL,"));
  Serial.print(millis());
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
  integralOutputSteps = 0.0;
  lastControlComputeMs = millis();
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

  profileStartMs = millis();
  emitReferenceConfig();
  updateReferenceProfile();
  digitalWrite(ENABLE_PIN, LOW);
  resetControllerState(ResetMode::kAcquireTarget);
}

void loop() {
  stepper.run();  // non-blocking stepper tick at top of every iteration
  updateReferenceProfile();

  if (!readSensor()) {
    const unsigned long now_ms = millis();

    if (invalidReadStreak == 0) {
      invalidSequenceStartMs = now_ms;
      graceHoldTargetSteps = stepper.currentPosition();
    }
    ++invalidReadStreak;

    const unsigned long invalidDurationMs = now_ms - invalidSequenceStartMs;
    // Grace applies to ALL invalid readings (out-of-range and low-voltage).
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
    lastControlComputeMs = now_ms;

    input = targetDistanceCm - ball_position;
    const bool holdWindow = fabs(input) <= SETPOINT_HOLD_POS_TOL_CM &&
                            fabs(ballVelocityCmS) <= SETPOINT_HOLD_RATE_TOL_CM_S;
    integralOutputSteps = 0.0;
    if (holdWindow) {
      output = 0.0;
    } else {
      const double pTermSteps = -KP * input;
      const double dTermSteps = KD * ballVelocityCmS;
      output = clampOutput(pTermSteps + dTermSteps);
    }
  }
  emitTelemetry();
  move();
  stepper.run();
}
