#include <AccelStepper.h>
#include <Arduino.h>
#include <PID_v1.h>
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

// Gains tuned from run analysis.
// KP = 18.0: raised from 13 to ensure far-end recovery is friction-proof.
//   At 8 cm error (ball at far end, CENTER=12): P = 144 steps = 16.2°.
//   With run_173954 KP=13: D-noise (±50 steps from Kalman jitter at far end)
//   could drop net output to 54 steps (6°) — below friction threshold of ~8.5°
//   (76 steps). Ball got stuck at far end. KP=18 gives min net = 144-50 = 94
//   steps (10.6°), reliably above friction regardless of D-term oscillation.
//   Near center (0.5 cm error): P = 9 steps = 1° — below friction, same as before.
// KI = 0.3: prevents integral windup.
// KD = 2.5: PID_v1 internally scales kd = Kd / SampleTimeInSec = 2.5/0.1 = 25.
//   At 25 cm/s approach (2.5 cm/100ms sample), D provides ~62 steps of braking.
constexpr double KP = 18.0;
constexpr double KI = 0.3;
constexpr double KD = 2.5;
constexpr double SET_POINT = 0.0;

// CENTER_DISTANCE_CM is the Sharp reading when the ball is at the target
// equilibrium position. Sharp is mounted at the FULCRUM/PIVOT end, so it reads
// SMALL when the ball is near the sensor and LARGE when the ball is at the
// far/motor end.
//
// Target equilibrium. Set to the sensor reading at the physical ball rest
// position. In run_173954 the ball averaged 11.64 cm when placed at physical
// centre; CENTER=13.0 caused 22s of negative integral buildup (ball couldn't
// reach 13 cm due to friction at small beam angles) that weakened far-end
// recovery. 12.0 matches actual resting position, keeping integral near zero.
constexpr double CENTER_DISTANCE_CM = 12.0;

// Working Sharp window for the current mount. Near-end fold-back is handled
// by the physical runner stop (ball cannot reach the fold-back zone < 8 cm).
// MAX = 22.0 covers the full far-end range so the controller can pull the
// ball in from the far end (~20 cm).
// MIN = 9.0: Sharp GP2Y0A21YK0F folds back below ~8 cm (output voltage drops,
// power-law maps 4-5 cm physical distance back to ~10 cm). Near-end stop must
// be at ≥9 cm from sensor face to stay in the monotonic region.
constexpr double MIN_VALID_DISTANCE_CM = 9.0;
constexpr double MAX_VALID_DISTANCE_CM = 27.0;

// Grace period extended and applies to ALL invalid reading types.
constexpr unsigned long INVALID_GRACE_MS = 600UL;
constexpr int INVALID_GRACE_READS = 15;


constexpr unsigned long kRefSerialBaud = 115200UL;
constexpr int PID_SAMPLE_TIME_MS = 100;
constexpr unsigned long TELEMETRY_INTERVAL_MS = 100UL;
constexpr int WARMUP_SAMPLES = 10;
constexpr unsigned long WARMUP_DELAY_MS = 40UL;  // one sensor cycle per warmup sample

double raw_distance_cm = 0.0;
double ball_position = 0.0;
double setPoint = SET_POINT;
double input = 0.0;
double output = 0.0;
double radiansPerStep = 0.0;
bool invalidFallbackActive = false;
unsigned long invalidSequenceStartMs = 0UL;
int invalidReadStreak = 0;
long graceHoldTargetSteps = 0;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
SimpleKalmanFilter filter(MEASUREMENT_ERROR, MEASUREMENT_ERROR, VARIANCE);
PID pid(&input, &output, &setPoint, KP, KI, KD, DIRECT);

// GP2Y0A21YK0F empirical power-law: distance_cm = 29.988 * voltage^(-1.173)
float sharpAdcToCm(uint16_t adc) {
  if (adc < 10) return 0.0f;
  const float v = static_cast<float>(adc) * (5.0f / 1024.0f);
  if (v < 0.05f) return 0.0f;
  return 29.988f * powf(v, -1.173f);
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
  // Sign audit run_20260317_164432: positive AccelStepper steps → distance
  // INCREASES (pivot end rises, ball rolls toward motor). Negate output so that
  // a positive PID output (ball too far) commands negative steps → distance
  // DECREASES → ball moves toward sensor.
  stepper.moveTo(lround(-output));
}

void holdGracePosition() { stepper.moveTo(graceHoldTargetSteps); }

void resetPidState() {
  // Preload output for bumpless transfer: desired = lround(-output), so to
  // make desired == currentPosition we need output = -currentPosition.
  output = -static_cast<double>(stepper.currentPosition());
  input = ball_position;
  pid.SetMode(MANUAL);
  pid.SetMode(AUTOMATIC);
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
  // No DIR inversion: positive AccelStepper steps → pivot UP → distance
  // increases. move() negates output so positive PID output → negative steps
  // → distance decreases → ball toward sensor. See move() comment.

  const int stepperMax = 25 * MICROSTEPS;  // ±400 steps ≈ ±45° — matches reference design
  const int stepperMin = -stepperMax;
  pid = PID(&input, &output, &setPoint, KP, KI, KD, DIRECT);
  pid.SetSampleTime(PID_SAMPLE_TIME_MS);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(stepperMin, stepperMax);

  for (int i = 0; i < WARMUP_SAMPLES; ++i) {
    readSensor();
    delay(WARMUP_DELAY_MS);
  }

  digitalWrite(ENABLE_PIN, LOW);
}

void loop() {
  stepper.run();

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
      resetPidState();
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
  invalidFallbackActive = false;

  // input = distance error: positive when ball is too close, negative when too far.
  // setPoint = 0, so PID drives this to zero → ball_position → CENTER_DISTANCE_CM.
  input = CENTER_DISTANCE_CM - ball_position;
  pid.Compute();
  emitTelemetry();
  move();
  stepper.run();
}
