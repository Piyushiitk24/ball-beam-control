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
// KP raised 5→8: more force at medium error (4 cm at 17 cm → 32 steps = 3.6°),
//   so ball doesn't stall 14s waiting for integral to build up.
// KI dropped 1.0→0.3: prevents integral windup from dominating when ball crosses
//   centre; with KI=1.0 the integral accumulated ~76 units in 14s and kept output
//   positive even when ball was at 9 cm past centre.
// KD = 2.5: PID_v1 internally scales kd = Kd / SampleTimeInSec = 2.5/0.1 = 25.
//   At 25 cm/s approach (2.5 cm/100ms sample), D provides ~62 steps of braking.
//   KD=6.0 was tried but kd_per_compute=60 caused ±30-120 step D swings from
//   0.5-2 cm filtered noise, overwhelming P and making the ball completely stuck.
constexpr double KP = 13.0;
constexpr double KI = 0.3;
constexpr double KD = 2.5;
constexpr double SET_POINT = 0.0;

// CENTER_DISTANCE_CM is the Sharp reading when the ball is at the physical
// centre of the runner. Sharp is mounted at the FULCRUM/PIVOT end (same as
// HC-SR04), so it reads SMALL (~7.5 cm) when the ball is near the sensor and
// LARGE (~20 cm) when the ball is at the far/motor end.
//
// TO CALIBRATE: run sharp_ir_check, place the ball at the physical centre
// of the runner, and update this value to match the reading.
constexpr double CENTER_DISTANCE_CM = 13.0;

// Working Sharp window for the current mount.
// MIN = 9.0: Sharp GP2Y0A21YK0F folds back below ~8 cm (output voltage drops,
// power-law maps 4-5 cm physical distance back to ~10 cm). Near-end stop must
// be at ≥9 cm from sensor face to stay in the monotonic region.
// MAX = 27.0 covers the full far-end range with margin.
constexpr double MIN_VALID_DISTANCE_CM = 9.0;
constexpr double MAX_VALID_DISTANCE_CM = 27.0;
// Grace period extended and applies to ALL invalid reading types.
constexpr unsigned long INVALID_GRACE_MS = 600UL;
constexpr int INVALID_GRACE_READS = 15;
constexpr unsigned long kRefSerialBaud = 115200UL;
constexpr int PID_SAMPLE_TIME_MS = 100;
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
  // Sign audit run_20260317_164432: positive AccelStepper steps → distance
  // INCREASES (pivot end rises, ball rolls toward motor). Negate output so that
  // a positive PID output (ball too far) commands negative steps → distance
  // DECREASES → ball moves toward sensor.
  stepper.moveTo(lround(-output));
}

void holdGracePosition() { stepper.moveTo(graceHoldTargetSteps); }

void resetPidState() {
  // Preload for bumpless transfer: desired = lround(-output), so to make
  // desired == currentPosition we set output = -currentPosition.
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
  // → distance decreases → ball toward sensor.

  // Reduced from 25 * MICROSTEPS (400 steps = ±45°) to 6 * MICROSTEPS
  // (96 steps ≈ ±11°). The linearization is valid only within ±4° (±35 steps);
  // ±11° provides a working margin while preventing runaway integral windup.
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
  stepper.run();  // non-blocking stepper tick at top of every iteration

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
