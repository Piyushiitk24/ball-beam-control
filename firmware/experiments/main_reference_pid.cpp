#include <AccelStepper.h>
#include <Arduino.h>
#include <HCSR04.h>
#include <PID_v1.h>
#include <SimpleKalmanFilter.h>

#include <math.h>

namespace {

constexpr int STEP_PIN = 2;
constexpr int DIR_PIN = 3;
constexpr int TRIG_PIN = 8;
constexpr int ECHO_PIN = 9;
constexpr int ENABLE_PIN = 4;
constexpr int MICROSTEPS = 16;

constexpr double MEASUREMENT_ERROR = 0.04;
constexpr double VARIANCE = 1.0;
constexpr double KP = 3.836778110512597;
constexpr double KI = 1.4835192523619078;
constexpr double KD = 1.4009765095107707;
constexpr double SET_POINT = 0.0;
constexpr double CENTER_DISTANCE_CM = 13.5;
constexpr double MIN_VALID_DISTANCE_CM = 2.0;
constexpr double MAX_VALID_DISTANCE_CM = 25.0;
constexpr unsigned long kRefSerialBaud = 115200UL;
constexpr int PID_SAMPLE_TIME_MS = 100;
constexpr int WARMUP_SAMPLES = 10;
constexpr unsigned long WARMUP_DELAY_MS = 200UL;

double raw_distance_cm = 0.0;
double ball_position = 0.0;
double setPoint = SET_POINT;
double input = 0.0;
double output = 0.0;
double radiansPerStep = 0.0;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
HCSR04 sensor(TRIG_PIN, ECHO_PIN);
SimpleKalmanFilter filter(MEASUREMENT_ERROR, MEASUREMENT_ERROR, VARIANCE);
PID pid(&input, &output, &setPoint, KP, KI, KD, DIRECT);

bool readSensor() {
  raw_distance_cm = sensor.dist();
  if (raw_distance_cm > MIN_VALID_DISTANCE_CM && raw_distance_cm < MAX_VALID_DISTANCE_CM) {
    ball_position = filter.updateEstimate(raw_distance_cm);
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

void move() { stepper.runToNewPosition(-output); }

}  // namespace

void setup() {
  Serial.begin(kRefSerialBaud);
  Serial.println(F("BALL_BEAM_REF_BOOT"));

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);  // TMC2209 enable is active-low.

  const double degreesPerStep = 360.0 / (200.0 * MICROSTEPS);
  radiansPerStep = degreesPerStep * (PI / 180.0);

  stepper.setMaxSpeed(25000.0);
  stepper.setAcceleration(15000.0);
  stepper.setMinPulseWidth(5);

  const int stepperMax = 25 * MICROSTEPS;
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
  if (!readSensor()) {
    input = 0.0;
    output = 0.0;
    emitTelemetry();
    stepper.runToNewPosition(0);
    return;
  }

  const double h = CENTER_DISTANCE_CM - ball_position;
  const double a = static_cast<double>(stepper.currentPosition()) * radiansPerStep;
  input = h * cos(a);
  pid.Compute();
  emitTelemetry();
  move();
}
