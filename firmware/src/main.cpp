#include <Arduino.h>
#include <Wire.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "app/state_machine.h"
#include "calibration.h"
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

StepperTMC2209 g_stepper(PIN_STEP, PIN_DIR, PIN_EN);
AS5600Sensor g_as5600;
HCSR04Sensor g_hcsr04(PIN_TRIG, PIN_ECHO);
CascadeController g_controller;
StateMachine g_state_machine;

SensorData g_sensor;
Setpoint g_setpoint;
FaultFlags g_fault_flags;

bool g_sign_calibrated = false;
bool g_inner_loop_stable = true;
bool g_manual_enable = false;

uint32_t g_last_control_ms = 0;
uint32_t g_last_telemetry_ms = 0;
uint32_t g_last_valid_sensor_ms = 0;

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

bool hasAnyFault() {
  return g_fault_flags.sonar_timeout || g_fault_flags.i2c_error ||
         g_fault_flags.angle_oob || g_fault_flags.pos_oob;
}

void applySafeDisable() {
  g_stepper.stop();
  g_stepper.enable(false);
}

bool readSensors(uint32_t now_ms) {
  g_sensor.ts_ms = now_ms;

  float theta_deg = 0.0f;
  float theta_raw_deg = 0.0f;
  const bool angle_ok = g_as5600.readBeamThetaDeg(theta_deg, theta_raw_deg);
  if (angle_ok) {
    g_sensor.beam_angle_deg = theta_deg;
    g_sensor.beam_angle_raw_deg = theta_raw_deg;
    g_fault_flags.i2c_error = false;
  } else {
    g_fault_flags.i2c_error = true;
  }
  g_sensor.valid_angle = angle_ok;

  float x_cm = 0.0f;
  float x_filt_cm = 0.0f;
  float dist_cm = 0.0f;
  const bool pos_ok = g_hcsr04.readPositionCm(x_cm, x_filt_cm, dist_cm);
  if (pos_ok) {
    g_sensor.ball_pos_cm = x_cm;
    g_sensor.ball_pos_filt_cm = x_filt_cm;
    g_sensor.sonar_distance_raw_cm = dist_cm;
    g_fault_flags.sonar_timeout = false;
  } else {
    g_fault_flags.sonar_timeout = true;
  }
  g_sensor.valid_pos = pos_ok;

  if (angle_ok && pos_ok) {
    g_last_valid_sensor_ms = now_ms;
  }

  if (angle_ok) {
    g_fault_flags.angle_oob = fabsf(g_sensor.beam_angle_deg) > kThetaHardLimitDeg;
  }
  if (pos_ok) {
    g_fault_flags.pos_oob = fabsf(g_sensor.ball_pos_filt_cm) > kBallPosHardLimitCm;
  }

  return angle_ok && pos_ok;
}

void updateFaultState(uint32_t now_ms) {
  if (static_cast<uint32_t>(now_ms - g_last_valid_sensor_ms) > kSensorInvalidFaultMs) {
    g_fault_flags.sonar_timeout = true;
    g_fault_flags.i2c_error = true;
  }

  if (hasAnyFault()) {
    g_state_machine.requestFault();
    applySafeDisable();
  }
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  status"));
  Serial.println(F("  en 0|1"));
  Serial.println(F("  jog <signed_steps> <rate>"));
  Serial.println(F("  cal_sign begin"));
  Serial.println(F("  cal_sign save"));
  Serial.println(F("  cal_zero angle"));
  Serial.println(F("  cal_zero position"));
  Serial.println(F("  run"));
  Serial.println(F("  stop"));
  Serial.println(F("  fault_reset"));
}

void printStatus() {
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

  Serial.print(F("FAULTS,bits="));
  Serial.println(packFaultFlags(g_fault_flags));

  Serial.print(F("SIGNS,current_stepper="));
  Serial.print(STEPPER_DIR_SIGN);
  Serial.print(F(",current_as5600="));
  Serial.print(AS5600_SIGN);
  Serial.print(F(",current_sonar="));
  Serial.println(SONAR_POS_SIGN);

  Serial.print(F("CAL,sign_calibrated="));
  Serial.println(g_sign_calibrated ? F("yes") : F("no"));
}

void emitTelemetry(uint32_t now_ms) {
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
    g_stepper.serviceStepPulse(micros());
    if (static_cast<uint32_t>(millis() - start_ms) > timeout_ms) {
      g_stepper.stop();
      break;
    }
  }
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

  g_sign_cal.near_distance_cm = g_sensor.sonar_distance_raw_cm;
  g_sign_cal.theta_before_raw_deg = g_sensor.beam_angle_raw_deg;

  g_stepper.enable(true);
  g_stepper.requestJogSteps(SIGN_CAL_JOG_STEPS, SIGN_CAL_JOG_RATE_SPS);
  runJogBlocking(3000);

  delay(50);
  readSensors(millis());
  g_sign_cal.theta_after_raw_deg = g_sensor.beam_angle_raw_deg;
  g_sign_cal.theta_delta_raw_deg = wrapAngleDeltaDeg(
      g_sign_cal.theta_after_raw_deg - g_sign_cal.theta_before_raw_deg);

  const float mapped_delta =
      static_cast<float>(AS5600_SIGN) * g_sign_cal.theta_delta_raw_deg;

  g_sign_cal.suggested_stepper_sign =
      (mapped_delta >= 0.0f) ? STEPPER_DIR_SIGN : static_cast<int8_t>(-STEPPER_DIR_SIGN);

  g_sign_cal.suggested_as5600_sign =
      (g_sign_cal.theta_delta_raw_deg >= 0.0f) ? 1 : -1;

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

  if (!readSensors(millis())) {
    Serial.println(F("ERR,sensors_not_ready"));
    return;
  }

  g_sign_cal.far_distance_cm = g_sensor.sonar_distance_raw_cm;
  g_sign_cal.suggested_sonar_sign =
      (g_sign_cal.far_distance_cm >= g_sign_cal.near_distance_cm) ? 1 : -1;

  g_sign_cal.complete = true;
  g_sign_cal.active = false;
  g_sign_calibrated = true;

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

  Serial.println(F("INFO,update firmware/include/calibration.h and rebuild."));
}

AppConditions currentConditions() {
  AppConditions cond;
  cond.sign_calibrated = g_sign_calibrated;
  cond.sensors_ok = g_sensor.valid_angle && g_sensor.valid_pos;
  cond.faults_active = hasAnyFault();
  cond.inner_loop_stable = g_inner_loop_stable;
  return cond;
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
      Serial.println(F("ERR,usage:cal_zero angle|position"));
      return;
    }

    if (!readSensors(millis())) {
      Serial.println(F("ERR,sensors_not_ready"));
      return;
    }

    if (strcmp(sub, "angle") == 0) {
      Serial.print(F("CAL_SUGGEST,AS5600_ZERO_DEG="));
      Serial.println(g_sensor.beam_angle_raw_deg, 6);
      return;
    }

    if (strcmp(sub, "position") == 0) {
      Serial.print(F("CAL_SUGGEST,SONAR_CENTER_CM="));
      Serial.println(g_sensor.sonar_distance_raw_cm, 6);
      return;
    }

    Serial.println(F("ERR,usage:cal_zero angle|position"));
    return;
  }

  if (strcmp(token, "run") == 0) {
    readSensors(millis());
    updateFaultState(millis());

    if (g_state_machine.requestRun(currentConditions())) {
      g_controller.reset();
      g_setpoint.ball_pos_cm_target = kDefaultBallSetpointCm;
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

    const ActuatorCmd cmd = g_controller.update(g_sensor, g_setpoint, kControlDtSec);
    if (cmd.enable) {
      g_stepper.enable(true);
      g_stepper.setSignedStepRate(cmd.signed_step_rate_sps);
    } else {
      applySafeDisable();
    }
    return;
  }

  if (g_stepper.jogActive()) {
    g_stepper.enable(true);
    return;
  }

  if (g_manual_enable && !hasAnyFault()) {
    g_stepper.enable(true);
  } else {
    applySafeDisable();
  }
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(250);

  Serial.println(F("BALL_BEAM_BOOT"));

  g_stepper.begin();
  applySafeDisable();

  Wire.begin();
  const bool i2c_ok = g_as5600.begin(Wire, AS5600_I2C_ADDR);
  g_hcsr04.begin();

  g_fault_flags = FaultFlags{};
  if (!i2c_ok) {
    g_fault_flags.i2c_error = true;
  }

  g_setpoint.ball_pos_cm_target = kDefaultBallSetpointCm;

  g_last_control_ms = millis();
  g_last_telemetry_ms = g_last_control_ms;
  g_last_valid_sensor_ms = g_last_control_ms;

  printHelp();
  printStatus();
}

void loop() {
  g_stepper.serviceStepPulse(micros());
  pollSerial();

  const uint32_t now_ms = millis();

  if (static_cast<uint32_t>(now_ms - g_last_control_ms) >= kControlPeriodMs) {
    g_last_control_ms = now_ms;
    serviceControl(now_ms);
  }

  if (static_cast<uint32_t>(now_ms - g_last_telemetry_ms) >= kTelemetryPeriodMs) {
    g_last_telemetry_ms = now_ms;
    emitTelemetry(now_ms);
  }
}
