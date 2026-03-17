#include <AccelStepper.h>
#include <Arduino.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "calibration.h"

namespace {

constexpr uint8_t PIN_STEP = 2;
constexpr uint8_t PIN_DIR = 3;
constexpr uint8_t PIN_ENABLE = 4;
constexpr uint8_t PIN_SHARP_IR = A0;

constexpr unsigned long kSerialBaud = 115200UL;
constexpr unsigned long kSamplePeriodMs = 40UL;
constexpr long kDefaultJogSteps = bb::SIGN_CAL_JOG_STEPS;
constexpr float kDefaultJogRateSps = bb::SIGN_CAL_JOG_RATE_SPS;
constexpr float kMaxJogRateSps = 3000.0f;
constexpr float kJogAccelerationSps2 = 20000.0f;

AccelStepper g_stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

bool g_stream_enabled = true;
bool g_driver_enabled = false;
unsigned long g_last_sample_ms = 0UL;
uint16_t g_last_raw_adc = 0U;
float g_last_distance_cm = 0.0f;
char g_cmd_buf[64];
size_t g_cmd_len = 0U;

float sharpAdcToCm(uint16_t adc) {
  if (adc < 10U) {
    return 0.0f;
  }

  const float voltage = static_cast<float>(adc) * (5.0f / 1024.0f);
  if (voltage < 0.05f) {
    return 0.0f;
  }

  return 29.988f * powf(voltage, -1.173f);
}

void captureSharpSample() {
  g_last_raw_adc = static_cast<uint16_t>(analogRead(PIN_SHARP_IR));
  g_last_distance_cm = sharpAdcToCm(g_last_raw_adc);
  g_last_sample_ms = millis();
}

bool captureSharpSampleIfDue() {
  const unsigned long now_ms = millis();
  if (now_ms - g_last_sample_ms < kSamplePeriodMs) {
    return false;
  }

  captureSharpSample();
  return true;
}

void emitTelemetry(bool force_sample) {
  if (force_sample) {
    captureSharpSample();
  } else {
    if (!captureSharpSampleIfDue()) {
      return;
    }
  }

  Serial.print(F("AUDIT_TEL,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(g_last_raw_adc);
  Serial.print(',');
  Serial.print(g_last_distance_cm, 4);
  Serial.print(',');
  Serial.println(g_stepper.currentPosition());
}

void printHelp() {
  Serial.println(F("SHARP_SIGN_AUDIT_HELP"));
  Serial.println(F("Commands:"));
  Serial.println(F("  h / ?            help"));
  Serial.println(F("  e 0|1            disable / enable driver"));
  Serial.println(F("  s                emit one AUDIT_TEL sample"));
  Serial.println(F("  t                toggle AUDIT_TEL streaming"));
  Serial.println(F("  z                zero logical step position"));
  Serial.println(F("  j <steps> [rate] blocking jog, holds final position"));
}

void printUsageError(const __FlashStringHelper* usage) {
  Serial.print(F("ERR,usage,"));
  Serial.println(usage);
}

bool parseLongValue(const char* text, long& out) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char* end = nullptr;
  const long value = strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }

  out = value;
  return true;
}

bool parseFloatValue(const char* text, float& out) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char* end = nullptr;
  const float value = static_cast<float>(strtod(text, &end));
  if (end == text || *end != '\0') {
    return false;
  }

  out = value;
  return true;
}

void setDriverEnabled(bool enabled) {
  g_driver_enabled = enabled;
  digitalWrite(PIN_ENABLE, enabled ? LOW : HIGH);
  Serial.print(F("OK,driver_enabled="));
  Serial.println(enabled ? F("1") : F("0"));
}

void zeroStepPosition() {
  g_stepper.setCurrentPosition(0L);
  Serial.println(F("OK,step_pos=0"));
}

void runJogBlocking(long signed_steps, float abs_rate_sps) {
  if (!g_driver_enabled) {
    Serial.println(F("ERR,driver_disabled"));
    return;
  }
  if (signed_steps == 0L) {
    Serial.println(F("ERR,zero_steps"));
    return;
  }

  if (!(abs_rate_sps > 0.0f)) {
    abs_rate_sps = kDefaultJogRateSps;
  }
  if (abs_rate_sps > kMaxJogRateSps) {
    abs_rate_sps = kMaxJogRateSps;
  }

  const long start_step = g_stepper.currentPosition();
  const long target_step = start_step + signed_steps;
  const float signed_rate_sps = (signed_steps > 0L) ? abs_rate_sps : -abs_rate_sps;

  Serial.print(F("AUDIT_JOG_BEGIN,"));
  Serial.print(signed_steps);
  Serial.print(',');
  Serial.print(abs_rate_sps, 3);
  Serial.print(',');
  Serial.println(start_step);

  g_stepper.moveTo(target_step);
  g_stepper.setSpeed(signed_rate_sps);
  while (g_stepper.distanceToGo() != 0L) {
    g_stepper.runSpeedToPosition();
    if (g_stream_enabled) {
      emitTelemetry(false);
    }
  }

  Serial.print(F("AUDIT_JOG_END,"));
  Serial.println(g_stepper.currentPosition());
}

void handleEnableCommand(char* args) {
  if (args == nullptr || *args == '\0') {
    printUsageError(F("e 0|1"));
    return;
  }

  long enabled = 0L;
  if (!parseLongValue(args, enabled) || (enabled != 0L && enabled != 1L)) {
    printUsageError(F("e 0|1"));
    return;
  }

  setDriverEnabled(enabled == 1L);
}

void handleJogCommand(char* args) {
  if (args == nullptr || *args == '\0') {
    printUsageError(F("j <signed_steps> [abs_rate_sps]"));
    return;
  }

  char* rate_token = nullptr;
  char* steps_token = strtok_r(args, " \t", &rate_token);
  char* abs_rate_token = strtok_r(nullptr, " \t", &rate_token);

  long signed_steps = 0L;
  if (!parseLongValue(steps_token, signed_steps)) {
    printUsageError(F("j <signed_steps> [abs_rate_sps]"));
    return;
  }

  float abs_rate_sps = kDefaultJogRateSps;
  if (abs_rate_token != nullptr && !parseFloatValue(abs_rate_token, abs_rate_sps)) {
    printUsageError(F("j <signed_steps> [abs_rate_sps]"));
    return;
  }

  runJogBlocking(signed_steps, abs_rate_sps);
}

void handleCommand(char* line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (*line == '\0') {
    return;
  }

  char* args = line;
  while (*args != '\0' && *args != ' ' && *args != '\t') {
    ++args;
  }
  if (*args != '\0') {
    *args++ = '\0';
    while (*args == ' ' || *args == '\t') {
      ++args;
    }
  } else {
    args = nullptr;
  }

  if (strcmp(line, "h") == 0 || strcmp(line, "?") == 0) {
    printHelp();
    return;
  }
  if (strcmp(line, "e") == 0) {
    handleEnableCommand(args);
    return;
  }
  if (strcmp(line, "s") == 0) {
    emitTelemetry(true);
    return;
  }
  if (strcmp(line, "t") == 0) {
    g_stream_enabled = !g_stream_enabled;
    Serial.print(F("OK,stream="));
    Serial.println(g_stream_enabled ? F("1") : F("0"));
    return;
  }
  if (strcmp(line, "z") == 0) {
    zeroStepPosition();
    return;
  }
  if (strcmp(line, "j") == 0) {
    handleJogCommand(args);
    return;
  }

  Serial.print(F("ERR,unknown_command,cmd="));
  Serial.println(line);
  Serial.println(F("INFO,try=h"));
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      g_cmd_buf[g_cmd_len] = '\0';
      handleCommand(g_cmd_buf);
      g_cmd_len = 0U;
      continue;
    }
    if (g_cmd_len + 1U < sizeof(g_cmd_buf)) {
      g_cmd_buf[g_cmd_len++] = c;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  delay(200);

  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);  // active-low enable
  pinMode(PIN_SHARP_IR, INPUT);

  g_stepper.setMaxSpeed(kDefaultJogRateSps);
  g_stepper.setAcceleration(kJogAccelerationSps2);
  g_stepper.setMinPulseWidth(5);
  g_stepper.setCurrentPosition(0L);

  captureSharpSample();

  Serial.print(F("AUDIT_BOOT,baud="));
  Serial.print(kSerialBaud);
  Serial.print(F(",sample_ms="));
  Serial.print(kSamplePeriodMs);
  Serial.print(F(",default_jog_steps="));
  Serial.print(kDefaultJogSteps);
  Serial.print(F(",default_jog_rate_sps="));
  Serial.print(kDefaultJogRateSps, 3);
  Serial.print(F(",stream="));
  Serial.println(g_stream_enabled ? F("1") : F("0"));
  Serial.println(F("Wiring: STEP->D2 DIR->D3 EN->D4 Sharp->A0"));
  printHelp();
}

void loop() {
  pollSerial();

  if (g_stream_enabled) {
    emitTelemetry(false);
  }
}
