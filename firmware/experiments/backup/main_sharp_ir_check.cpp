// Sharp GP2Y0A21YK0F standalone check — no Timer1, no stepper, just sensor
// reads to verify wiring and distance response.
//
// Wiring: Vo (yellow) → A0, VCC (red) → 5V, GND (black) → GND
//
// Expected output (move target 10–80 cm from sensor):
//   SHARP_IR,raw_adc=<val>,voltage=<V>,distance_cm=<d>

#include <Arduino.h>

constexpr uint8_t PIN_SHARP_IR = A0;
constexpr uint32_t SAMPLE_PERIOD_MS = 40;  // ~25 Hz (sensor internal cycle)
constexpr uint32_t SERIAL_BAUD_RATE = 115200;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(200);
  Serial.println(F("SHARP_IR_CHECK_BOOT"));
  Serial.println(F("Wiring: Vo(yellow)->A0  VCC(red)->5V  GND(black)->GND"));
  Serial.println(F("Move target 10-80 cm from sensor..."));
  Serial.println(F("==========================================="));
  pinMode(PIN_SHARP_IR, INPUT);
}

void loop() {
  static uint32_t last_ms = 0;
  const uint32_t now_ms = millis();

  if (static_cast<uint32_t>(now_ms - last_ms) < SAMPLE_PERIOD_MS) {
    return;
  }
  last_ms = now_ms;

  const uint16_t adc_raw = static_cast<uint16_t>(analogRead(PIN_SHARP_IR));
  const float voltage = static_cast<float>(adc_raw) * (5.0f / 1024.0f);

  float distance_cm = 0.0f;
  if (adc_raw >= 10 && voltage >= 0.05f) {
    distance_cm = 29.988f * powf(voltage, -1.173f);
  }

  Serial.print(F("SHARP_IR,raw_adc="));
  Serial.print(adc_raw);
  Serial.print(F(",voltage="));
  Serial.print(voltage, 3);
  Serial.print(F(",distance_cm="));
  Serial.println(distance_cm, 1);
}
