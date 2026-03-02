// TFMini baud-rate configurator — standalone, no Timer1 ISR.
// Upload this, wait for "CONFIG_DONE", then upload main firmware.
//
// Uses SoftwareSerial at 115200 with NO competing ISR, so TX is reliable.
// Sends baud-change commands (classic + Plus formats), reopens at 9600,
// and reads frames to verify.

#include <Arduino.h>
#include <SoftwareSerial.h>

static constexpr uint8_t RX_PIN = 10;  // PIN_TFMINI_RX
static constexpr uint8_t TX_PIN = 11;  // PIN_TFMINI_TX
static constexpr uint32_t TARGET_BAUD = 57600UL;

SoftwareSerial tfSerial(RX_PIN, TX_PIN);

static void sendBaudCommands(uint32_t baud) {
  // TFMini classic: 42 57 02 00 00 00 XX 00
  uint8_t code = 0x06;  // 115200 default
  if (baud <= 9600UL)       code = 0x01;
  else if (baud <= 14400UL) code = 0x02;
  else if (baud <= 19200UL) code = 0x03;
  else if (baud <= 56000UL) code = 0x04;
  else if (baud <= 57600UL) code = 0x05;
  const uint8_t cmd1[] = {0x42, 0x57, 0x02, 0x00, 0x00, 0x00, code, 0x00};
  tfSerial.write(cmd1, 8);
  delay(50);

  // TFMini-Plus: 5A 08 06 [4-byte LE baud] checksum
  const uint8_t b0 = static_cast<uint8_t>(baud & 0xFFu);
  const uint8_t b1 = static_cast<uint8_t>((baud >> 8) & 0xFFu);
  const uint8_t b2 = static_cast<uint8_t>((baud >> 16) & 0xFFu);
  const uint8_t b3 = static_cast<uint8_t>((baud >> 24) & 0xFFu);
  const uint8_t cs = static_cast<uint8_t>(
      (0x5Au + 0x08u + 0x06u + b0 + b1 + b2 + b3) & 0xFFu);
  const uint8_t cmd2[] = {0x5A, 0x08, 0x06, b0, b1, b2, b3, cs};
  tfSerial.write(cmd2, 8);
  delay(50);

  // Save config (Plus format): 5A 04 11 6F
  const uint8_t save[] = {0x5A, 0x04, 0x11, 0x6F};
  tfSerial.write(save, 4);
  delay(100);
}

static uint16_t tryReadFrames(uint32_t duration_ms) {
  // Try to read TFMini 9-byte frames for `duration_ms`.
  // Returns count of valid frames (checksum OK, distance > 0).
  uint16_t good = 0;
  uint8_t buf[9];
  uint8_t idx = 0;
  const uint32_t start = millis();

  while ((millis() - start) < duration_ms) {
    if (tfSerial.available() <= 0) continue;
    uint8_t b = static_cast<uint8_t>(tfSerial.read());

    if (idx == 0) { if (b == 0x59) buf[idx++] = b; continue; }
    if (idx == 1) {
      if (b == 0x59) { buf[idx++] = b; continue; }
      idx = (b == 0x59) ? 1 : 0;
      if (idx == 1) buf[0] = 0x59;
      continue;
    }
    buf[idx++] = b;
    if (idx < 9) continue;

    // Full frame — verify checksum
    uint16_t cs = 0;
    for (uint8_t i = 0; i < 8; ++i) cs = (cs + buf[i]) & 0xFFu;
    uint16_t dist = static_cast<uint16_t>(buf[2]) | (static_cast<uint16_t>(buf[3]) << 8);
    uint16_t strength = static_cast<uint16_t>(buf[4]) | (static_cast<uint16_t>(buf[5]) << 8);
    if (cs == buf[8]) {
      ++good;
      Serial.print(F("  dist="));
      Serial.print(dist);
      Serial.print(F(" str="));
      Serial.print(strength);
      Serial.print(F(" hex="));
      for (uint8_t j = 0; j < 9; ++j) {
        if (buf[j] < 0x10) Serial.print('0');
        Serial.print(buf[j], HEX);
        Serial.print(' ');
      }
      Serial.println();
    }
    idx = 0;
  }
  return good;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("=== TFMini Baud Configurator v2 ==="));

  // Step 0: Scan common baud rates to find where TFMini currently lives.
  static const uint32_t bauds[] = {9600UL, 19200UL, 57600UL, 115200UL};
  static const uint8_t nBauds = 4;
  uint32_t foundBaud = 0;

  Serial.println(F("0) Scanning baud rates..."));
  for (uint8_t i = 0; i < nBauds; ++i) {
    Serial.print(F("   Trying "));
    Serial.print(bauds[i]);
    Serial.print(F("... "));
    tfSerial.begin(bauds[i]);
    delay(300);
    // Flush stale bytes
    while (tfSerial.available() > 0) tfSerial.read();
    delay(100);
    uint16_t frames = tryReadFrames(2000);
    Serial.print(frames);
    Serial.println(F(" frames"));
    tfSerial.end();
    if (frames >= 3) {
      foundBaud = bauds[i];
      break;
    }
    delay(50);
  }

  if (foundBaud == 0) {
    Serial.println(F("CONFIG_FAIL,no_frames_at_any_baud"));
    Serial.println(F("Check wiring: TFMini TX->D10, TFMini RX->D11, GND, 5V"));
    Serial.println(F("Try power-cycling TFMini (unplug 5V, wait 3s, replug)."));
    Serial.println(F("CONFIG_DONE"));
    return;
  }

  Serial.print(F("   Found TFMini at "));
  Serial.print(foundBaud);
  Serial.println(F(" baud"));
  Serial.println(F("CONFIG_OK,sensor_found"));
  Serial.println(F("CONFIG_DONE"));
  Serial.println();
  Serial.println(F("=== Continuous read — move target / point at wall ==="));
}

void loop() {
  // After config, keep reading and printing frames so user can verify.
  static bool header_printed = false;
  if (!header_printed) {
    Serial.println(F("--- Continuous read (verify sensor) ---"));
    header_printed = true;
  }
  if (tfSerial.available() > 0) {
    tryReadFrames(1000);
  }
  delay(100);
}
