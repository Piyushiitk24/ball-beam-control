#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace bb {

struct AS5600Status {
  bool magnet_detected = false;
  bool too_strong = false;
  bool too_weak = false;
  uint8_t agc = 0;
  uint16_t magnitude = 0;
  bool read_ok = false;
};

class AS5600Sensor {
 public:
  AS5600Sensor();

  bool begin(TwoWire& wire = Wire, uint8_t address = 0x36);

  bool readRaw(uint16_t& raw);
  bool readRawAngleDeg(float& raw_angle_deg);
  bool readBeamThetaDeg(float& theta_deg, float& raw_angle_deg);

  AS5600Status readStatus();
  uint32_t errorCount() const;

 private:
  TwoWire* wire_;
  uint8_t address_;
  uint32_t error_count_;

  bool readRegByte(uint8_t reg, uint8_t& out);
  bool readRegWord(uint8_t reg, uint16_t& out);
};

}  // namespace bb
