#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace bb {

class AS5600Sensor {
 public:
  AS5600Sensor();

  bool begin(TwoWire& wire = Wire, uint8_t address = 0x36);

  bool readRaw(uint16_t& raw);
  bool readRawAngleDeg(float& raw_angle_deg);
  bool readBeamThetaDeg(float& theta_deg, float& raw_angle_deg);

  uint32_t errorCount() const;

 private:
  TwoWire* wire_;
  uint8_t address_;
  uint32_t error_count_;
};

}  // namespace bb
