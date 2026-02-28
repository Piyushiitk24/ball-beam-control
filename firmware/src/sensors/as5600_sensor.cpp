#include "sensors/as5600_sensor.h"

#include "calibration_runtime.h"

namespace bb {
namespace {

constexpr uint8_t kRawAngleHighReg = 0x0C;
constexpr uint8_t kRawAngleLowReg = 0x0D;
constexpr uint8_t kStatusReg = 0x0B;
constexpr uint8_t kAgcReg = 0x1A;
constexpr uint8_t kMagnitudeHighReg = 0x1B;

}  // namespace

AS5600Sensor::AS5600Sensor() : wire_(nullptr), address_(0x36), error_count_(0) {}

bool AS5600Sensor::begin(TwoWire& wire, uint8_t address) {
  wire_ = &wire;
  address_ = address;

  wire_->begin();

  wire_->beginTransmission(address_);
  return (wire_->endTransmission() == 0);
}

bool AS5600Sensor::readRaw(uint16_t& raw) {
  if (wire_ == nullptr) {
    ++error_count_;
    return false;
  }

  wire_->beginTransmission(address_);
  wire_->write(kRawAngleHighReg);
  if (wire_->endTransmission(false) != 0) {
    ++error_count_;
    return false;
  }

  const uint8_t requested = wire_->requestFrom(address_, static_cast<uint8_t>(2));
  if (requested != 2 || wire_->available() < 2) {
    ++error_count_;
    return false;
  }

  const uint8_t high = wire_->read();
  const uint8_t low = wire_->read();
  raw = static_cast<uint16_t>(((high & 0x0F) << 8) | low);
  return true;
}

bool AS5600Sensor::readRawAngleDeg(float& raw_angle_deg) {
  uint16_t raw = 0;
  if (!readRaw(raw)) {
    return false;
  }

  raw_angle_deg = (static_cast<float>(raw) * 360.0f) / 4096.0f;
  return true;
}

bool AS5600Sensor::readBeamThetaDeg(float& theta_deg, float& raw_angle_deg) {
  if (!readRawAngleDeg(raw_angle_deg)) {
    return false;
  }

  theta_deg = runtimeMapThetaDeg(raw_angle_deg);
  return true;
}

uint32_t AS5600Sensor::errorCount() const { return error_count_; }

bool AS5600Sensor::readRegByte(uint8_t reg, uint8_t& out) {
  if (wire_ == nullptr) {
    return false;
  }
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) {
    return false;
  }
  if (wire_->requestFrom(address_, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  out = wire_->read();
  return true;
}

bool AS5600Sensor::readRegWord(uint8_t reg, uint16_t& out) {
  if (wire_ == nullptr) {
    return false;
  }
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) {
    return false;
  }
  if (wire_->requestFrom(address_, static_cast<uint8_t>(2)) < 2) {
    return false;
  }
  const uint8_t high = wire_->read();
  const uint8_t low = wire_->read();
  out = static_cast<uint16_t>((high << 8) | low);
  return true;
}

AS5600Status AS5600Sensor::readStatus() {
  AS5600Status s;
  uint8_t status_byte = 0;
  if (!readRegByte(kStatusReg, status_byte)) {
    return s;
  }
  s.magnet_detected = (status_byte & 0x20) != 0;  // MD bit5
  s.too_strong      = (status_byte & 0x08) != 0;   // MH bit3
  s.too_weak        = (status_byte & 0x10) != 0;   // ML bit4

  uint8_t agc = 0;
  if (readRegByte(kAgcReg, agc)) {
    s.agc = agc;
  }

  uint16_t mag = 0;
  if (readRegWord(kMagnitudeHighReg, mag)) {
    s.magnitude = mag & 0x0FFF;
  }

  s.read_ok = true;
  return s;
}

}  // namespace bb
