#include "sensors/as5600_sensor.h"

#include "calibration_runtime.h"

namespace bb {
namespace {

constexpr uint8_t kRawAngleHighReg = 0x0C;
constexpr uint8_t kRawAngleLowReg = 0x0D;

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

}  // namespace bb
