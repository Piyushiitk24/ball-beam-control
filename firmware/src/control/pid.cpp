#include "control/pid.h"

namespace bb {
namespace {

float clampf(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

}  // namespace

PID::PID() : gains_(), integral_(0.0f), prev_error_(0.0f), has_prev_(false) {}

PID::PID(const PIDGains& gains)
    : gains_(gains), integral_(0.0f), prev_error_(0.0f), has_prev_(false) {}

void PID::setGains(const PIDGains& gains) { gains_ = gains; }

const PIDGains& PID::gains() const { return gains_; }

void PID::reset() {
  integral_ = 0.0f;
  prev_error_ = 0.0f;
  has_prev_ = false;
}

float PID::update(float error, float dt_s) {
  if (dt_s <= 0.0f) {
    return 0.0f;
  }

  float derivative = 0.0f;
  if (has_prev_) {
    derivative = (error - prev_error_) / dt_s;
  }

  // Evaluate saturation tendency before integration.
  const float pre_int_output =
      gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
  const bool sat_high = (pre_int_output >= gains_.out_max);
  const bool sat_low = (pre_int_output <= gains_.out_min);

  bool allow_integral = true;
  if ((sat_high && error > 0.0f) || (sat_low && error < 0.0f)) {
    allow_integral = false;
  }

  if (allow_integral) {
    integral_ += error * dt_s;
    integral_ = clampf(integral_, gains_.i_min, gains_.i_max);
  }

  float output = gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
  output = clampf(output, gains_.out_min, gains_.out_max);

  prev_error_ = error;
  has_prev_ = true;

  return output;
}

float PID::integral() const { return integral_; }

float PID::lastError() const { return prev_error_; }

}  // namespace bb
