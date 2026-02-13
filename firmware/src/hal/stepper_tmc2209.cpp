#include "hal/stepper_tmc2209.h"

#include <math.h>

#include "calibration.h"
#include "config.h"

namespace bb {

StepperTMC2209::StepperTMC2209(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin)
    : step_pin_(step_pin),
      dir_pin_(dir_pin),
      en_pin_(en_pin),
      enabled_(false),
      step_level_(false),
      direction_positive_(true),
      signed_rate_sps_(0.0f),
      abs_rate_sps_(0.0f),
      jog_steps_remaining_(0),
      jog_mode_(false),
      last_toggle_us_(0) {}

void StepperTMC2209::begin() {
  pinMode(step_pin_, OUTPUT);
  pinMode(dir_pin_, OUTPUT);
  pinMode(en_pin_, OUTPUT);

  digitalWrite(step_pin_, LOW);
  digitalWrite(dir_pin_, LOW);
  digitalWrite(en_pin_, HIGH);  // active-low enable

  enabled_ = false;
  step_level_ = false;
  last_toggle_us_ = micros();
}

void StepperTMC2209::enable(bool enabled) {
  enabled_ = enabled;
  digitalWrite(en_pin_, enabled ? LOW : HIGH);
}

bool StepperTMC2209::isEnabled() const { return enabled_; }

void StepperTMC2209::stop() {
  signed_rate_sps_ = 0.0f;
  abs_rate_sps_ = 0.0f;
  jog_mode_ = false;
  jog_steps_remaining_ = 0;
  step_level_ = false;
  digitalWrite(step_pin_, LOW);
}

void StepperTMC2209::setSignedStepRate(float signed_rate_sps) {
  if (signed_rate_sps > kMaxStepRateSps) {
    signed_rate_sps = kMaxStepRateSps;
  } else if (signed_rate_sps < -kMaxStepRateSps) {
    signed_rate_sps = -kMaxStepRateSps;
  }

  signed_rate_sps_ = signed_rate_sps;
  abs_rate_sps_ = fabsf(signed_rate_sps_);

  const float hardware_dir_reference =
      signed_rate_sps_ * static_cast<float>(STEPPER_DIR_SIGN);
  direction_positive_ = (hardware_dir_reference >= 0.0f);
  digitalWrite(dir_pin_, direction_positive_ ? HIGH : LOW);
}

float StepperTMC2209::targetSignedStepRate() const { return signed_rate_sps_; }

void StepperTMC2209::requestJogSteps(long signed_steps, float abs_rate_sps) {
  if (signed_steps == 0) {
    stop();
    return;
  }

  if (abs_rate_sps < 1.0f) {
    abs_rate_sps = 1.0f;
  }

  jog_steps_remaining_ = labs(signed_steps);
  jog_mode_ = true;
  setSignedStepRate((signed_steps > 0) ? abs_rate_sps : -abs_rate_sps);
}

bool StepperTMC2209::jogActive() const { return jog_mode_ && jog_steps_remaining_ > 0; }

void StepperTMC2209::serviceStepPulse(uint32_t now_us) {
  if (!enabled_ || abs_rate_sps_ < 1.0f) {
    return;
  }

  uint32_t half_period_us = static_cast<uint32_t>(500000.0f / abs_rate_sps_);
  if (half_period_us < 150) {
    half_period_us = 150;
  }

  if (static_cast<uint32_t>(now_us - last_toggle_us_) < half_period_us) {
    return;
  }

  last_toggle_us_ = now_us;
  step_level_ = !step_level_;
  digitalWrite(step_pin_, step_level_ ? HIGH : LOW);

  // Count only rising edges as actual steps.
  if (step_level_ && jog_mode_ && jog_steps_remaining_ > 0) {
    --jog_steps_remaining_;
    if (jog_steps_remaining_ == 0) {
      jog_mode_ = false;
      signed_rate_sps_ = 0.0f;
      abs_rate_sps_ = 0.0f;
      step_level_ = false;
      digitalWrite(step_pin_, LOW);
    }
  }
}

}  // namespace bb
