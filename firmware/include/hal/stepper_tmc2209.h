#pragma once

#include <Arduino.h>

namespace bb {

class StepperTMC2209 {
 public:
  StepperTMC2209(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin);

  void begin();
  void enable(bool enabled);
  bool isEnabled() const;

  void stop();
  void setSignedStepRate(float signed_rate_sps);
  float targetSignedStepRate() const;

  void requestJogSteps(long signed_steps, float abs_rate_sps);
  bool jogActive() const;

  void serviceStepPulse(uint32_t now_us);

 private:
  uint8_t step_pin_;
  uint8_t dir_pin_;
  uint8_t en_pin_;

  bool enabled_;
  bool step_level_;
  bool direction_positive_;

  float signed_rate_sps_;
  float abs_rate_sps_;

  long jog_steps_remaining_;
  bool jog_mode_;

  uint32_t last_toggle_us_;
};

}  // namespace bb
