#pragma once

#include <Arduino.h>

namespace bb {

class StepperTMC2209 {
 public:
  StepperTMC2209(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin);

  void begin();
  void beginScheduler();
  void endScheduler();

  void enable(bool enabled);
  bool isEnabled() const;

  void stop();
  void setSignedStepRate(float signed_rate_sps);
  float targetSignedStepRate() const;

  // Integrated signed position in STEP pulses. Increments/decrements on each
  // rising edge according to the DIR state actually driven to the motor.
  int32_t positionSteps() const;
  void resetPositionSteps(int32_t steps = 0);

  void requestJogSteps(long signed_steps, float abs_rate_sps);
  bool jogActive() const;

  void processIsrFlags();
  void handleTimerCompareIsr();

 private:
  static constexpr uint16_t kTimerHz = 40000;  // 25us tick at prescaler 8
  static constexpr uint16_t kMinHalfPeriodTicks = 1;
  static constexpr uint16_t kMaxHalfPeriodTicks = 60000;

  uint8_t step_pin_;
  uint8_t dir_pin_;
  uint8_t en_pin_;

  volatile uint8_t* step_port_out_;
  uint8_t step_mask_;
  volatile uint8_t* dir_port_out_;
  uint8_t dir_mask_;

  volatile bool scheduler_started_;
  volatile bool enabled_;
  volatile bool step_level_;
  volatile bool pulse_active_;
  volatile bool direction_positive_;
  volatile int8_t step_dir_sign_;

  float signed_rate_sps_;
  float abs_rate_sps_;

  volatile uint16_t half_period_ticks_;
  volatile uint16_t ticks_until_toggle_;

  volatile int32_t jog_steps_remaining_;
  volatile bool jog_mode_;
  volatile bool jog_finished_flag_;

  volatile int32_t position_steps_;

  uint16_t rateToHalfPeriodTicks(float abs_rate_sps) const;
  void setStepPinLevelFast(bool high);
  void setDirPinFast(bool positive);
};

}  // namespace bb
