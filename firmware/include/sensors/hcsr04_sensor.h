#pragma once

#include <Arduino.h>

namespace bb {

class HCSR04Sensor {
 public:
  HCSR04Sensor(uint8_t trig_pin, uint8_t echo_pin);

  void begin();

  bool readDistanceCm(float& distance_cm);
  bool readPositionCm(float& x_cm, float& x_filt_cm, float& distance_raw_cm);

 private:
  static constexpr uint8_t kWindow = 5;

  uint8_t trig_pin_;
  uint8_t echo_pin_;

  float history_[kWindow];
  uint8_t history_count_;
  uint8_t history_index_;

  bool ema_initialized_;
  float ema_cm_;

  void pushHistory(float sample_cm);
  float medianHistory() const;
};

}  // namespace bb
