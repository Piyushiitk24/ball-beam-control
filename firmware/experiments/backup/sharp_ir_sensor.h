#pragma once

#include <Arduino.h>

#include "config.h"
#include "sensors/sensor_types.h"

namespace bb {

/// Sharp GP2Y0A21YK0F analog IR distance sensor driver.
/// Reads an analog pin, converts voltage → distance via empirical power-law
/// fit, applies EMA smoothing, and exposes the same position interface as the
/// legacy HC-SR04 / TFMini drivers.
class SharpIRSensor {
 public:
  explicit SharpIRSensor(uint8_t analog_pin);

  void begin();
  void service(uint32_t now_ms);

  bool getPosition(float& x_cm, float& x_filt_cm, float& distance_raw_cm) const;
  bool hasFreshSample(uint32_t now_ms) const;
  uint32_t sampleAgeMs(uint32_t now_ms) const;
  bool hasTimeout() const;
  void getDiag(uint32_t now_ms, SonarDiag& diag) const;

 private:
  uint8_t pin_;

  bool has_sample_;
  float last_distance_cm_;
  float last_x_cm_;
  float last_x_filt_cm_;
  uint32_t last_sample_ms_;
  uint32_t last_read_ms_;

  bool ema_initialized_;
  float ema_cm_;

  uint16_t valid_streak_;
  uint16_t timeout_count_;

  bool timeout_flag_;

  static float adcToDistanceCm(uint16_t adc_raw);
};

}  // namespace bb
