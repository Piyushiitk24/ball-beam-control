#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "config.h"
#include "sensors/hcsr04_sensor.h"

namespace bb {

class TFMiniSensor {
 public:
  TFMiniSensor(uint8_t rx_pin, uint8_t tx_pin);

  void begin();
  void service(uint32_t now_ms);

  bool getPosition(float& x_cm, float& x_filt_cm, float& distance_raw_cm) const;
  bool hasFreshSample(uint32_t now_ms) const;
  uint32_t sampleAgeMs(uint32_t now_ms) const;
  bool hasTimeout() const;
  void getDiag(uint32_t now_ms, SonarDiag& diag) const;

 private:
  static constexpr uint8_t kFrameSize = 9;
  static constexpr uint8_t kWindow = SONAR_MEDIAN_WINDOW;
  static constexpr uint8_t kMinValidStreak = 1;

  SoftwareSerial serial_;

  uint8_t frame_[kFrameSize];
  uint8_t frame_index_;
  uint32_t last_rx_ms_;

  float history_[kWindow];
  uint8_t valid_flags_[kWindow];
  uint8_t history_count_;
  uint8_t history_index_;
  uint8_t valid_count_;

  bool ema_initialized_;
  float ema_cm_;

  bool has_sample_;
  float last_distance_cm_;
  float last_x_cm_;
  float last_x_filt_cm_;
  uint32_t last_sample_ms_;
  uint16_t valid_streak_;
  uint16_t timeout_count_;
  uint16_t jump_reject_count_;

  bool timeout_flag_;
  bool stale_timeout_latched_;

  void pushAttempt(bool valid, float sample_cm);
  float medianHistory() const;
  void parseByte(uint8_t byte, uint32_t now_ms);
  void processFrame(const uint8_t frame[kFrameSize], uint32_t now_ms);
};

}  // namespace bb

