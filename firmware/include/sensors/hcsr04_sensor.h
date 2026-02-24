#pragma once

#include <Arduino.h>

namespace bb {

struct SonarDiag {
  bool has_sample = false;
  bool timeout = false;
  bool fresh = false;
  uint32_t age_ms = 0;
  float raw_cm = 0.0f;
  float filt_cm = 0.0f;
  uint16_t valid_streak = 0;
  uint16_t timeout_count = 0;
  uint16_t jump_reject_count = 0;
};

class HCSR04Sensor {
 public:
  HCSR04Sensor(uint8_t trig_pin, uint8_t echo_pin);

  void begin();
  void service(uint32_t now_us, uint32_t now_ms);

  bool getPosition(float& x_cm, float& x_filt_cm, float& distance_raw_cm) const;
  bool hasFreshSample(uint32_t now_ms) const;
  uint32_t sampleAgeMs(uint32_t now_ms) const;
  bool hasTimeout() const;
  void getDiag(uint32_t now_ms, SonarDiag& diag) const;

  void handleEchoEdgeIsr(uint32_t now_us, bool level_high);

 private:
  static constexpr uint8_t kWindow = 5;

  uint8_t trig_pin_;
  uint8_t echo_pin_;

  float history_[kWindow];
  uint8_t history_count_;
  uint8_t history_index_;

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
  uint32_t last_trigger_us_;
  bool waiting_echo_;

  volatile uint32_t rise_us_;
  volatile uint32_t pulse_width_us_;
  volatile bool awaiting_fall_;
  volatile bool sample_ready_;

  void pushHistory(float sample_cm);
  float medianHistory() const;
};

}  // namespace bb
