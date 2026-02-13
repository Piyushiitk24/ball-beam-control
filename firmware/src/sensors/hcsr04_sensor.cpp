#include "sensors/hcsr04_sensor.h"

#include <math.h>

#include "calibration.h"
#include "config.h"

namespace bb {

HCSR04Sensor::HCSR04Sensor(uint8_t trig_pin, uint8_t echo_pin)
    : trig_pin_(trig_pin),
      echo_pin_(echo_pin),
      history_(),
      history_count_(0),
      history_index_(0),
      ema_initialized_(false),
      ema_cm_(0.0f) {}

void HCSR04Sensor::begin() {
  pinMode(trig_pin_, OUTPUT);
  pinMode(echo_pin_, INPUT);

  digitalWrite(trig_pin_, LOW);
  delayMicroseconds(5);
}

bool HCSR04Sensor::readDistanceCm(float& distance_cm) {
  digitalWrite(trig_pin_, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin_, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin_, LOW);

  const unsigned long duration_us = pulseIn(echo_pin_, HIGH, kSonarEchoTimeoutUs);
  if (duration_us == 0) {
    return false;
  }

  distance_cm = static_cast<float>(duration_us) * 0.0343f * 0.5f;
  return true;
}

bool HCSR04Sensor::readPositionCm(float& x_cm,
                                  float& x_filt_cm,
                                  float& distance_raw_cm) {
  if (!readDistanceCm(distance_raw_cm)) {
    return false;
  }

  pushHistory(distance_raw_cm);
  const float median_cm = medianHistory();

  if (!ema_initialized_) {
    ema_cm_ = median_cm;
    ema_initialized_ = true;
  } else {
    ema_cm_ = (kSonarEmaAlpha * median_cm) + ((1.0f - kSonarEmaAlpha) * ema_cm_);
  }

  x_cm = mapBallPosCm(distance_raw_cm);
  x_filt_cm = mapBallPosCm(ema_cm_);
  return true;
}

void HCSR04Sensor::pushHistory(float sample_cm) {
  history_[history_index_] = sample_cm;
  history_index_ = (history_index_ + 1) % kWindow;
  if (history_count_ < kWindow) {
    ++history_count_;
  }
}

float HCSR04Sensor::medianHistory() const {
  if (history_count_ == 0) {
    return 0.0f;
  }

  float tmp[kWindow];
  for (uint8_t i = 0; i < history_count_; ++i) {
    tmp[i] = history_[i];
  }

  for (uint8_t i = 0; i + 1 < history_count_; ++i) {
    for (uint8_t j = 0; j + 1 < history_count_ - i; ++j) {
      if (tmp[j] > tmp[j + 1]) {
        const float swap = tmp[j];
        tmp[j] = tmp[j + 1];
        tmp[j + 1] = swap;
      }
    }
  }

  const uint8_t mid = history_count_ / 2;
  if ((history_count_ % 2U) == 0U) {
    return 0.5f * (tmp[mid - 1] + tmp[mid]);
  }
  return tmp[mid];
}

}  // namespace bb
