#include "sensors/hcsr04_sensor.h"

#include <math.h>
#include <util/atomic.h>

#include "calibration_runtime.h"
#include "config.h"

namespace bb {
namespace {

constexpr float kSonarMaxJumpCm = 18.0f;
constexpr uint8_t kSonarMinValidStreak = 2;

}  // namespace

HCSR04Sensor::HCSR04Sensor(uint8_t trig_pin, uint8_t echo_pin)
    : trig_pin_(trig_pin),
      echo_pin_(echo_pin),
      history_(),
      history_count_(0),
      history_index_(0),
      ema_initialized_(false),
      ema_cm_(0.0f),
      has_sample_(false),
      last_distance_cm_(0.0f),
      last_x_cm_(0.0f),
      last_x_filt_cm_(0.0f),
      last_sample_ms_(0),
      valid_streak_(0),
      timeout_count_(0),
      jump_reject_count_(0),
      timeout_flag_(false),
      last_trigger_us_(0),
      waiting_echo_(false),
      rise_us_(0),
      pulse_width_us_(0),
      awaiting_fall_(false),
      sample_ready_(false) {}

void HCSR04Sensor::begin() {
  pinMode(trig_pin_, OUTPUT);
  pinMode(echo_pin_, INPUT);

  digitalWrite(trig_pin_, LOW);
  delayMicroseconds(5);

  last_trigger_us_ = micros() - kSonarTriggerPeriodUs;
}

void HCSR04Sensor::service(uint32_t now_us, uint32_t now_ms) {
  if (!waiting_echo_ && static_cast<uint32_t>(now_us - last_trigger_us_) >= kSonarTriggerPeriodUs) {
    digitalWrite(trig_pin_, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin_, LOW);

    last_trigger_us_ = now_us;
    waiting_echo_ = true;
  }

  uint32_t pulse_width_us = 0;
  bool sample_ready = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (sample_ready_) {
      sample_ready_ = false;
      pulse_width_us = pulse_width_us_;
      sample_ready = true;
    }
  }

  if (sample_ready) {
    waiting_echo_ = false;
    if (pulse_width_us > 0 && pulse_width_us <= kSonarEchoTimeoutUs) {
      const float distance_cm = static_cast<float>(pulse_width_us) * 0.0343f * 0.5f;
      if (has_sample_ && fabsf(distance_cm - last_distance_cm_) > kSonarMaxJumpCm) {
        ++jump_reject_count_;
        valid_streak_ = 0;
        timeout_flag_ = true;
      } else {
        last_distance_cm_ = distance_cm;

        pushHistory(distance_cm);
        const float median_cm = medianHistory();

        if (!ema_initialized_) {
          ema_cm_ = median_cm;
          ema_initialized_ = true;
        } else {
          ema_cm_ = (kSonarEmaAlpha * median_cm) + ((1.0f - kSonarEmaAlpha) * ema_cm_);
        }

        last_x_cm_ = runtimeMapBallPosCm(last_distance_cm_);
        last_x_filt_cm_ = runtimeMapBallPosCm(ema_cm_);
        last_sample_ms_ = now_ms;
        has_sample_ = true;
        timeout_flag_ = false;
        if (valid_streak_ < 0xFFFFu) {
          ++valid_streak_;
        }
      }
    } else {
      timeout_flag_ = true;
      valid_streak_ = 0;
      if (timeout_count_ < 0xFFFFu) {
        ++timeout_count_;
      }
    }
  }

  if (waiting_echo_ && static_cast<uint32_t>(now_us - last_trigger_us_) > kSonarEchoTimeoutUs) {
    waiting_echo_ = false;
    timeout_flag_ = true;
    valid_streak_ = 0;
    if (timeout_count_ < 0xFFFFu) {
      ++timeout_count_;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      awaiting_fall_ = false;
      sample_ready_ = false;
    }
  }
}

bool HCSR04Sensor::getPosition(float& x_cm, float& x_filt_cm, float& distance_raw_cm) const {
  if (!has_sample_) {
    return false;
  }

  x_cm = last_x_cm_;
  x_filt_cm = last_x_filt_cm_;
  distance_raw_cm = last_distance_cm_;
  return true;
}

bool HCSR04Sensor::hasFreshSample(uint32_t now_ms) const {
  if (!has_sample_) {
    return false;
  }
  if (valid_streak_ < kSonarMinValidStreak) {
    return false;
  }
  return sampleAgeMs(now_ms) <= kPosSampleFreshMs;
}

uint32_t HCSR04Sensor::sampleAgeMs(uint32_t now_ms) const {
  if (!has_sample_) {
    return 0xFFFFFFFFUL;
  }
  return static_cast<uint32_t>(now_ms - last_sample_ms_);
}

bool HCSR04Sensor::hasTimeout() const { return timeout_flag_; }

void HCSR04Sensor::getDiag(uint32_t now_ms, SonarDiag& diag) const {
  diag.has_sample = has_sample_;
  diag.timeout = timeout_flag_;
  diag.age_ms = sampleAgeMs(now_ms);
  diag.fresh = hasFreshSample(now_ms);
  diag.raw_cm = last_distance_cm_;
  diag.filt_cm = ema_cm_;
  diag.valid_streak = valid_streak_;
  diag.timeout_count = timeout_count_;
  diag.jump_reject_count = jump_reject_count_;
}

void HCSR04Sensor::handleEchoEdgeIsr(uint32_t now_us, bool level_high) {
  if (level_high) {
    rise_us_ = now_us;
    awaiting_fall_ = true;
    return;
  }

  if (!awaiting_fall_) {
    return;
  }

  pulse_width_us_ = static_cast<uint32_t>(now_us - rise_us_);
  sample_ready_ = true;
  awaiting_fall_ = false;
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
