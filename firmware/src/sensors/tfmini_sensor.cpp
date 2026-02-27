#include "sensors/tfmini_sensor.h"

#include <math.h>

#include "calibration_runtime.h"

namespace bb {
namespace {

constexpr float kTfminiMaxJumpCm = 25.0f;
constexpr float kTfminiMinDistanceCm = 2.0f;
constexpr float kTfminiMaxDistanceCm = 0.1f * SONAR_MAX_VALID_MM;

}  // namespace

TFMiniSensor::TFMiniSensor(uint8_t rx_pin, uint8_t tx_pin)
    : serial_(rx_pin, tx_pin),
      frame_(),
      frame_index_(0),
      last_rx_ms_(0),
      history_(),
      valid_flags_(),
      history_count_(0),
      history_index_(0),
      valid_count_(0),
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
      stale_timeout_latched_(false) {}

void TFMiniSensor::begin() {
  serial_.begin(kTfminiUartBaud);
  frame_index_ = 0;
  last_rx_ms_ = millis();
}

void TFMiniSensor::service(uint32_t now_ms) {
  while (serial_.available() > 0) {
    const int value = serial_.read();
    if (value < 0) {
      break;
    }
    parseByte(static_cast<uint8_t>(value), now_ms);
  }

  if (static_cast<uint32_t>(now_ms - last_rx_ms_) > kTfminiReadStaleMs) {
    timeout_flag_ = true;
    if (!stale_timeout_latched_) {
      stale_timeout_latched_ = true;
      pushAttempt(false, 0.0f);
      if (timeout_count_ < 0xFFFFu) {
        ++timeout_count_;
      }
    }
  }
}

bool TFMiniSensor::getPosition(float& x_cm, float& x_filt_cm, float& distance_raw_cm) const {
  if (!has_sample_) {
    return false;
  }
  x_cm = last_x_cm_;
  x_filt_cm = last_x_filt_cm_;
  distance_raw_cm = last_distance_cm_;
  return true;
}

bool TFMiniSensor::hasFreshSample(uint32_t now_ms) const {
  if (!has_sample_) {
    return false;
  }
  if (valid_streak_ < kMinValidStreak) {
    return false;
  }
  return sampleAgeMs(now_ms) <= kPosSampleFreshMs;
}

uint32_t TFMiniSensor::sampleAgeMs(uint32_t now_ms) const {
  if (!has_sample_) {
    return 0xFFFFFFFFUL;
  }
  return static_cast<uint32_t>(now_ms - last_sample_ms_);
}

bool TFMiniSensor::hasTimeout() const { return timeout_flag_; }

void TFMiniSensor::getDiag(uint32_t now_ms, SonarDiag& diag) const {
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

void TFMiniSensor::parseByte(uint8_t byte, uint32_t now_ms) {
  if (frame_index_ == 0) {
    if (byte == 0x59) {
      frame_[frame_index_++] = byte;
    }
    return;
  }

  if (frame_index_ == 1) {
    if (byte == 0x59) {
      frame_[frame_index_++] = byte;
      return;
    }
    frame_index_ = (byte == 0x59) ? 1 : 0;
    if (frame_index_ == 1) {
      frame_[0] = 0x59;
    }
    return;
  }

  frame_[frame_index_++] = byte;
  if (frame_index_ < kFrameSize) {
    return;
  }

  processFrame(frame_, now_ms);
  frame_index_ = 0;
}

void TFMiniSensor::processFrame(const uint8_t frame[kFrameSize], uint32_t now_ms) {
  last_rx_ms_ = now_ms;
  stale_timeout_latched_ = false;

  uint16_t checksum = 0;
  for (uint8_t i = 0; i < (kFrameSize - 1); ++i) {
    checksum = static_cast<uint16_t>((checksum + frame[i]) & 0xFFu);
  }
  if (checksum != frame[kFrameSize - 1]) {
    timeout_flag_ = true;
    pushAttempt(false, 0.0f);
    if (timeout_count_ < 0xFFFFu) {
      ++timeout_count_;
    }
    return;
  }

  const uint16_t distance_cm_u16 =
      static_cast<uint16_t>(frame[2]) | static_cast<uint16_t>(frame[3] << 8);
  const float raw_cm = static_cast<float>(distance_cm_u16);
  const bool range_ok = (raw_cm >= kTfminiMinDistanceCm) && (raw_cm <= kTfminiMaxDistanceCm);

  float cm = raw_cm;
  if (range_ok && has_sample_ && fabsf(cm - last_distance_cm_) > kTfminiMaxJumpCm) {
    ++jump_reject_count_;
    cm = last_distance_cm_ + ((cm > last_distance_cm_) ? kTfminiMaxJumpCm : -kTfminiMaxJumpCm);
  }

  pushAttempt(range_ok, cm);
  if (!range_ok) {
    timeout_flag_ = true;
    if (timeout_count_ < 0xFFFFu) {
      ++timeout_count_;
    }
    return;
  }

  if (valid_count_ < static_cast<uint8_t>(SONAR_MIN_VALID_IN_WINDOW)) {
    timeout_flag_ = true;
    return;
  }

  const float median_cm = medianHistory();
  if (!ema_initialized_) {
    ema_cm_ = median_cm;
    ema_initialized_ = true;
  } else {
    ema_cm_ = (kSonarEmaAlpha * median_cm) + ((1.0f - kSonarEmaAlpha) * ema_cm_);
  }

  last_distance_cm_ = median_cm;
  last_x_cm_ = runtimeMapBallPosCm(last_distance_cm_);
  last_x_filt_cm_ = runtimeMapBallPosCm(ema_cm_);
  last_sample_ms_ = now_ms;
  has_sample_ = true;
  timeout_flag_ = false;
  if (valid_streak_ < 0xFFFFu) {
    ++valid_streak_;
  }
}

void TFMiniSensor::pushAttempt(bool valid, float sample_cm) {
  const uint8_t idx = history_index_;
  if (history_count_ == kWindow) {
    if (valid_flags_[idx]) {
      --valid_count_;
    }
  } else {
    ++history_count_;
  }

  valid_flags_[idx] = valid ? 1U : 0U;
  if (valid) {
    history_[idx] = sample_cm;
    ++valid_count_;
  }

  history_index_ = static_cast<uint8_t>((idx + 1U) % kWindow);
}

float TFMiniSensor::medianHistory() const {
  float tmp[kWindow];
  uint8_t n = 0;
  for (uint8_t i = 0; i < history_count_; ++i) {
    if (valid_flags_[i]) {
      tmp[n++] = history_[i];
    }
  }
  if (n == 0) {
    return 0.0f;
  }

  for (uint8_t i = 0; i + 1 < n; ++i) {
    for (uint8_t j = 0; j + 1 < n - i; ++j) {
      if (tmp[j] > tmp[j + 1]) {
        const float swap = tmp[j];
        tmp[j] = tmp[j + 1];
        tmp[j + 1] = swap;
      }
    }
  }

  const uint8_t mid = n / 2;
  if ((n % 2U) == 0U) {
    return 0.5f * (tmp[mid - 1] + tmp[mid]);
  }
  return tmp[mid];
}

}  // namespace bb

