#include "sensors/sharp_ir_sensor.h"

#include <math.h>

#include "calibration_runtime.h"

namespace bb {
SharpIRSensor::SharpIRSensor(uint8_t analog_pin)
    : pin_(analog_pin),
      has_sample_(false),
      last_distance_cm_(0.0f),
      last_x_cm_(0.0f),
      last_x_filt_cm_(0.0f),
      last_sample_ms_(0),
      last_read_ms_(0),
      ema_initialized_(false),
      ema_cm_(0.0f),
      valid_streak_(0),
      consecutive_miss_count_(0),
      timeout_count_(0),
      jump_reject_count_(0),
      timeout_flag_(false) {}

void SharpIRSensor::begin() {
  pinMode(pin_, INPUT);
  last_read_ms_ = millis();
  last_sample_ms_ = last_read_ms_;
}

float SharpIRSensor::adcToDistanceCm(uint16_t adc_raw) {
  // GP2Y0A21YK0F empirical power-law fit:
  //   distance_cm ≈ 29.988 * voltage^(-1.173)
  // where voltage = adc_raw * (5.0 / 1024.0)
  //
  // Guard against division by zero / nonsense at very low ADC.
  if (adc_raw < 10) {
    return 0.0f;  // invalid — too far or disconnected
  }
  const float voltage = static_cast<float>(adc_raw) * (5.0f / 1024.0f);
  if (voltage < 0.05f) {
    return 0.0f;
  }
  return 29.988f * powf(voltage, -1.173f);
}

void SharpIRSensor::service(uint32_t now_us, uint32_t now_ms) {
  (void)now_us;

  // Rate-limit reads to match sensor internal cycle.
  if (static_cast<uint32_t>(now_ms - last_read_ms_) < kSharpIrSamplePeriodMs) {
    timeout_flag_ = has_sample_ && !hasFreshSample(now_ms);
    return;
  }
  last_read_ms_ = now_ms;

  const uint16_t adc_raw = static_cast<uint16_t>(analogRead(pin_));
  const float raw_cm = adcToDistanceCm(adc_raw);

  const bool range_ok = (raw_cm >= kSharpIrMinValidCm) && (raw_cm <= kSharpIrMaxValidCm);

  if (!range_ok) {
    if (consecutive_miss_count_ < 0xFFFFu) {
      ++consecutive_miss_count_;
    }
    if (timeout_count_ < 0xFFFFu) {
      ++timeout_count_;
    }
    timeout_flag_ = !hasFreshSample(now_ms);
    return;
  }

  // Jump rejection.
  float cm = raw_cm;
  if (has_sample_ && fabsf(cm - last_distance_cm_) > kSharpIrMaxJumpCm) {
    cm = last_distance_cm_ + ((cm > last_distance_cm_) ? kSharpIrMaxJumpCm : -kSharpIrMaxJumpCm);
    if (jump_reject_count_ < 0xFFFFu) {
      ++jump_reject_count_;
    }
  }

  // EMA filter.
  if (!ema_initialized_) {
    ema_cm_ = cm;
    ema_initialized_ = true;
  } else {
    ema_cm_ = (kSonarEmaAlpha * cm) + ((1.0f - kSonarEmaAlpha) * ema_cm_);
  }

  last_distance_cm_ = cm;
  last_x_cm_ = runtimeMapBallPosCm(last_distance_cm_);
  last_x_filt_cm_ = runtimeMapBallPosCm(ema_cm_);
  last_sample_ms_ = now_ms;
  has_sample_ = true;
  timeout_flag_ = false;
  consecutive_miss_count_ = 0;
  if (valid_streak_ < 0xFFFFu) {
    ++valid_streak_;
  }
}

bool SharpIRSensor::hasSample() const { return has_sample_; }

bool SharpIRSensor::getPosition(float& x_cm, float& x_filt_cm,
                                float& distance_raw_cm) const {
  if (!has_sample_) {
    return false;
  }
  x_cm = last_x_cm_;
  x_filt_cm = last_x_filt_cm_;
  distance_raw_cm = last_distance_cm_;
  return true;
}

bool SharpIRSensor::hasFreshSample(uint32_t now_ms) const {
  if (!has_sample_) {
    return false;
  }
  return sampleAgeMs(now_ms) <= kPosSampleFreshMs;
}

uint32_t SharpIRSensor::sampleAgeMs(uint32_t now_ms) const {
  if (!has_sample_) {
    return 0xFFFFFFFFUL;
  }
  return static_cast<uint32_t>(now_ms - last_sample_ms_);
}

bool SharpIRSensor::hasTimeout() const { return timeout_flag_; }

uint16_t SharpIRSensor::consecutiveMissCount() const { return consecutive_miss_count_; }

void SharpIRSensor::getDiag(uint32_t now_ms, SonarDiag& diag) const {
  diag.has_sample = has_sample_;
  diag.timeout = timeout_flag_ || (has_sample_ && !hasFreshSample(now_ms));
  diag.age_ms = sampleAgeMs(now_ms);
  diag.fresh = hasFreshSample(now_ms);
  diag.raw_cm = last_distance_cm_;
  diag.filt_cm = ema_cm_;
  diag.valid_streak = valid_streak_;
  diag.consecutive_miss_count = consecutive_miss_count_;
  diag.timeout_count = timeout_count_;
  diag.jump_reject_count = jump_reject_count_;
}

}  // namespace bb
