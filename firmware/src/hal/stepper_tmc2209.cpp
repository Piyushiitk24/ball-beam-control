#include "hal/stepper_tmc2209.h"

#include <math.h>

#include <util/atomic.h>

#include "calibration_runtime.h"
#include "config.h"

namespace bb {

StepperTMC2209::StepperTMC2209(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin)
    : step_pin_(step_pin),
      dir_pin_(dir_pin),
      en_pin_(en_pin),
      step_port_out_(nullptr),
      step_mask_(0),
      dir_port_out_(nullptr),
      dir_mask_(0),
      scheduler_started_(false),
      enabled_(false),
      step_level_(false),
      pulse_active_(false),
      direction_positive_(true),
      step_dir_sign_(1),
      signed_rate_sps_(0.0f),
      abs_rate_sps_(0.0f),
      half_period_ticks_(0),
      ticks_until_toggle_(0),
      jog_steps_remaining_(0),
      jog_mode_(false),
      jog_finished_flag_(false),
      position_steps_(0) {}

void StepperTMC2209::begin() {
  pinMode(step_pin_, OUTPUT);
  pinMode(dir_pin_, OUTPUT);
  pinMode(en_pin_, OUTPUT);

  step_port_out_ = portOutputRegister(digitalPinToPort(step_pin_));
  step_mask_ = digitalPinToBitMask(step_pin_);
  dir_port_out_ = portOutputRegister(digitalPinToPort(dir_pin_));
  dir_mask_ = digitalPinToBitMask(dir_pin_);

  setStepPinLevelFast(false);
  setDirPinFast(false);
  digitalWrite(en_pin_, HIGH);  // active-low enable

  enabled_ = false;
}

void StepperTMC2209::beginScheduler() {
  const uint16_t compare = static_cast<uint16_t>((F_CPU / 8UL / kTimerHz) - 1UL);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = compare;
    TCCR1B |= _BV(WGM12);  // CTC
    TCCR1B |= _BV(CS11);   // prescaler 8
    TIMSK1 |= _BV(OCIE1A);
    scheduler_started_ = true;
  }
}

void StepperTMC2209::endScheduler() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TIMSK1 &= static_cast<uint8_t>(~_BV(OCIE1A));
    scheduler_started_ = false;
  }
}

void StepperTMC2209::enable(bool enabled) {
  enabled_ = enabled;
  digitalWrite(en_pin_, enabled ? LOW : HIGH);

  if (!enabled) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      step_level_ = false;
      ticks_until_toggle_ = half_period_ticks_;
    }
    setStepPinLevelFast(false);
  }
}

bool StepperTMC2209::isEnabled() const { return enabled_; }

void StepperTMC2209::stop() {
  signed_rate_sps_ = 0.0f;
  abs_rate_sps_ = 0.0f;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pulse_active_ = false;
    jog_mode_ = false;
    jog_steps_remaining_ = 0;
    jog_finished_flag_ = false;
    half_period_ticks_ = 0;
    ticks_until_toggle_ = 0;
    step_level_ = false;
  }

  setStepPinLevelFast(false);
}

uint16_t StepperTMC2209::rateToHalfPeriodTicks(float abs_rate_sps) const {
  if (abs_rate_sps < 1.0f) {
    return 0;
  }

  float ticks_f = static_cast<float>(kTimerHz) / (2.0f * abs_rate_sps);
  if (ticks_f < static_cast<float>(kMinHalfPeriodTicks)) {
    ticks_f = static_cast<float>(kMinHalfPeriodTicks);
  }
  if (ticks_f > static_cast<float>(kMaxHalfPeriodTicks)) {
    ticks_f = static_cast<float>(kMaxHalfPeriodTicks);
  }

  return static_cast<uint16_t>(ticks_f + 0.5f);
}

void StepperTMC2209::setSignedStepRate(float signed_rate_sps) {
  if (signed_rate_sps > kMaxStepRateSps) {
    signed_rate_sps = kMaxStepRateSps;
  } else if (signed_rate_sps < -kMaxStepRateSps) {
    signed_rate_sps = -kMaxStepRateSps;
  }

  signed_rate_sps_ = signed_rate_sps;
  abs_rate_sps_ = fabsf(signed_rate_sps_);

  const float hardware_dir_reference =
      signed_rate_sps_ * static_cast<float>(runtimeCalStepperDirSign());
  direction_positive_ = (hardware_dir_reference >= 0.0f);
  setDirPinFast(direction_positive_);
  step_dir_sign_ = direction_positive_ ? 1 : -1;

  const uint16_t half_period_ticks = rateToHalfPeriodTicks(abs_rate_sps_);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (half_period_ticks == 0) {
      // Rate too low — stop pulsing.
      half_period_ticks_ = 0;
      ticks_until_toggle_ = 0;
      pulse_active_ = false;
      step_level_ = false;
    } else if (!pulse_active_) {
      // Starting from idle — full reset.
      half_period_ticks_ = half_period_ticks;
      ticks_until_toggle_ = half_period_ticks;
      pulse_active_ = true;
    } else {
      // Already pulsing — update period without resetting countdown so the
      // current half‑cycle completes naturally.  Clamp the remaining count
      // to the new period so a large decrease in rate takes effect quickly.
      half_period_ticks_ = half_period_ticks;
      if (ticks_until_toggle_ > half_period_ticks) {
        ticks_until_toggle_ = half_period_ticks;
      }
    }
  }

  if (half_period_ticks == 0) {
    setStepPinLevelFast(false);
  }
}

float StepperTMC2209::targetSignedStepRate() const { return signed_rate_sps_; }

int32_t StepperTMC2209::positionSteps() const {
  int32_t steps = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    steps = position_steps_;
  }
  return steps;
}

void StepperTMC2209::resetPositionSteps(int32_t steps) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    position_steps_ = steps;
  }
}

void StepperTMC2209::requestJogSteps(long signed_steps, float abs_rate_sps) {
  if (signed_steps == 0) {
    stop();
    return;
  }

  if (abs_rate_sps < 1.0f) {
    abs_rate_sps = 1.0f;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    jog_steps_remaining_ = labs(signed_steps);
    jog_mode_ = true;
    jog_finished_flag_ = false;
  }

  setSignedStepRate((signed_steps > 0) ? abs_rate_sps : -abs_rate_sps);
}

bool StepperTMC2209::jogActive() const {
  bool active = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    active = jog_mode_ && (jog_steps_remaining_ > 0);
  }
  return active;
}

void StepperTMC2209::processIsrFlags() {
  bool finished = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (jog_finished_flag_) {
      jog_finished_flag_ = false;
      jog_mode_ = false;
      jog_steps_remaining_ = 0;
      finished = true;
    }
  }

  if (finished) {
    setSignedStepRate(0.0f);
  }
}

void StepperTMC2209::handleTimerCompareIsr() {
  if (!scheduler_started_ || !enabled_ || !pulse_active_ || half_period_ticks_ == 0) {
    return;
  }

  if (ticks_until_toggle_ > 1) {
    --ticks_until_toggle_;
    return;
  }

  ticks_until_toggle_ = half_period_ticks_;
  step_level_ = !step_level_;
  if (step_level_) {
    *step_port_out_ |= step_mask_;
  } else {
    *step_port_out_ &= static_cast<uint8_t>(~step_mask_);
  }

  // Count only rising edges as actual steps.
  if (step_level_) {
    position_steps_ += step_dir_sign_;
  }
  if (step_level_ && jog_mode_ && jog_steps_remaining_ > 0) {
    --jog_steps_remaining_;
    if (jog_steps_remaining_ == 0) {
      pulse_active_ = false;
      step_level_ = false;
      *step_port_out_ &= static_cast<uint8_t>(~step_mask_);
      jog_finished_flag_ = true;
    }
  }
}

void StepperTMC2209::setStepPinLevelFast(bool high) {
  if (step_port_out_ == nullptr) {
    return;
  }
  if (high) {
    *step_port_out_ |= step_mask_;
  } else {
    *step_port_out_ &= static_cast<uint8_t>(~step_mask_);
  }
}

void StepperTMC2209::setDirPinFast(bool positive) {
  if (dir_port_out_ == nullptr) {
    return;
  }
  if (positive) {
    *dir_port_out_ |= dir_mask_;
  } else {
    *dir_port_out_ &= static_cast<uint8_t>(~dir_mask_);
  }
}

}  // namespace bb
