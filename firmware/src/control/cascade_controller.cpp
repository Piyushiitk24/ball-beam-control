#include "control/cascade_controller.h"

#include <math.h>

#include "config.h"

namespace bb {
namespace {

float clampf(float value, float lo, float hi) {
  if (value < lo) {
    return lo;
  }
  if (value > hi) {
    return hi;
  }
  return value;
}

}  // namespace

CascadeController::CascadeController()
    : integral_output_steps_(0.0f),
      prev_measurement_cm_(0.0f),
      prev_measurement_ts_ms_(0),
      has_prev_measurement_(false),
      prev_linear_measurement_cm_(0.0f),
      prev_linear_measurement_ts_ms_(0),
      has_prev_linear_measurement_(false),
      last_target_steps_(0.0f),
      last_target_steps_unclamped_(0.0f),
      last_target_saturated_(false),
      last_signed_rate_sps_(0.0f),
      center_bias_steps_(0.0f) {}

void CascadeController::reset() {
  integral_output_steps_ = 0.0f;
  prev_measurement_cm_ = 0.0f;
  prev_measurement_ts_ms_ = 0;
  has_prev_measurement_ = false;
  prev_linear_measurement_cm_ = 0.0f;
  prev_linear_measurement_ts_ms_ = 0;
  has_prev_linear_measurement_ = false;
  last_target_steps_ = 0.0f;
  last_target_steps_unclamped_ = 0.0f;
  last_target_saturated_ = false;
  last_signed_rate_sps_ = 0.0f;
}

void CascadeController::clearAdaptiveCenterBias() { center_bias_steps_ = 0.0f; }

ActuatorCmd CascadeController::update(const SensorData& sensor,
                                     const Setpoint& setpoint,
                                     float dt_s,
                                     int32_t pos_min_steps,
                                     int32_t pos_max_steps) {
  ActuatorCmd cmd;
  if (!sensor.valid_pos) {
    cmd.enable = false;
    cmd.signed_step_rate_sps = 0.0f;
    cmd.dir_positive = true;
    last_signed_rate_sps_ = 0.0f;
    has_prev_measurement_ = false;
    return cmd;
  }

  if (pos_min_steps > pos_max_steps) {
    const int32_t swap = pos_min_steps;
    pos_min_steps = pos_max_steps;
    pos_max_steps = swap;
  }

  const bool center_mode = fabsf(setpoint.ball_pos_cm_target) <= 1.0e-4f;
  const uint32_t measurement_ts_ms = sensor.ts_ms - sensor.sonar_age_ms;
  float linear_measurement_rate_cm_s = 0.0f;
  const bool has_new_linear_measurement =
      !has_prev_linear_measurement_ || (measurement_ts_ms != prev_linear_measurement_ts_ms_);
  if (has_prev_linear_measurement_ && has_new_linear_measurement) {
    const uint32_t delta_ms = measurement_ts_ms - prev_linear_measurement_ts_ms_;
    if (delta_ms > 0U) {
      linear_measurement_rate_cm_s =
          (sensor.ball_pos_linear_filt_cm - prev_linear_measurement_cm_) /
          (0.001f * static_cast<float>(delta_ms));
    }
  }
  // Keep the softened center-only behavior only near the calibrated center.
  // Large return-to-center moves use the normal stronger PID path so q c can
  // recover from the runner ends instead of behaving like a tiny hold loop.
  const bool center_soft_mode =
      center_mode &&
      (fabsf(sensor.ball_pos_linear_filt_cm) <= kCenterSoftPosWindowCm) &&
      (fabsf(linear_measurement_rate_cm_s) <= kCenterSoftRateTolCmS);
  const float measurement_cm = center_soft_mode ? sensor.ball_pos_linear_filt_cm
                                                : sensor.ball_pos_filt_cm;
  float pos_error_cm = center_mode ? -measurement_cm
                                   : (setpoint.ball_pos_cm_target - sensor.ball_pos_filt_cm);
  const bool positive_side_mode =
      (!center_mode && (setpoint.ball_pos_cm_target > 0.0f)) ||
      (center_mode && !center_soft_mode && (pos_error_cm > 0.0f));
  float measurement_rate_cm_s = 0.0f;
  const bool has_new_measurement =
      !has_prev_measurement_ || (measurement_ts_ms != prev_measurement_ts_ms_);
  if (has_prev_measurement_ && has_new_measurement) {
    const uint32_t delta_ms = measurement_ts_ms - prev_measurement_ts_ms_;
    if (delta_ms > 0U) {
      measurement_rate_cm_s =
          (measurement_cm - prev_measurement_cm_) / (0.001f * static_cast<float>(delta_ms));
    }
  }

  const bool center_hold =
      center_soft_mode &&
      (fabsf(sensor.ball_pos_linear_filt_cm) <= kCenterHoldPosTolCm) &&
      (fabsf(measurement_rate_cm_s) <= kCenterHoldRateTolCmS);
  if (center_hold) {
    pos_error_cm = 0.0f;
    measurement_rate_cm_s = 0.0f;
    const float bleed = 1.0f - (kCenterIntegralBleedPerSec * dt_s);
    integral_output_steps_ *= (bleed > 0.0f) ? bleed : 0.0f;
  }

  const float kp_scale = center_soft_mode ? kCenterPidKpScale : 1.0f;
  const float ki_scale = center_soft_mode ? kCenterPidKiScale : 1.0f;
  const float kd_scale = center_soft_mode ? kCenterPidKdScale : 1.0f;
  float kp_steps_per_cm = kPosPidKpStepsPerCm * kp_scale;
  float ki_steps_per_cm_s = kPosPidKiStepsPerCmSec * ki_scale;
  if (positive_side_mode) {
    kp_steps_per_cm *= kPositiveSideKpScale;
    ki_steps_per_cm_s *= kPositiveSideKiScale;
  }
  if (center_soft_mode &&
      fabsf(measurement_cm) <= kCenterBiasLearnPosWindowCm &&
      fabsf(measurement_rate_cm_s) <= kCenterBiasLearnRateTolCmS) {
    center_bias_steps_ += kCenterBiasLearnStepsPerCmSec * pos_error_cm * dt_s;
    center_bias_steps_ =
        clampf(center_bias_steps_, -kCenterBiasClampSteps, kCenterBiasClampSteps);
  }

  const float p_term_steps = kp_steps_per_cm * pos_error_cm;
  const float d_term_steps = -(kPosPidKdStepsSecPerCm * kd_scale) * measurement_rate_cm_s;
  const float center_bias_steps = center_soft_mode ? center_bias_steps_ : 0.0f;
  const float pre_i_output = center_bias_steps + p_term_steps + integral_output_steps_ + d_term_steps;
  const bool sat_high = (pre_i_output >= static_cast<float>(pos_max_steps));
  const bool sat_low = (pre_i_output <= static_cast<float>(pos_min_steps));
  if (!((sat_high && pos_error_cm > 0.0f) || (sat_low && pos_error_cm < 0.0f))) {
    integral_output_steps_ += ki_steps_per_cm_s * pos_error_cm * dt_s;
    integral_output_steps_ =
        clampf(integral_output_steps_, -kPosPidIntegralClampSteps, kPosPidIntegralClampSteps);
  }

  last_target_steps_unclamped_ = center_bias_steps + p_term_steps + integral_output_steps_ + d_term_steps;
  const float target_steps = clampf(last_target_steps_unclamped_,
                                    static_cast<float>(pos_min_steps),
                                    static_cast<float>(pos_max_steps));
  last_target_steps_ = target_steps;
  last_target_saturated_ = fabsf(last_target_steps_unclamped_ - last_target_steps_) > 1.0e-4f;
  if (has_new_measurement) {
    prev_measurement_cm_ = measurement_cm;
    prev_measurement_ts_ms_ = measurement_ts_ms;
    has_prev_measurement_ = true;
  }
  if (has_new_linear_measurement) {
    prev_linear_measurement_cm_ = sensor.ball_pos_linear_filt_cm;
    prev_linear_measurement_ts_ms_ = measurement_ts_ms;
    has_prev_linear_measurement_ = true;
  }

  const float current_steps = sensor.beam_angle_deg / kStepperDegPerStep;
  const float step_error = target_steps - current_steps;

  float desired_rate_sps = 0.0f;
  if (fabsf(step_error) > kRunPositionDeadbandSteps) {
    const float mag = sqrtf(2.0f * kRunMotionAccelSps2 * fabsf(step_error));
    desired_rate_sps = (step_error >= 0.0f ? 1.0f : -1.0f) * clampf(mag, 0.0f, kMaxStepRateSps);
  }

  float delta = desired_rate_sps - last_signed_rate_sps_;
  const float max_delta = kRunMotionAccelSps2 * dt_s;
  if (delta > max_delta) {
    delta = max_delta;
  } else if (delta < -max_delta) {
    delta = -max_delta;
  }
  last_signed_rate_sps_ += delta;

  cmd.enable = true;
  cmd.signed_step_rate_sps = last_signed_rate_sps_;
  cmd.dir_positive = (cmd.signed_step_rate_sps >= 0.0f);
  return cmd;
}

float CascadeController::lastThetaCmdDeg() const {
  return last_target_steps_ * kStepperDegPerStep;
}

float CascadeController::lastThetaCmdUnclampedDeg() const {
  return last_target_steps_unclamped_ * kStepperDegPerStep;
}

bool CascadeController::lastThetaCmdSaturated() const { return last_target_saturated_; }

}  // namespace bb
