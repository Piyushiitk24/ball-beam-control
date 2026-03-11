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
      has_prev_measurement_(false),
      last_target_steps_(0.0f),
      last_target_steps_unclamped_(0.0f),
      last_target_saturated_(false),
      last_signed_rate_sps_(0.0f) {}

void CascadeController::reset() {
  integral_output_steps_ = 0.0f;
  prev_measurement_cm_ = 0.0f;
  has_prev_measurement_ = false;
  last_target_steps_ = 0.0f;
  last_target_steps_unclamped_ = 0.0f;
  last_target_saturated_ = false;
  last_signed_rate_sps_ = 0.0f;
}

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

  const float pos_error_cm = setpoint.ball_pos_cm_target - sensor.ball_pos_filt_cm;
  float measurement_rate_cm_s = 0.0f;
  if (has_prev_measurement_) {
    measurement_rate_cm_s = (sensor.ball_pos_filt_cm - prev_measurement_cm_) / dt_s;
  }

  const float p_term_steps = kPosPidKpStepsPerCm * pos_error_cm;
  const float d_term_steps = -kPosPidKdStepsSecPerCm * measurement_rate_cm_s;
  const float pre_i_output = p_term_steps + integral_output_steps_ + d_term_steps;
  const bool sat_high = (pre_i_output >= static_cast<float>(pos_max_steps));
  const bool sat_low = (pre_i_output <= static_cast<float>(pos_min_steps));
  if (!((sat_high && pos_error_cm > 0.0f) || (sat_low && pos_error_cm < 0.0f))) {
    integral_output_steps_ += kPosPidKiStepsPerCmSec * pos_error_cm * dt_s;
    integral_output_steps_ =
        clampf(integral_output_steps_, -kPosPidIntegralClampSteps, kPosPidIntegralClampSteps);
  }

  last_target_steps_unclamped_ = p_term_steps + integral_output_steps_ + d_term_steps;
  const float target_steps = clampf(last_target_steps_unclamped_,
                                    static_cast<float>(pos_min_steps),
                                    static_cast<float>(pos_max_steps));
  last_target_steps_ = target_steps;
  last_target_saturated_ = fabsf(last_target_steps_unclamped_ - last_target_steps_) > 1.0e-4f;
  prev_measurement_cm_ = sensor.ball_pos_filt_cm;
  has_prev_measurement_ = true;

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
