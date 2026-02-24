#include "control/cascade_controller.h"

#include "config.h"
#include "generated/controller_gains.h"

namespace bb {
namespace {

float clampf(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

}  // namespace

CascadeController::CascadeController() : last_theta_cmd_rad_(0.0f) {
  PIDGains outer_gains;
  outer_gains.kp = generated::kOuterKp;
  outer_gains.ki = generated::kOuterKi;
  outer_gains.kd = generated::kOuterKd;
  outer_gains.i_min = generated::kOuterIMin;
  outer_gains.i_max = generated::kOuterIMax;
  outer_gains.out_min = generated::kOuterOutMin;
  outer_gains.out_max = generated::kOuterOutMax;
  outer_pos_pid_.setGains(outer_gains);

  PIDGains inner_gains;
  inner_gains.kp = generated::kInnerKp;
  inner_gains.ki = generated::kInnerKi;
  inner_gains.kd = generated::kInnerKd;
  inner_gains.i_min = generated::kInnerIMin;
  inner_gains.i_max = generated::kInnerIMax;
  inner_gains.out_min = generated::kInnerOutMin;
  inner_gains.out_max = generated::kInnerOutMax;
  inner_theta_pid_.setGains(inner_gains);
}

void CascadeController::reset() {
  outer_pos_pid_.reset();
  inner_theta_pid_.reset();
  last_theta_cmd_rad_ = 0.0f;
}

ActuatorCmd CascadeController::update(const SensorData& sensor,
                                     const Setpoint& setpoint,
                                     float dt_s,
                                     float theta_cmd_min_rad,
                                     float theta_cmd_max_rad) {
  ActuatorCmd cmd;
  if (!sensor.valid_angle || !sensor.valid_pos) {
    cmd.enable = false;
    cmd.signed_step_rate_sps = 0.0f;
    cmd.dir_positive = true;
    last_theta_cmd_rad_ = 0.0f;
    return cmd;
  }

  const float pos_error_m = setpoint.ball_pos_m_target - sensor.ball_pos_filt_m;

  if (theta_cmd_min_rad > theta_cmd_max_rad) {
    const float swap = theta_cmd_min_rad;
    theta_cmd_min_rad = theta_cmd_max_rad;
    theta_cmd_max_rad = swap;
  }

  float theta_cmd_rad = outer_pos_pid_.update(pos_error_m, dt_s);
  theta_cmd_rad = clampf(theta_cmd_rad, theta_cmd_min_rad, theta_cmd_max_rad);
  last_theta_cmd_rad_ = theta_cmd_rad;

  const float theta_error_rad = theta_cmd_rad - sensor.beam_angle_rad;
  float signed_step_rate = inner_theta_pid_.update(theta_error_rad, dt_s);
  signed_step_rate = clampf(signed_step_rate, -kMaxStepRateSps, kMaxStepRateSps);

  cmd.enable = true;
  cmd.signed_step_rate_sps = signed_step_rate;
  cmd.dir_positive = (signed_step_rate >= 0.0f);
  return cmd;
}

float CascadeController::lastThetaCmdDeg() const {
  return last_theta_cmd_rad_ * kRadToDeg;
}

float CascadeController::lastThetaCmdRad() const { return last_theta_cmd_rad_; }

}  // namespace bb
