#pragma once

#include <Arduino.h>

namespace bb {

enum class AppState : uint8_t {
  SAFE_DISABLED = 0,
  CALIB_SIGN,
  READY,
  RUNNING,
  FAULT
};

struct SensorData {
  // Control-frame actuator estimate from relative step counts.
  float beam_angle_deg = 0.0f;
  float beam_angle_rad = 0.0f;
  // AS5600-relative actuator pose (safety / verification only).
  float as5600_theta_deg = 0.0f;
  float as5600_theta_rad = 0.0f;
  float beam_angle_raw_deg = 0.0f;
  float actuator_abs_deg = 0.0f;
  float actuator_verify_err_deg = 0.0f;

  float ball_pos_linear_cm = 0.0f;
  float ball_pos_linear_m = 0.0f;
  float ball_pos_linear_filt_cm = 0.0f;
  float ball_pos_linear_filt_m = 0.0f;
  float ball_pos_ctrl_cm = 0.0f;
  float ball_pos_ctrl_filt_cm = 0.0f;
  float ball_pos_feedback_cm = 0.0f;
  float ball_pos_feedback_filt_cm = 0.0f;
  float feedback_blend = 0.0f;
  float ball_pos_cm = 0.0f;
  float ball_pos_m = 0.0f;
  float ball_pos_filt_cm = 0.0f;
  float ball_pos_filt_m = 0.0f;

  float sonar_distance_raw_cm = 0.0f;
  uint32_t sonar_age_ms = 0;
  uint16_t sonar_miss_count = 0;

  bool valid_angle = false;
  bool valid_pos = false;
  bool pos_fresh = false;
  bool pos_held = false;
  bool pos_control_usable = false;
  bool control_hold_active = false;
  uint32_t ts_ms = 0;
};

struct Setpoint {
  float ball_pos_cm_target = 0.0f;
  float ball_pos_m_target = 0.0f;
};

struct ActuatorCmd {
  float signed_step_rate_sps = 0.0f;
  bool enable = false;
  bool dir_positive = true;
};

struct FaultFlags {
  bool sonar_timeout = false;
  bool i2c_error = false;
  bool angle_oob = false;
  bool pos_oob = false;
  bool actuator_drift = false;
};

inline uint8_t packFaultFlags(const FaultFlags& flags) {
  return (flags.sonar_timeout ? 0x01 : 0x00) |
         (flags.i2c_error ? 0x02 : 0x00) |
         (flags.angle_oob ? 0x04 : 0x00) |
         (flags.pos_oob ? 0x08 : 0x00) |
         (flags.actuator_drift ? 0x10 : 0x00);
}

}  // namespace bb
