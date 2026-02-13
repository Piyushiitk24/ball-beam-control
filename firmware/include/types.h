#pragma once

#include <Arduino.h>

namespace bb {

enum class AppState : uint8_t {
  SAFE_DISABLED = 0,
  CALIB_SIGN,
  CALIB_SCALE,
  READY,
  RUNNING,
  FAULT
};

struct SensorData {
  float beam_angle_deg = 0.0f;
  float beam_angle_raw_deg = 0.0f;
  float ball_pos_cm = 0.0f;
  float ball_pos_filt_cm = 0.0f;
  float sonar_distance_raw_cm = 0.0f;
  bool valid_angle = false;
  bool valid_pos = false;
  uint32_t ts_ms = 0;
};

struct Setpoint {
  float ball_pos_cm_target = 0.0f;
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
};

inline uint8_t packFaultFlags(const FaultFlags& flags) {
  return (flags.sonar_timeout ? 0x01 : 0x00) |
         (flags.i2c_error ? 0x02 : 0x00) |
         (flags.angle_oob ? 0x04 : 0x00) |
         (flags.pos_oob ? 0x08 : 0x00);
}

}  // namespace bb
