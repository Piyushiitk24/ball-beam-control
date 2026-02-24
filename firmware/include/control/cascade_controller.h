#pragma once

#include "control/pid.h"
#include "types.h"

namespace bb {

class CascadeController {
 public:
  CascadeController();

  void reset();
  ActuatorCmd update(const SensorData& sensor,
                     const Setpoint& setpoint,
                     float dt_s,
                     float theta_cmd_min_rad,
                     float theta_cmd_max_rad);

  float lastThetaCmdDeg() const;
  float lastThetaCmdRad() const;

 private:
  PID outer_pos_pid_;
  PID inner_theta_pid_;
  float last_theta_cmd_rad_;
};

}  // namespace bb
