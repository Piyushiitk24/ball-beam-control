#pragma once

#include "control/pid.h"
#include "types.h"

namespace bb {

class CascadeController {
 public:
  CascadeController();

  void reset();
  ActuatorCmd update(const SensorData& sensor, const Setpoint& setpoint, float dt_s);

  float lastThetaCmdDeg() const;

 private:
  PID outer_pos_pid_;
  PID inner_theta_pid_;
  float last_theta_cmd_deg_;
};

}  // namespace bb
