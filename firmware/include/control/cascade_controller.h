#pragma once

#include "types.h"

namespace bb {

class CascadeController {
 public:
  CascadeController();

  void reset();
  void clearAdaptiveCenterBias();
  ActuatorCmd update(const SensorData& sensor,
                     const Setpoint& setpoint,
                     float dt_s,
                     int32_t pos_min_steps,
                     int32_t pos_max_steps);

  float lastThetaCmdDeg() const;
  float lastThetaCmdUnclampedDeg() const;
  bool lastThetaCmdSaturated() const;

 private:
  float integral_output_steps_;
  float prev_measurement_cm_;
  uint32_t prev_measurement_ts_ms_;
  bool has_prev_measurement_;
  float prev_linear_measurement_cm_;
  uint32_t prev_linear_measurement_ts_ms_;
  bool has_prev_linear_measurement_;
  float last_target_steps_;
  float last_target_steps_unclamped_;
  bool last_target_saturated_;
  float last_signed_rate_sps_;
  float center_bias_steps_;
};

}  // namespace bb
