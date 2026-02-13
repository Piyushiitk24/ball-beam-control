#pragma once

#include <Arduino.h>

namespace bb {

// Compile-time calibration placeholders. Update after sign/zero calibration.
constexpr int8_t AS5600_SIGN = +1;
constexpr int8_t STEPPER_DIR_SIGN = +1;
constexpr int8_t SONAR_POS_SIGN = +1;

constexpr float AS5600_ZERO_DEG = 0.0f;
constexpr float SONAR_CENTER_CM = 20.0f;

constexpr long SIGN_CAL_JOG_STEPS = 120;
constexpr float SIGN_CAL_JOG_RATE_SPS = 500.0f;
constexpr float SIGN_CAL_MIN_DELTA_DEG = 0.25f;

inline float wrapAngleDeltaDeg(float delta_deg) {
  while (delta_deg > 180.0f) {
    delta_deg -= 360.0f;
  }
  while (delta_deg < -180.0f) {
    delta_deg += 360.0f;
  }
  return delta_deg;
}

inline float mapThetaDeg(float raw_angle_deg) {
  return static_cast<float>(AS5600_SIGN) * (raw_angle_deg - AS5600_ZERO_DEG);
}

inline float mapBallPosCm(float sonar_distance_cm) {
  return static_cast<float>(SONAR_POS_SIGN) * (sonar_distance_cm - SONAR_CENTER_CM);
}

}  // namespace bb
