#pragma once

#include <Arduino.h>

namespace bb {

/// Diagnostic snapshot for any ball-position sensor.
/// Shared by HC-SR04, TFMini, and Sharp IR backends.
struct SonarDiag {
  bool has_sample = false;
  bool timeout = false;
  bool fresh = false;
  uint32_t age_ms = 0;
  float raw_cm = 0.0f;
  float filt_cm = 0.0f;
  uint16_t valid_streak = 0;
  uint16_t timeout_count = 0;
  uint16_t jump_reject_count = 0;
};

}  // namespace bb
