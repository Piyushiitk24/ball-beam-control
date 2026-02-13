#pragma once

#include <Arduino.h>

namespace bb {

#ifndef CONTROL_HZ
#define CONTROL_HZ 50
#endif

#ifndef TELEMETRY_HZ
#define TELEMETRY_HZ 10
#endif

constexpr uint32_t kControlHz = CONTROL_HZ;
constexpr uint32_t kTelemetryHz = TELEMETRY_HZ;
constexpr uint32_t kControlPeriodMs = 1000UL / kControlHz;
constexpr uint32_t kTelemetryPeriodMs = 1000UL / kTelemetryHz;

constexpr float kControlDtSec = 1.0f / static_cast<float>(kControlHz);
constexpr float kMaxStepRateSps = 1800.0f;
constexpr float kThetaCmdLimitDeg = 8.0f;
constexpr float kThetaHardLimitDeg = 15.0f;
constexpr float kBallPosHardLimitCm = 22.0f;

constexpr uint32_t kSensorInvalidFaultMs = 200;
constexpr uint32_t kSonarEchoTimeoutUs = 25000;
constexpr float kSonarEmaAlpha = 0.35f;

constexpr float kDefaultBallSetpointCm = 0.0f;

}  // namespace bb
