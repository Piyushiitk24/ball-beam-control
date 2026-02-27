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

constexpr float kDegToRad = 0.01745329252f;
constexpr float kRadToDeg = 57.295779513f;
constexpr float kCmToM = 0.01f;
constexpr float kMToCm = 100.0f;

constexpr float kThetaCmdLimitRad = kThetaCmdLimitDeg * kDegToRad;
constexpr float kThetaHardLimitRad = kThetaHardLimitDeg * kDegToRad;
constexpr float kBallPosHardLimitM = kBallPosHardLimitCm * kCmToM;

constexpr uint32_t kSensorInvalidFaultMsBringup = 1000;
constexpr uint32_t kSensorInvalidFaultMsRunning = 300;

// Sonar timing:
// - Use a conservative trigger period to reduce ghost echoes on cheap HC-SR04 clones.
// - Keep echo waits bounded (no pulseIn()); 30ms corresponds to ~5m round-trip max.
constexpr uint32_t kSonarEchoTimeoutUs = 30000;
constexpr uint32_t kSonarTriggerPeriodUs = 60000;
constexpr uint32_t kPosSampleFreshMs = 120;
constexpr float kSonarEmaAlpha = 0.35f;

constexpr float kDefaultBallSetpointCm = 0.0f;

}  // namespace bb
