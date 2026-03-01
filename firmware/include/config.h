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
constexpr float kMaxStepRateSps = 5000.0f;
constexpr float kThetaCmdLimitDeg = 8.0f;
constexpr float kThetaHardLimitDeg = 15.0f;
constexpr float kBallPosHardLimitCm = 22.0f;

// Stepper angle (reference-style step-count angle source).
#ifndef STEPPER_STEPS_PER_REV
#define STEPPER_STEPS_PER_REV 200
#endif
#ifndef STEPPER_MICROSTEPS
#define STEPPER_MICROSTEPS 16
#endif
#ifndef STEPPER_LIMIT_FULL_STEPS
#define STEPPER_LIMIT_FULL_STEPS 25
#endif

constexpr float kDegToRad = 0.01745329252f;
constexpr float kRadToDeg = 57.295779513f;
constexpr float kCmToM = 0.01f;
constexpr float kMToCm = 100.0f;

constexpr float kThetaCmdLimitRad = kThetaCmdLimitDeg * kDegToRad;
constexpr float kThetaHardLimitRad = kThetaHardLimitDeg * kDegToRad;
constexpr float kBallPosHardLimitM = kBallPosHardLimitCm * kCmToM;

constexpr float kStepperDegPerStep =
    360.0f / (static_cast<float>(STEPPER_STEPS_PER_REV) * static_cast<float>(STEPPER_MICROSTEPS));
constexpr int32_t kStepperPosLimitSteps =
    static_cast<int32_t>(STEPPER_LIMIT_FULL_STEPS) * static_cast<int32_t>(STEPPER_MICROSTEPS);
constexpr int32_t kStepperPosLimitMarginSteps = 8;

constexpr uint32_t kSensorInvalidFaultMsBringup = 1000;
constexpr uint32_t kSensorInvalidFaultMsRunning = 300;

// Sonar tuning (NewPing-style: rolling median + hold + EMA).
//
// Note: Unlike NewPing's ping_median(N) (a short burst), this firmware uses a
// rolling window across time, so defaults are kept modest to reduce lag.
#ifndef SONAR_TRIGGER_PERIOD_US
#define SONAR_TRIGGER_PERIOD_US 40000UL
#endif
#ifndef SONAR_ECHO_TIMEOUT_US
#define SONAR_ECHO_TIMEOUT_US 25000UL
#endif
#ifndef SONAR_POS_SAMPLE_FRESH_MS
#define SONAR_POS_SAMPLE_FRESH_MS 200UL
#endif
#ifndef SONAR_EMA_ALPHA
#define SONAR_EMA_ALPHA 0.6f
#endif
#ifndef SONAR_MEDIAN_WINDOW
#define SONAR_MEDIAN_WINDOW 11
#endif
#ifndef SONAR_MIN_VALID_IN_WINDOW
#define SONAR_MIN_VALID_IN_WINDOW 1
#endif
#ifndef SONAR_MAX_VALID_MM
#define SONAR_MAX_VALID_MM 650.0f
#endif

// Benewake TFMini (UART) tuning.
#ifndef TFMINI_UART_BAUD
#define TFMINI_UART_BAUD 115200UL
#endif
#ifndef TFMINI_READ_STALE_MS
#define TFMINI_READ_STALE_MS SONAR_POS_SAMPLE_FRESH_MS
#endif

// AS5600 tuning (noise rejection + calibration stability).
#ifndef AS5600_EMA_ALPHA
#define AS5600_EMA_ALPHA 0.3f
#endif
#ifndef AS5600_MAX_JUMP_DEG
#define AS5600_MAX_JUMP_DEG 25.0f
#endif
#ifndef AS5600_CAL_NEED_GOOD
#define AS5600_CAL_NEED_GOOD 5
#endif
#ifndef AS5600_CAL_TIMEOUT_MS
#define AS5600_CAL_TIMEOUT_MS 900UL
#endif
#ifndef AS5600_CAL_MAX_JUMP_DEG
#define AS5600_CAL_MAX_JUMP_DEG 10.0f
#endif

static_assert(SONAR_MIN_VALID_IN_WINDOW <= SONAR_MEDIAN_WINDOW,
              "SONAR_MIN_VALID_IN_WINDOW must be <= SONAR_MEDIAN_WINDOW");

constexpr uint32_t kSonarTriggerPeriodUs = SONAR_TRIGGER_PERIOD_US;
constexpr uint32_t kSonarEchoTimeoutUs = SONAR_ECHO_TIMEOUT_US;
constexpr uint32_t kPosSampleFreshMs = SONAR_POS_SAMPLE_FRESH_MS;
constexpr float kSonarEmaAlpha = SONAR_EMA_ALPHA;
constexpr uint32_t kTfminiUartBaud = TFMINI_UART_BAUD;
constexpr uint32_t kTfminiReadStaleMs = TFMINI_READ_STALE_MS;

constexpr float kAs5600EmaAlpha = AS5600_EMA_ALPHA;
constexpr float kAs5600MaxJumpDeg = AS5600_MAX_JUMP_DEG;

constexpr float kDefaultBallSetpointCm = 0.0f;

}  // namespace bb
