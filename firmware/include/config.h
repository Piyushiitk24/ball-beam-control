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
constexpr float kMaxStepRateSps = 2000.0f;
constexpr float kMaxStepRateChangeSpsPerTick = 120.0f;  // slew limit per 50 Hz control tick
constexpr float kBallPosHardLimitCm = 22.0f;
constexpr float kRunMotionAccelSps2 = kMaxStepRateChangeSpsPerTick / kControlDtSec;
constexpr float kRunPositionDeadbandSteps = 2.0f;
constexpr float kAs5600VerifyMaxErrDeg = 8.0f;
constexpr uint32_t kAs5600VerifyFaultMs = 250;

// Single position PID, modeled on PID_v1 behavior: derivative on measured
// position, not derivative on setpoint error. Gains are in physical-beam
// microstep units after applying the AS5600 motor-angle -> beam-angle map.
constexpr float kPosPidKpStepsPerCm = 25.0f;
constexpr float kPosPidKiStepsPerCmSec = 0.8f;
constexpr float kPosPidKdStepsSecPerCm = 10.0f;
constexpr float kCenterPidKpScale = 0.6f;
constexpr float kCenterPidKiScale = 0.5f;
constexpr float kCenterPidKdScale = 0.25f;
constexpr float kCenterSoftPosWindowCm = 3.0f;
constexpr float kCenterSoftRateTolCmS = 6.0f;
constexpr float kCenterHoldPosTolCm = 0.7f;
constexpr float kCenterHoldRateTolCmS = 2.0f;
constexpr float kCenterIntegralBleedPerSec = 2.0f;
constexpr float kCenterBiasLearnStepsPerCmSec = 0.25f;
constexpr float kCenterBiasLearnPosWindowCm = 6.0f;
constexpr float kCenterBiasLearnRateTolCmS = 10.0f;
constexpr float kCenterBiasClampSteps = 160.0f;
constexpr float kPositiveSideKpScale = 1.0f;
constexpr float kPositiveSideKiScale = 1.0f;

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

constexpr float kBallPosHardLimitM = kBallPosHardLimitCm * kCmToM;

constexpr float kStepperDegPerStep =
    360.0f / (static_cast<float>(STEPPER_STEPS_PER_REV) * static_cast<float>(STEPPER_MICROSTEPS));
constexpr float kBeamDegPerActuatorDeg = 0.0767406f;
constexpr float kBeamLevelFracFromLower = 0.79618f;
constexpr float kBeamDegPerStep = kStepperDegPerStep * kBeamDegPerActuatorDeg;
constexpr int32_t kStepperPosLimitSteps =
    static_cast<int32_t>(STEPPER_LIMIT_FULL_STEPS) * static_cast<int32_t>(STEPPER_MICROSTEPS);
constexpr int32_t kStepperPosLimitMarginSteps = 8;
constexpr float kPosPidIntegralClampSteps = 120.0f;

constexpr uint32_t kSensorInvalidFaultMsBringup = 1000;
constexpr uint32_t kSensorInvalidFaultMsRunning = 2500;

// Active HC-SR04 freshness / timeout settings. Telemetry and host tooling keep
// the long-standing `sonar_*` naming, even when other experimental sensors are
// present in the repo.
#ifndef SONAR_TRIGGER_PERIOD_US
#define SONAR_TRIGGER_PERIOD_US 40000UL
#endif
#ifndef SONAR_ECHO_TIMEOUT_US
#define SONAR_ECHO_TIMEOUT_US 25000UL
#endif
#ifndef SONAR_POS_SAMPLE_FRESH_MS
#define SONAR_POS_SAMPLE_FRESH_MS 500UL
#endif
#ifndef SONAR_EMA_ALPHA
#define SONAR_EMA_ALPHA 0.3f
#endif
#ifndef SONAR_MEDIAN_WINDOW
#define SONAR_MEDIAN_WINDOW 11
#endif
#ifndef SONAR_MIN_VALID_IN_WINDOW
#define SONAR_MIN_VALID_IN_WINDOW 3
#endif
#ifndef SONAR_MAX_VALID_MM
#define SONAR_MAX_VALID_MM 650.0f
#endif
#ifndef SONAR_MAX_JUMP_CM
#define SONAR_MAX_JUMP_CM 3.0f
#endif
#ifndef SONAR_MAX_CONSECUTIVE_MISSES
#define SONAR_MAX_CONSECUTIVE_MISSES 6
#endif

// Archived / experimental Sharp GP2Y0A21YK0F analog IR defaults retained for
// standalone characterization and future fallback work.
#ifndef SHARP_IR_SAMPLE_PERIOD_MS
#define SHARP_IR_SAMPLE_PERIOD_MS 40UL
#endif
#ifndef SHARP_IR_MIN_VALID_CM
#define SHARP_IR_MIN_VALID_CM 10.0f
#endif
#ifndef SHARP_IR_MAX_VALID_CM
#define SHARP_IR_MAX_VALID_CM 80.0f
#endif
#ifndef SHARP_IR_MAX_JUMP_CM
#define SHARP_IR_MAX_JUMP_CM 25.0f
#endif

// Legacy: Benewake TFMini (UART) — not connected.
// #define TFMINI_UART_BAUD 115200UL
// #define TFMINI_READ_STALE_MS SONAR_POS_SAMPLE_FRESH_MS

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
constexpr float kSonarMaxJumpCm = SONAR_MAX_JUMP_CM;
constexpr uint8_t kSonarMaxConsecutiveMisses = SONAR_MAX_CONSECUTIVE_MISSES;
constexpr uint32_t kSharpIrSamplePeriodMs = SHARP_IR_SAMPLE_PERIOD_MS;
constexpr float kSharpIrMinValidCm = SHARP_IR_MIN_VALID_CM;
constexpr float kSharpIrMaxValidCm = SHARP_IR_MAX_VALID_CM;
constexpr float kSharpIrMaxJumpCm = SHARP_IR_MAX_JUMP_CM;

constexpr float kAs5600EmaAlpha = AS5600_EMA_ALPHA;
constexpr float kAs5600MaxJumpDeg = AS5600_MAX_JUMP_DEG;

constexpr float kDefaultBallSetpointCm = 0.0f;

}  // namespace bb
