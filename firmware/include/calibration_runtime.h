#pragma once

#include <Arduino.h>

namespace bb {

struct RuntimeCalibration {
  uint16_t magic = 0;
  uint16_t version = 0;
  int8_t as5600_sign = 1;
  int8_t stepper_dir_sign = 1;
  int8_t sonar_pos_sign = 1;
  uint8_t flags = 0;
  float as5600_zero_deg = 0.0f;
  float sonar_center_cm = 0.0f;
  float theta_lower_limit_deg = -8.0f;
  float theta_upper_limit_deg = 8.0f;
  uint16_t crc16 = 0;
};

enum class RuntimeCalLoadStatus : uint8_t {
  kUninitialized = 0,
  kLoadedFromEeprom,
  kDefaultsApplied
};

void runtimeCalInit();
bool runtimeCalLoad();
bool runtimeCalSave();
void runtimeCalResetDefaults();

const RuntimeCalibration& runtimeCalData();
RuntimeCalLoadStatus runtimeCalLoadStatus();
const char* runtimeCalLoadStatusName(RuntimeCalLoadStatus status);

bool runtimeCalIsZeroSet();
bool runtimeCalIsZeroAngleCaptured();
bool runtimeCalIsZeroPosCaptured();
bool runtimeCalIsLimitsSet();
bool runtimeCalIsLowerLimitCaptured();
bool runtimeCalIsUpperLimitCaptured();
bool runtimeCalIsSignSet();
bool runtimeCalHasValidLimitSpan();

void runtimeCalMarkZeroAngleCaptured(bool enabled);
void runtimeCalMarkZeroPosCaptured(bool enabled);
void runtimeCalMarkLowerLimitCaptured(bool enabled);
void runtimeCalMarkUpperLimitCaptured(bool enabled);
void runtimeCalSetSignCaptured(bool enabled);

int8_t runtimeCalAs5600Sign();
int8_t runtimeCalStepperDirSign();
int8_t runtimeCalSonarPosSign();
void runtimeCalSetAs5600Sign(int8_t sign);
void runtimeCalSetStepperDirSign(int8_t sign);
void runtimeCalSetSonarPosSign(int8_t sign);

float runtimeCalAs5600ZeroDeg();
float runtimeCalSonarCenterCm();
void runtimeCalSetAs5600ZeroDeg(float deg);
void runtimeCalSetSonarCenterCm(float cm);

float runtimeCalThetaLowerLimitDeg();
float runtimeCalThetaUpperLimitDeg();
void runtimeCalSetThetaLowerLimitDeg(float deg);
void runtimeCalSetThetaUpperLimitDeg(float deg);

float runtimeMapThetaDeg(float raw_angle_deg);
float runtimeMapBallPosCm(float sonar_distance_cm);

}  // namespace bb
