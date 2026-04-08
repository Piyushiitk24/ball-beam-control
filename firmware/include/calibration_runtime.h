#pragma once

#include <Arduino.h>

namespace bb {

enum class SonarSignMode : uint8_t {
  kOrientation = 0,
  kManual = 1
};

enum class CenterSource : uint8_t {
  kAuto = 0,
  kManual = 1
};

enum class ActuatorTrimSource : uint8_t {
  kSearched = 0,
  kSaved = 1
};

struct RuntimeCalibration {
  uint16_t magic = 0;
  uint16_t version = 0;
  int8_t as5600_sign = 1;
  int8_t stepper_dir_sign = 1;
  int8_t sonar_pos_sign = 1;
  uint8_t flags = 0;
  uint8_t upper_limit_near_sensor = 1;
  uint8_t sonar_sign_mode = static_cast<uint8_t>(SonarSignMode::kOrientation);
  uint8_t sonar_center_source = static_cast<uint8_t>(CenterSource::kAuto);
  uint8_t actuator_trim_valid = 0;
  uint8_t actuator_trim_source = static_cast<uint8_t>(ActuatorTrimSource::kSearched);
  float sonar_center_cm = 0.0f;
  float as5600_lower_raw_deg = 0.0f;
  float as5600_upper_raw_deg = 0.0f;
  float sonar_lower_cm = 0.0f;
  float sonar_upper_cm = 0.0f;
  float actuator_trim_deg = 0.0f;
  uint16_t crc16 = 0;
};

void runtimeCalInit();
bool runtimeCalLoad();
bool runtimeCalSave();
void runtimeCalResetDefaults();

const RuntimeCalibration& runtimeCalData();

bool runtimeCalIsZeroSet();
bool runtimeCalIsZeroAngleCaptured();
bool runtimeCalIsZeroPosCaptured();
bool runtimeCalIsLimitsSet();
bool runtimeCalIsLowerLimitCaptured();
bool runtimeCalIsUpperLimitCaptured();
bool runtimeCalIsSignSet();
bool runtimeCalHasValidLimitSpan();
bool runtimeCalHasSonarCenter();

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
bool runtimeCalUpperLimitNearSensor();
void runtimeCalSetUpperLimitNearSensor(bool enabled);
SonarSignMode runtimeCalSonarSignMode();
void runtimeCalSetSonarSignMode(SonarSignMode mode);
int8_t runtimeCalOrientationSonarSign();
void runtimeCalApplyOrientationSonarSign();

CenterSource runtimeCalSonarCenterSource();
void runtimeCalSetSonarCenterSource(CenterSource source);

ActuatorTrimSource runtimeCalActuatorTrimSource();
void runtimeCalSetActuatorTrimSource(ActuatorTrimSource source);
bool runtimeCalActuatorTrimValid();
void runtimeCalSetActuatorTrimValid(bool enabled);

float runtimeCalAs5600ZeroDeg();
float runtimeCalSonarCenterCm();
void runtimeCalSetAs5600ZeroDeg(float deg);
void runtimeCalSetSonarCenterCm(float cm);

float runtimeCalAs5600LowerRawDeg();
float runtimeCalAs5600UpperRawDeg();
void runtimeCalSetAs5600LowerRawDeg(float deg);
void runtimeCalSetAs5600UpperRawDeg(float deg);

float runtimeCalSonarLowerCm();
float runtimeCalSonarUpperCm();
float runtimeCalSonarNearCm();
float runtimeCalSonarFarCm();
void runtimeCalSetSonarLowerCm(float cm);
void runtimeCalSetSonarUpperCm(float cm);

float runtimeCalActuatorTrimDeg();
void runtimeCalSetActuatorTrimDeg(float deg);
float runtimeCalActiveActuatorTrimDeg();
float runtimeCalBeamLevelActuatorDeg();
float runtimeCalActuatorSpanDeg();
float runtimeCalActuatorMidpointDeg();
float runtimeCalActiveSonarCenterCm();
float runtimeCalDerivedSonarCenterCm();

float runtimeCalThetaLowerLimitDeg();
float runtimeCalThetaUpperLimitDeg();
void runtimeCalSetThetaLowerLimitDeg(float deg);
void runtimeCalSetThetaUpperLimitDeg(float deg);

float runtimeMapBeamDegFromActuatorDeg(float actuator_deg);
float runtimeMapActuatorDeg(float raw_angle_deg);
float runtimeMapThetaDeg(float raw_angle_deg);
float runtimeMapBallPosCm(float sonar_distance_cm);

}  // namespace bb
