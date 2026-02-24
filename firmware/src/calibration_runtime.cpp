#include "calibration_runtime.h"

#include <EEPROM.h>

#include <stddef.h>
#include <string.h>

#include "calibration.h"
#include "config.h"

namespace bb {
namespace {

constexpr uint16_t kRuntimeCalMagic = 0xBB10;
constexpr uint16_t kRuntimeCalVersion = 1;
constexpr int kRuntimeCalEepromAddress = 0;

constexpr uint8_t kFlagZeroAngleCaptured = 1u << 0;
constexpr uint8_t kFlagZeroPosCaptured = 1u << 1;
constexpr uint8_t kFlagLowerLimitCaptured = 1u << 2;
constexpr uint8_t kFlagUpperLimitCaptured = 1u << 3;
constexpr uint8_t kFlagSignCaptured = 1u << 4;

RuntimeCalibration g_runtime_cal;
RuntimeCalLoadStatus g_load_status = RuntimeCalLoadStatus::kUninitialized;

int8_t normalizeSign(int8_t sign) { return (sign >= 0) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1); }

uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000u) != 0u) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

uint16_t runtimeCalCrc(const RuntimeCalibration& cal) {
  constexpr size_t kPayloadBytes = offsetof(RuntimeCalibration, crc16);
  return crc16Ccitt(reinterpret_cast<const uint8_t*>(&cal), kPayloadBytes);
}

void applyDefaults(RuntimeCalibration& cal) {
  cal = RuntimeCalibration{};
  cal.magic = kRuntimeCalMagic;
  cal.version = kRuntimeCalVersion;
  cal.as5600_sign = normalizeSign(AS5600_SIGN);
  cal.stepper_dir_sign = normalizeSign(STEPPER_DIR_SIGN);
  cal.sonar_pos_sign = normalizeSign(SONAR_POS_SIGN);
  cal.as5600_zero_deg = AS5600_ZERO_DEG;
  cal.sonar_center_cm = SONAR_CENTER_CM;
  cal.theta_lower_limit_deg = -kThetaCmdLimitDeg;
  cal.theta_upper_limit_deg = kThetaCmdLimitDeg;
  cal.flags = 0;
  cal.crc16 = runtimeCalCrc(cal);
}

void sanitizeLoaded(RuntimeCalibration& cal) {
  cal.as5600_sign = normalizeSign(cal.as5600_sign);
  cal.stepper_dir_sign = normalizeSign(cal.stepper_dir_sign);
  cal.sonar_pos_sign = normalizeSign(cal.sonar_pos_sign);
  if (cal.theta_lower_limit_deg >= cal.theta_upper_limit_deg) {
    cal.flags &= static_cast<uint8_t>(~(kFlagLowerLimitCaptured | kFlagUpperLimitCaptured));
  }
}

bool hasFlag(uint8_t flag) { return (g_runtime_cal.flags & flag) != 0u; }

void setFlag(uint8_t flag, bool enabled) {
  if (enabled) {
    g_runtime_cal.flags |= flag;
  } else {
    g_runtime_cal.flags &= static_cast<uint8_t>(~flag);
  }
}

}  // namespace

void runtimeCalInit() {
  if (!runtimeCalLoad()) {
    runtimeCalResetDefaults();
  }
}

bool runtimeCalLoad() {
  RuntimeCalibration loaded{};
  EEPROM.get(kRuntimeCalEepromAddress, loaded);

  if (loaded.magic != kRuntimeCalMagic || loaded.version != kRuntimeCalVersion) {
    applyDefaults(g_runtime_cal);
    g_load_status = RuntimeCalLoadStatus::kDefaultsApplied;
    return false;
  }

  const uint16_t expected_crc = runtimeCalCrc(loaded);
  if (loaded.crc16 != expected_crc) {
    applyDefaults(g_runtime_cal);
    g_load_status = RuntimeCalLoadStatus::kDefaultsApplied;
    return false;
  }

  g_runtime_cal = loaded;
  sanitizeLoaded(g_runtime_cal);
  g_runtime_cal.crc16 = runtimeCalCrc(g_runtime_cal);
  g_load_status = RuntimeCalLoadStatus::kLoadedFromEeprom;
  return true;
}

bool runtimeCalSave() {
  g_runtime_cal.magic = kRuntimeCalMagic;
  g_runtime_cal.version = kRuntimeCalVersion;
  sanitizeLoaded(g_runtime_cal);
  g_runtime_cal.crc16 = runtimeCalCrc(g_runtime_cal);
  EEPROM.put(kRuntimeCalEepromAddress, g_runtime_cal);
  g_load_status = RuntimeCalLoadStatus::kLoadedFromEeprom;
  return true;
}

void runtimeCalResetDefaults() {
  applyDefaults(g_runtime_cal);
  g_load_status = RuntimeCalLoadStatus::kDefaultsApplied;
}

const RuntimeCalibration& runtimeCalData() { return g_runtime_cal; }

RuntimeCalLoadStatus runtimeCalLoadStatus() { return g_load_status; }

const char* runtimeCalLoadStatusName(RuntimeCalLoadStatus status) {
  switch (status) {
    case RuntimeCalLoadStatus::kLoadedFromEeprom:
      return "loaded";
    case RuntimeCalLoadStatus::kDefaultsApplied:
      return "defaults";
    case RuntimeCalLoadStatus::kUninitialized:
    default:
      return "uninitialized";
  }
}

bool runtimeCalIsZeroSet() { return hasFlag(kFlagZeroAngleCaptured) && hasFlag(kFlagZeroPosCaptured); }

bool runtimeCalIsZeroAngleCaptured() { return hasFlag(kFlagZeroAngleCaptured); }

bool runtimeCalIsZeroPosCaptured() { return hasFlag(kFlagZeroPosCaptured); }

bool runtimeCalHasValidLimitSpan() { return g_runtime_cal.theta_lower_limit_deg < g_runtime_cal.theta_upper_limit_deg; }

bool runtimeCalIsLimitsSet() {
  return hasFlag(kFlagLowerLimitCaptured) && hasFlag(kFlagUpperLimitCaptured) && runtimeCalHasValidLimitSpan();
}

bool runtimeCalIsLowerLimitCaptured() { return hasFlag(kFlagLowerLimitCaptured); }

bool runtimeCalIsUpperLimitCaptured() { return hasFlag(kFlagUpperLimitCaptured); }

bool runtimeCalIsSignSet() { return hasFlag(kFlagSignCaptured); }

void runtimeCalMarkZeroAngleCaptured(bool enabled) { setFlag(kFlagZeroAngleCaptured, enabled); }

void runtimeCalMarkZeroPosCaptured(bool enabled) { setFlag(kFlagZeroPosCaptured, enabled); }

void runtimeCalMarkLowerLimitCaptured(bool enabled) { setFlag(kFlagLowerLimitCaptured, enabled); }

void runtimeCalMarkUpperLimitCaptured(bool enabled) { setFlag(kFlagUpperLimitCaptured, enabled); }

void runtimeCalSetSignCaptured(bool enabled) { setFlag(kFlagSignCaptured, enabled); }

int8_t runtimeCalAs5600Sign() { return g_runtime_cal.as5600_sign; }

int8_t runtimeCalStepperDirSign() { return g_runtime_cal.stepper_dir_sign; }

int8_t runtimeCalSonarPosSign() { return g_runtime_cal.sonar_pos_sign; }

void runtimeCalSetAs5600Sign(int8_t sign) { g_runtime_cal.as5600_sign = normalizeSign(sign); }

void runtimeCalSetStepperDirSign(int8_t sign) { g_runtime_cal.stepper_dir_sign = normalizeSign(sign); }

void runtimeCalSetSonarPosSign(int8_t sign) { g_runtime_cal.sonar_pos_sign = normalizeSign(sign); }

float runtimeCalAs5600ZeroDeg() { return g_runtime_cal.as5600_zero_deg; }

float runtimeCalSonarCenterCm() { return g_runtime_cal.sonar_center_cm; }

void runtimeCalSetAs5600ZeroDeg(float deg) { g_runtime_cal.as5600_zero_deg = deg; }

void runtimeCalSetSonarCenterCm(float cm) { g_runtime_cal.sonar_center_cm = cm; }

float runtimeCalThetaLowerLimitDeg() { return g_runtime_cal.theta_lower_limit_deg; }

float runtimeCalThetaUpperLimitDeg() { return g_runtime_cal.theta_upper_limit_deg; }

void runtimeCalSetThetaLowerLimitDeg(float deg) { g_runtime_cal.theta_lower_limit_deg = deg; }

void runtimeCalSetThetaUpperLimitDeg(float deg) { g_runtime_cal.theta_upper_limit_deg = deg; }

float runtimeMapThetaDeg(float raw_angle_deg) {
  const float delta_deg = wrapAngleDeltaDeg(raw_angle_deg - g_runtime_cal.as5600_zero_deg);
  return static_cast<float>(g_runtime_cal.as5600_sign) * delta_deg;
}

float runtimeMapBallPosCm(float sonar_distance_cm) {
  return static_cast<float>(g_runtime_cal.sonar_pos_sign) * (sonar_distance_cm - g_runtime_cal.sonar_center_cm);
}

}  // namespace bb
