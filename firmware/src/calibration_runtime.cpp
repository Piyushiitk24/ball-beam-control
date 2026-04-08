#include "calibration_runtime.h"

#include <EEPROM.h>

#include <stddef.h>

#include "calibration.h"
#include "config.h"

namespace bb {
namespace {

constexpr uint16_t kRuntimeCalMagic = 0xBB10;
constexpr uint16_t kRuntimeCalVersion = 6;
constexpr uint16_t kRuntimeCalVersionV5 = 5;
constexpr uint16_t kRuntimeCalVersionV4 = 4;
constexpr uint16_t kRuntimeCalVersionV3 = 3;
constexpr int kRuntimeCalEepromAddress = 0;

constexpr uint8_t kFlagZeroAngleCaptured = 1u << 0;
constexpr uint8_t kFlagZeroPosCaptured = 1u << 1;
constexpr uint8_t kFlagLowerLimitCaptured = 1u << 2;
constexpr uint8_t kFlagUpperLimitCaptured = 1u << 3;
constexpr uint8_t kFlagSignCaptured = 1u << 4;

RuntimeCalibration g_runtime_cal;

int8_t normalizeSign(int8_t sign) { return (sign >= 0) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1); }

uint8_t normalizeBoolByte(uint8_t value) { return (value != 0u) ? 1u : 0u; }

CenterSource normalizeCenterSource(uint8_t source) {
  return (source == static_cast<uint8_t>(CenterSource::kManual))
             ? CenterSource::kManual
             : CenterSource::kAuto;
}

ActuatorTrimSource normalizeActuatorTrimSource(uint8_t source) {
  return (source == static_cast<uint8_t>(ActuatorTrimSource::kSaved))
             ? ActuatorTrimSource::kSaved
             : ActuatorTrimSource::kSearched;
}

SonarSignMode normalizeSonarSignMode(uint8_t mode) {
  return (mode == static_cast<uint8_t>(SonarSignMode::kManual))
             ? SonarSignMode::kManual
             : SonarSignMode::kOrientation;
}

int8_t orientationSonarSignFor(uint8_t upper_limit_near_sensor) {
  return (upper_limit_near_sensor != 0u) ? static_cast<int8_t>(-1) : static_cast<int8_t>(1);
}

float wrapAngle360(float deg) {
  while (deg >= 360.0f) {
    deg -= 360.0f;
  }
  while (deg < 0.0f) {
    deg += 360.0f;
  }
  return deg;
}

float clampf(float value, float lo, float hi) {
  if (value < lo) {
    return lo;
  }
  if (value > hi) {
    return hi;
  }
  return value;
}

struct RuntimeCalibrationV4 {
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

struct RuntimeCalibrationV3 {
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

uint16_t runtimeCalV3Crc(const RuntimeCalibrationV3& cal) {
  constexpr size_t kPayloadBytes = offsetof(RuntimeCalibrationV3, crc16);
  return crc16Ccitt(reinterpret_cast<const uint8_t*>(&cal), kPayloadBytes);
}

uint16_t runtimeCalV4Crc(const RuntimeCalibrationV4& cal) {
  constexpr size_t kPayloadBytes = offsetof(RuntimeCalibrationV4, crc16);
  return crc16Ccitt(reinterpret_cast<const uint8_t*>(&cal), kPayloadBytes);
}

bool hasFlag(uint8_t flag) { return (g_runtime_cal.flags & flag) != 0u; }

void setFlag(uint8_t flag, bool enabled) {
  if (enabled) {
    g_runtime_cal.flags |= flag;
  } else {
    g_runtime_cal.flags &= static_cast<uint8_t>(~flag);
  }
}

bool rawLimitsLookValid(const RuntimeCalibration& cal) {
  if (((cal.flags & kFlagLowerLimitCaptured) == 0u) || ((cal.flags & kFlagUpperLimitCaptured) == 0u)) {
    return false;
  }
  const float delta = static_cast<float>(cal.as5600_sign) *
                      wrapAngleDeltaDeg(cal.as5600_upper_raw_deg - cal.as5600_lower_raw_deg);
  return delta > 0.5f;
}

float rawSpanDeg(const RuntimeCalibration& cal) {
  return static_cast<float>(cal.as5600_sign) *
         wrapAngleDeltaDeg(cal.as5600_upper_raw_deg - cal.as5600_lower_raw_deg);
}

float derivedSonarCenterCm(const RuntimeCalibration& cal) {
  return 0.5f * (cal.sonar_lower_cm + cal.sonar_upper_cm);
}

float activeSonarCenterCm(const RuntimeCalibration& cal) {
  if (normalizeCenterSource(cal.sonar_center_source) == CenterSource::kManual) {
    return cal.sonar_center_cm;
  }
  return derivedSonarCenterCm(cal);
}

float actuatorMidpointDeg(const RuntimeCalibration& cal) {
  return 0.5f * rawSpanDeg(cal);
}

float beamLevelActuatorDeg(const RuntimeCalibration& cal) {
  const float span = rawSpanDeg(cal);
  return (span > 0.0f) ? (kBeamLevelFracFromLower * span) : 0.0f;
}

float activeActuatorTrimDeg(const RuntimeCalibration& cal) {
  const float span = rawSpanDeg(cal);
  if (span <= 0.0f) {
    return 0.0f;
  }
  if (normalizeBoolByte(cal.actuator_trim_valid) != 0u) {
    return clampf(cal.actuator_trim_deg, 0.0f, span);
  }
  return beamLevelActuatorDeg(cal);
}

void applyDefaults(RuntimeCalibration& cal) {
  cal = RuntimeCalibration{};
  cal.magic = kRuntimeCalMagic;
  cal.version = kRuntimeCalVersion;
  cal.as5600_sign = normalizeSign(AS5600_SIGN);
  cal.stepper_dir_sign = normalizeSign(STEPPER_DIR_SIGN);
  cal.upper_limit_near_sensor = 1u;
  cal.sonar_sign_mode = static_cast<uint8_t>(SonarSignMode::kOrientation);
  cal.sonar_center_source = static_cast<uint8_t>(CenterSource::kManual);
  cal.actuator_trim_valid = 0u;
  cal.actuator_trim_source = static_cast<uint8_t>(ActuatorTrimSource::kSearched);
  cal.sonar_pos_sign = orientationSonarSignFor(cal.upper_limit_near_sensor);
  cal.sonar_center_cm = SONAR_CENTER_CM;
  cal.as5600_lower_raw_deg = 0.0f;
  cal.as5600_upper_raw_deg = 0.0f;
  cal.sonar_lower_cm = 0.0f;
  cal.sonar_upper_cm = 0.0f;
  cal.actuator_trim_deg = 0.0f;
  cal.flags = 0;
  cal.crc16 = runtimeCalCrc(cal);
}

void sanitizeLoaded(RuntimeCalibration& cal) {
  cal.as5600_sign = normalizeSign(cal.as5600_sign);
  cal.stepper_dir_sign = normalizeSign(cal.stepper_dir_sign);
  cal.sonar_pos_sign = normalizeSign(cal.sonar_pos_sign);
  cal.upper_limit_near_sensor = normalizeBoolByte(cal.upper_limit_near_sensor);
  cal.sonar_sign_mode = static_cast<uint8_t>(normalizeSonarSignMode(cal.sonar_sign_mode));
  cal.sonar_center_source = static_cast<uint8_t>(normalizeCenterSource(cal.sonar_center_source));
  cal.actuator_trim_valid = normalizeBoolByte(cal.actuator_trim_valid);
  cal.actuator_trim_source = static_cast<uint8_t>(normalizeActuatorTrimSource(cal.actuator_trim_source));
  cal.as5600_lower_raw_deg = wrapAngle360(cal.as5600_lower_raw_deg);
  cal.as5600_upper_raw_deg = wrapAngle360(cal.as5600_upper_raw_deg);

  if (!rawLimitsLookValid(cal)) {
    cal.flags &= static_cast<uint8_t>(~(kFlagLowerLimitCaptured | kFlagUpperLimitCaptured | kFlagZeroAngleCaptured));
    cal.actuator_trim_valid = 0u;
    cal.actuator_trim_deg = 0.0f;
  }

  if (normalizeSonarSignMode(cal.sonar_sign_mode) == SonarSignMode::kOrientation) {
    cal.sonar_pos_sign = orientationSonarSignFor(cal.upper_limit_near_sensor);
  }

  if (cal.sonar_center_cm <= 0.0f) {
    cal.flags &= static_cast<uint8_t>(~kFlagZeroPosCaptured);
  }

  const float span = rawSpanDeg(cal);
  if (span > 0.0f) {
    cal.actuator_trim_deg = clampf(cal.actuator_trim_deg, 0.0f, span);
  } else {
    cal.actuator_trim_deg = 0.0f;
  }
  cal.actuator_trim_valid =
      (cal.actuator_trim_valid != 0u) && (span > 0.0f) ? 1u : 0u;
  if (cal.actuator_trim_valid == 0u && span > 0.0f) {
    cal.actuator_trim_deg = beamLevelActuatorDeg(cal);
  }
  cal.flags &= static_cast<uint8_t>(~kFlagZeroAngleCaptured);
}

void migrateV3ToV4(const RuntimeCalibrationV3& legacy, RuntimeCalibration& cal) {
  applyDefaults(cal);
  cal.flags = legacy.flags;
  cal.as5600_sign = normalizeSign(legacy.as5600_sign);
  cal.stepper_dir_sign = normalizeSign(legacy.stepper_dir_sign);
  cal.sonar_pos_sign = normalizeSign(legacy.sonar_pos_sign);
  cal.upper_limit_near_sensor = normalizeBoolByte(legacy.upper_limit_near_sensor);
  cal.sonar_sign_mode = legacy.sonar_sign_mode;
  cal.sonar_center_source = legacy.sonar_center_source;
  cal.sonar_center_cm = legacy.sonar_center_cm;
  cal.as5600_lower_raw_deg = legacy.as5600_lower_raw_deg;
  cal.as5600_upper_raw_deg = legacy.as5600_upper_raw_deg;
  cal.sonar_lower_cm = legacy.sonar_lower_cm;
  cal.sonar_upper_cm = legacy.sonar_upper_cm;
  cal.actuator_trim_deg = legacy.actuator_trim_deg;
  cal.actuator_trim_valid = 0u;
  cal.actuator_trim_source = static_cast<uint8_t>(ActuatorTrimSource::kSearched);
  cal.flags &= static_cast<uint8_t>(~kFlagZeroAngleCaptured);
  sanitizeLoaded(cal);
}

void migrateV4ToV5(const RuntimeCalibrationV4& legacy, RuntimeCalibration& cal) {
  applyDefaults(cal);
  cal.flags = legacy.flags;
  cal.as5600_sign = normalizeSign(legacy.as5600_sign);
  cal.stepper_dir_sign = normalizeSign(legacy.stepper_dir_sign);
  cal.sonar_pos_sign = normalizeSign(legacy.sonar_pos_sign);
  cal.upper_limit_near_sensor = normalizeBoolByte(legacy.upper_limit_near_sensor);
  cal.sonar_sign_mode = legacy.sonar_sign_mode;
  cal.sonar_center_source = legacy.sonar_center_source;
  cal.sonar_center_cm = legacy.sonar_center_cm;
  cal.as5600_lower_raw_deg = legacy.as5600_lower_raw_deg;
  cal.as5600_upper_raw_deg = legacy.as5600_upper_raw_deg;
  cal.sonar_lower_cm = legacy.sonar_lower_cm;
  cal.sonar_upper_cm = legacy.sonar_upper_cm;
  cal.actuator_trim_deg = legacy.actuator_trim_deg;
  // Force a fresh `p` capture for the new step-count-origin controller.
  cal.actuator_trim_valid = 0u;
  cal.actuator_trim_source = static_cast<uint8_t>(ActuatorTrimSource::kSearched);
  cal.flags &= static_cast<uint8_t>(~kFlagZeroAngleCaptured);
  sanitizeLoaded(cal);
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

  if (loaded.magic != kRuntimeCalMagic) {
    applyDefaults(g_runtime_cal);
    return false;
  }

  if (loaded.version == kRuntimeCalVersion) {
    const uint16_t expected_crc = runtimeCalCrc(loaded);
    if (loaded.crc16 != expected_crc) {
      applyDefaults(g_runtime_cal);
      return false;
    }

    g_runtime_cal = loaded;
    sanitizeLoaded(g_runtime_cal);
    if (g_runtime_cal.actuator_trim_valid != 0u) {
      g_runtime_cal.actuator_trim_source = static_cast<uint8_t>(ActuatorTrimSource::kSaved);
    }
    g_runtime_cal.crc16 = runtimeCalCrc(g_runtime_cal);
    return true;
  }

  if (loaded.version == kRuntimeCalVersionV5) {
    const uint16_t expected_crc = runtimeCalCrc(loaded);
    if (loaded.crc16 != expected_crc) {
      applyDefaults(g_runtime_cal);
      return false;
    }

    g_runtime_cal = loaded;
    // V5 stored the center/trim as a captured actuator pose. V6 derives the
    // level trim from the measured beam-angle calibration, so require a fresh
    // center capture while preserving limit and sonar endpoint data.
    g_runtime_cal.actuator_trim_valid = 0u;
    g_runtime_cal.actuator_trim_source = static_cast<uint8_t>(ActuatorTrimSource::kSearched);
    g_runtime_cal.flags &= static_cast<uint8_t>(~kFlagZeroAngleCaptured);
    sanitizeLoaded(g_runtime_cal);
    runtimeCalSave();
    return true;
  }

  if (loaded.version == kRuntimeCalVersionV4) {
    RuntimeCalibrationV4 legacy{};
    EEPROM.get(kRuntimeCalEepromAddress, legacy);
    const uint16_t expected_crc = runtimeCalV4Crc(legacy);
    if (legacy.crc16 != expected_crc) {
      applyDefaults(g_runtime_cal);
      return false;
    }

    migrateV4ToV5(legacy, g_runtime_cal);
    runtimeCalSave();
    return true;
  }

  if (loaded.version == kRuntimeCalVersionV3) {
    RuntimeCalibrationV3 legacy{};
    EEPROM.get(kRuntimeCalEepromAddress, legacy);
    const uint16_t expected_crc = runtimeCalV3Crc(legacy);
    if (legacy.crc16 != expected_crc) {
      applyDefaults(g_runtime_cal);
      return false;
    }

    migrateV3ToV4(legacy, g_runtime_cal);
    runtimeCalSave();
    return true;
  }

  applyDefaults(g_runtime_cal);
  return false;
}

bool runtimeCalSave() {
  g_runtime_cal.magic = kRuntimeCalMagic;
  g_runtime_cal.version = kRuntimeCalVersion;
  if (g_runtime_cal.actuator_trim_valid != 0u) {
    g_runtime_cal.actuator_trim_source = static_cast<uint8_t>(ActuatorTrimSource::kSaved);
  }
  sanitizeLoaded(g_runtime_cal);
  g_runtime_cal.crc16 = runtimeCalCrc(g_runtime_cal);
  EEPROM.put(kRuntimeCalEepromAddress, g_runtime_cal);
  return true;
}

void runtimeCalResetDefaults() {
  applyDefaults(g_runtime_cal);
}

const RuntimeCalibration& runtimeCalData() { return g_runtime_cal; }

bool runtimeCalIsZeroSet() { return hasFlag(kFlagZeroPosCaptured) && runtimeCalActuatorTrimValid(); }

bool runtimeCalIsZeroAngleCaptured() { return runtimeCalActuatorTrimValid(); }

bool runtimeCalIsZeroPosCaptured() { return hasFlag(kFlagZeroPosCaptured); }

bool runtimeCalHasValidLimitSpan() { return rawLimitsLookValid(g_runtime_cal); }

bool runtimeCalIsLimitsSet() {
  return hasFlag(kFlagLowerLimitCaptured) && hasFlag(kFlagUpperLimitCaptured) && runtimeCalHasValidLimitSpan();
}

bool runtimeCalIsLowerLimitCaptured() { return hasFlag(kFlagLowerLimitCaptured); }

bool runtimeCalIsUpperLimitCaptured() { return hasFlag(kFlagUpperLimitCaptured); }

bool runtimeCalIsSignSet() { return hasFlag(kFlagSignCaptured); }

bool runtimeCalHasSonarCenter() { return hasFlag(kFlagZeroPosCaptured); }

void runtimeCalMarkZeroAngleCaptured(bool enabled) {
  g_runtime_cal.actuator_trim_valid = enabled ? 1u : 0u;
  setFlag(kFlagZeroAngleCaptured, enabled);
}

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

bool runtimeCalUpperLimitNearSensor() { return g_runtime_cal.upper_limit_near_sensor != 0u; }

void runtimeCalSetUpperLimitNearSensor(bool enabled) {
  g_runtime_cal.upper_limit_near_sensor = enabled ? 1u : 0u;
}

SonarSignMode runtimeCalSonarSignMode() {
  return normalizeSonarSignMode(g_runtime_cal.sonar_sign_mode);
}

void runtimeCalSetSonarSignMode(SonarSignMode mode) {
  g_runtime_cal.sonar_sign_mode = static_cast<uint8_t>(mode);
}

int8_t runtimeCalOrientationSonarSign() {
  return orientationSonarSignFor(g_runtime_cal.upper_limit_near_sensor);
}

void runtimeCalApplyOrientationSonarSign() {
  g_runtime_cal.sonar_pos_sign = runtimeCalOrientationSonarSign();
  g_runtime_cal.sonar_sign_mode = static_cast<uint8_t>(SonarSignMode::kOrientation);
}

CenterSource runtimeCalSonarCenterSource() {
  return normalizeCenterSource(g_runtime_cal.sonar_center_source);
}

void runtimeCalSetSonarCenterSource(CenterSource source) {
  g_runtime_cal.sonar_center_source = static_cast<uint8_t>(source);
}

ActuatorTrimSource runtimeCalActuatorTrimSource() {
  return normalizeActuatorTrimSource(g_runtime_cal.actuator_trim_source);
}

void runtimeCalSetActuatorTrimSource(ActuatorTrimSource source) {
  g_runtime_cal.actuator_trim_source = static_cast<uint8_t>(source);
}

bool runtimeCalActuatorTrimValid() { return g_runtime_cal.actuator_trim_valid != 0u; }

void runtimeCalSetActuatorTrimValid(bool enabled) { g_runtime_cal.actuator_trim_valid = enabled ? 1u : 0u; }

float runtimeCalAs5600ZeroDeg() { return runtimeCalActiveActuatorTrimDeg(); }

float runtimeCalSonarCenterCm() { return runtimeCalActiveSonarCenterCm(); }

void runtimeCalSetAs5600ZeroDeg(float deg) {
  g_runtime_cal.actuator_trim_deg = deg;
  g_runtime_cal.actuator_trim_valid = 1u;
}

void runtimeCalSetSonarCenterCm(float cm) { g_runtime_cal.sonar_center_cm = cm; }

float runtimeCalAs5600LowerRawDeg() { return g_runtime_cal.as5600_lower_raw_deg; }

float runtimeCalAs5600UpperRawDeg() { return g_runtime_cal.as5600_upper_raw_deg; }

void runtimeCalSetAs5600LowerRawDeg(float deg) { g_runtime_cal.as5600_lower_raw_deg = wrapAngle360(deg); }

void runtimeCalSetAs5600UpperRawDeg(float deg) { g_runtime_cal.as5600_upper_raw_deg = wrapAngle360(deg); }

float runtimeCalSonarLowerCm() { return g_runtime_cal.sonar_lower_cm; }

float runtimeCalSonarUpperCm() { return g_runtime_cal.sonar_upper_cm; }

float runtimeCalSonarNearCm() {
  return runtimeCalUpperLimitNearSensor() ? g_runtime_cal.sonar_upper_cm : g_runtime_cal.sonar_lower_cm;
}

float runtimeCalSonarFarCm() {
  return runtimeCalUpperLimitNearSensor() ? g_runtime_cal.sonar_lower_cm : g_runtime_cal.sonar_upper_cm;
}

void runtimeCalSetSonarLowerCm(float cm) { g_runtime_cal.sonar_lower_cm = cm; }

void runtimeCalSetSonarUpperCm(float cm) { g_runtime_cal.sonar_upper_cm = cm; }

float runtimeCalActuatorTrimDeg() { return g_runtime_cal.actuator_trim_deg; }

void runtimeCalSetActuatorTrimDeg(float deg) { g_runtime_cal.actuator_trim_deg = deg; }

float runtimeCalActiveActuatorTrimDeg() { return activeActuatorTrimDeg(g_runtime_cal); }

float runtimeCalBeamLevelActuatorDeg() { return beamLevelActuatorDeg(g_runtime_cal); }

float runtimeCalActuatorSpanDeg() {
  const float span = rawSpanDeg(g_runtime_cal);
  return (span > 0.0f) ? span : 0.0f;
}

float runtimeCalActuatorMidpointDeg() { return actuatorMidpointDeg(g_runtime_cal); }

float runtimeCalActiveSonarCenterCm() {
  if (!runtimeCalHasSonarCenter()) {
    if (runtimeCalIsLimitsSet()) {
      return derivedSonarCenterCm(g_runtime_cal);
    }
    return g_runtime_cal.sonar_center_cm;
  }
  return activeSonarCenterCm(g_runtime_cal);
}

float runtimeCalDerivedSonarCenterCm() { return derivedSonarCenterCm(g_runtime_cal); }

float runtimeCalThetaLowerLimitDeg() { return runtimeMapBeamDegFromActuatorDeg(0.0f); }

float runtimeCalThetaUpperLimitDeg() {
  return runtimeMapBeamDegFromActuatorDeg(runtimeCalActuatorSpanDeg());
}

void runtimeCalSetThetaLowerLimitDeg(float deg) {
  g_runtime_cal.as5600_lower_raw_deg = wrapAngle360(g_runtime_cal.as5600_lower_raw_deg);
  const float trim = -deg / kBeamDegPerActuatorDeg;
  g_runtime_cal.actuator_trim_deg = trim;
}

void runtimeCalSetThetaUpperLimitDeg(float deg) {
  const float trim = runtimeCalActuatorSpanDeg() - (deg / kBeamDegPerActuatorDeg);
  g_runtime_cal.actuator_trim_deg = trim;
}

float runtimeMapBeamDegFromActuatorDeg(float actuator_deg) {
  return (actuator_deg - runtimeCalActiveActuatorTrimDeg()) * kBeamDegPerActuatorDeg;
}

float runtimeMapActuatorDeg(float raw_angle_deg) {
  const float ref_raw = g_runtime_cal.as5600_lower_raw_deg;
  const float delta_deg = wrapAngleDeltaDeg(raw_angle_deg - ref_raw);
  return static_cast<float>(g_runtime_cal.as5600_sign) * delta_deg;
}

float runtimeMapThetaDeg(float raw_angle_deg) {
  return runtimeMapBeamDegFromActuatorDeg(runtimeMapActuatorDeg(raw_angle_deg));
}

float runtimeMapBallPosCm(float sonar_distance_cm) {
  return static_cast<float>(g_runtime_cal.sonar_pos_sign) *
         (sonar_distance_cm - runtimeCalActiveSonarCenterCm());
}

}  // namespace bb
