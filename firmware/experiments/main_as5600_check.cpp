#include <Arduino.h>
#include <Wire.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

namespace {

constexpr uint8_t kAs5600Addr = 0x36;
constexpr uint8_t kRawAngleHighReg = 0x0C;
constexpr uint8_t kRawAngleLowReg = 0x0D;

constexpr uint16_t kCountsPerRev = 4096;
constexpr float kCountsToDeg = 360.0f / 4096.0f;

constexpr uint16_t kCaptureNeedGood = 80;
constexpr uint32_t kCaptureTimeoutMs = 5000;
constexpr float kMaxJumpDeg = 25.0f;

constexpr uint16_t kMedianReads = 3;
constexpr uint16_t kMedianDelayUs = 1500;
constexpr uint8_t kCaptureDelayMs = 6;

uint32_t g_i2c_err = 0;

bool g_stream_enabled = false;
uint32_t g_last_stream_ms = 0;

struct CaptureStats {
  bool has = false;
  uint16_t n = 0;
  float med_deg = 0.0f;   // wrapped to [0,360)
  float mean_deg = 0.0f;  // wrapped to [0,360)
  float std_deg = 0.0f;   // std in unwrapped domain
  float min_deg = 0.0f;   // unwrapped
  float max_deg = 0.0f;   // unwrapped
  bool wrap_suspected = false;
  uint32_t i2c_err = 0;
};

CaptureStats g_lower;
CaptureStats g_center;
CaptureStats g_upper;

char g_cmd_buf[64];
size_t g_cmd_len = 0;

float wrapDeltaDeg(float delta_deg) {
  while (delta_deg > 180.0f) {
    delta_deg -= 360.0f;
  }
  while (delta_deg < -180.0f) {
    delta_deg += 360.0f;
  }
  return delta_deg;
}

float wrap360(float deg) {
  while (deg < 0.0f) {
    deg += 360.0f;
  }
  while (deg >= 360.0f) {
    deg -= 360.0f;
  }
  return deg;
}

bool readRawOnce(uint16_t& raw) {
  Wire.beginTransmission(kAs5600Addr);
  Wire.write(kRawAngleHighReg);
  if (Wire.endTransmission(false) != 0) {
    ++g_i2c_err;
    return false;
  }

  const uint8_t requested = Wire.requestFrom(kAs5600Addr, static_cast<uint8_t>(2));
  if (requested != 2 || Wire.available() < 2) {
    ++g_i2c_err;
    return false;
  }

  const uint8_t high = Wire.read();
  const uint8_t low = Wire.read();
  raw = static_cast<uint16_t>(((high & 0x0F) << 8) | low);
  return true;
}

bool readRawMedian3(uint16_t& out_raw) {
  uint16_t r[kMedianReads];
  for (uint16_t i = 0; i < kMedianReads; ++i) {
    if (!readRawOnce(r[i])) {
      return false;
    }
    delayMicroseconds(kMedianDelayUs);
  }

  // Unwrap around r[0] so the median works near 0/4095 boundary.
  int16_t a = static_cast<int16_t>(r[0]);
  int16_t b = static_cast<int16_t>(r[1]);
  int16_t c = static_cast<int16_t>(r[2]);

  int16_t d = static_cast<int16_t>(b - a);
  if (d > 2048) {
    b = static_cast<int16_t>(b - 4096);
  } else if (d < -2048) {
    b = static_cast<int16_t>(b + 4096);
  }

  d = static_cast<int16_t>(c - a);
  if (d > 2048) {
    c = static_cast<int16_t>(c - 4096);
  } else if (d < -2048) {
    c = static_cast<int16_t>(c + 4096);
  }

  // Median-of-3 on int16.
  if (a > b) {
    const int16_t t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    const int16_t t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    const int16_t t = a;
    a = b;
    b = t;
  }

  int16_t med = b;
  if (med < 0) {
    med = static_cast<int16_t>(med + 4096);
  } else if (med >= 4096) {
    med = static_cast<int16_t>(med - 4096);
  }
  out_raw = static_cast<uint16_t>(med);
  return true;
}

float rawToDeg(uint16_t raw) { return static_cast<float>(raw) * kCountsToDeg; }

void sortFloats(float* a, uint16_t n) {
  // Insertion sort (n is small; avoids pulling in qsort).
  for (uint16_t i = 1; i < n; ++i) {
    const float key = a[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      --j;
    }
    a[j + 1] = key;
  }
}

bool capture(const char* tag, CaptureStats& out) {
  float samples[kCaptureNeedGood];
  uint16_t good = 0;

  bool last_init = false;
  float last_wrap = 0.0f;
  float last_unwrap = 0.0f;

  uint16_t cnt_low = 0;
  uint16_t cnt_high = 0;

  const uint32_t start_ms = millis();
  while (good < kCaptureNeedGood && static_cast<uint32_t>(millis() - start_ms) < kCaptureTimeoutMs) {
    uint16_t raw = 0;
    if (!readRawMedian3(raw)) {
      delay(kCaptureDelayMs);
      continue;
    }

    const float deg_wrap = rawToDeg(raw);
    if (deg_wrap < 20.0f) {
      ++cnt_low;
    } else if (deg_wrap > 340.0f) {
      ++cnt_high;
    }

    if (!last_init) {
      last_wrap = deg_wrap;
      last_unwrap = deg_wrap;
      last_init = true;
      samples[good++] = last_unwrap;
      delay(kCaptureDelayMs);
      continue;
    }

    const float delta = wrapDeltaDeg(deg_wrap - last_wrap);
    if (fabsf(delta) > kMaxJumpDeg) {
      delay(kCaptureDelayMs);
      continue;
    }

    last_wrap = deg_wrap;
    last_unwrap = last_unwrap + delta;
    samples[good++] = last_unwrap;
    delay(kCaptureDelayMs);
  }

  if (good < kCaptureNeedGood) {
    Serial.print(F("ERR,capture_timeout,tag="));
    Serial.print(tag);
    Serial.print(F(",good="));
    Serial.print(good);
    Serial.print(F(",need="));
    Serial.println(kCaptureNeedGood);
    return false;
  }

  float sum = 0.0f;
  float min_v = samples[0];
  float max_v = samples[0];
  for (uint16_t i = 0; i < good; ++i) {
    const float v = samples[i];
    sum += v;
    if (v < min_v) {
      min_v = v;
    }
    if (v > max_v) {
      max_v = v;
    }
  }
  const float mean = sum / static_cast<float>(good);

  float var = 0.0f;
  for (uint16_t i = 0; i < good; ++i) {
    const float d = samples[i] - mean;
    var += d * d;
  }
  var /= static_cast<float>(good);
  const float std = sqrtf(var);

  float sorted[kCaptureNeedGood];
  for (uint16_t i = 0; i < good; ++i) {
    sorted[i] = samples[i];
  }
  sortFloats(sorted, good);
  float med_unwrap = 0.0f;
  if ((good % 2u) == 1u) {
    med_unwrap = sorted[good / 2u];
  } else {
    const float a = sorted[(good / 2u) - 1u];
    const float b = sorted[good / 2u];
    med_unwrap = 0.5f * (a + b);
  }

  out.has = true;
  out.n = good;
  out.med_deg = wrap360(med_unwrap);
  out.mean_deg = wrap360(mean);
  out.std_deg = std;
  out.min_deg = min_v;
  out.max_deg = max_v;
  out.wrap_suspected = (cnt_low > 0 && cnt_high > 0);
  out.i2c_err = g_i2c_err;

  Serial.print(F("AS5600_CAP,tag="));
  Serial.print(tag);
  Serial.print(F(",n="));
  Serial.print(out.n);
  Serial.print(F(",med_deg="));
  Serial.print(out.med_deg, 4);
  Serial.print(F(",mean_deg="));
  Serial.print(out.mean_deg, 4);
  Serial.print(F(",std_deg="));
  Serial.print(out.std_deg, 4);
  Serial.print(F(",min_deg="));
  Serial.print(out.min_deg, 4);
  Serial.print(F(",max_deg="));
  Serial.print(out.max_deg, 4);
  Serial.print(F(",wrap_suspect="));
  Serial.print(out.wrap_suspected ? F("1") : F("0"));
  Serial.print(F(",i2c_err="));
  Serial.println(out.i2c_err);

  return true;
}

void printHelp() {
  Serial.println(F("AS5600_CHECK_HELP"));
  Serial.println(F("Workflow:"));
  Serial.println(F("  1) Move beam to LOWER stop, then press L"));
  Serial.println(F("  2) Move beam to CENTER (balanced), then press C"));
  Serial.println(F("  3) Move beam to UPPER stop, then press U"));
  Serial.println(F("  4) Press P to print comparisons"));
  Serial.println(F("Commands:"));
  Serial.println(F("  h  help"));
  Serial.println(F("  t  toggle stream (20 Hz)"));
  Serial.println(F("  s  single sample"));
  Serial.println(F("  L  capture LOWER"));
  Serial.println(F("  C  capture CENTER"));
  Serial.println(F("  U  capture UPPER"));
  Serial.println(F("  P  print summary"));
  Serial.println(F("  R  reset captures"));
}

void printOneSample() {
  uint16_t raw = 0;
  const uint32_t ts = millis();
  const bool ok = readRawMedian3(raw);
  Serial.print(F("AS5600_SAMPLE,ts_ms="));
  Serial.print(ts);
  Serial.print(F(",ok="));
  Serial.print(ok ? F("1") : F("0"));
  Serial.print(F(",raw="));
  Serial.print(raw);
  Serial.print(F(",deg="));
  Serial.print(rawToDeg(raw), 4);
  Serial.print(F(",i2c_err="));
  Serial.println(g_i2c_err);
}

void printSummary() {
  Serial.println(F("AS5600_SUMMARY"));
  if (g_lower.has) {
    Serial.print(F("AS5600_POS,tag=LOWER,med_deg="));
    Serial.print(g_lower.med_deg, 4);
    Serial.print(F(",std_deg="));
    Serial.print(g_lower.std_deg, 4);
    Serial.print(F(",wrap_suspect="));
    Serial.print(g_lower.wrap_suspected ? F("1") : F("0"));
    Serial.print(F(",i2c_err="));
    Serial.println(g_lower.i2c_err);
  }
  if (g_center.has) {
    Serial.print(F("AS5600_POS,tag=CENTER,med_deg="));
    Serial.print(g_center.med_deg, 4);
    Serial.print(F(",std_deg="));
    Serial.print(g_center.std_deg, 4);
    Serial.print(F(",wrap_suspect="));
    Serial.print(g_center.wrap_suspected ? F("1") : F("0"));
    Serial.print(F(",i2c_err="));
    Serial.println(g_center.i2c_err);
  }
  if (g_upper.has) {
    Serial.print(F("AS5600_POS,tag=UPPER,med_deg="));
    Serial.print(g_upper.med_deg, 4);
    Serial.print(F(",std_deg="));
    Serial.print(g_upper.std_deg, 4);
    Serial.print(F(",wrap_suspect="));
    Serial.print(g_upper.wrap_suspected ? F("1") : F("0"));
    Serial.print(F(",i2c_err="));
    Serial.println(g_upper.i2c_err);
  }

  const bool have_lc = g_lower.has && g_center.has;
  const bool have_cu = g_center.has && g_upper.has;
  const bool have_lu = g_lower.has && g_upper.has;

  if (!have_lc && !have_cu && !have_lu) {
    Serial.println(F("INFO,summary_needs_at_least_two_captures"));
    return;
  }

  Serial.print(F("AS5600_DELTA"));
  if (have_lc) {
    const float d = wrapDeltaDeg(g_center.med_deg - g_lower.med_deg);
    Serial.print(F(",dLC_deg="));
    Serial.print(d, 4);
    Serial.print(F(",absLC_deg="));
    Serial.print(fabsf(d), 4);
  }
  if (have_cu) {
    const float d = wrapDeltaDeg(g_upper.med_deg - g_center.med_deg);
    Serial.print(F(",dCU_deg="));
    Serial.print(d, 4);
    Serial.print(F(",absCU_deg="));
    Serial.print(fabsf(d), 4);
  }
  if (have_lu) {
    const float d = wrapDeltaDeg(g_upper.med_deg - g_lower.med_deg);
    Serial.print(F(",dLU_deg="));
    Serial.print(d, 4);
    Serial.print(F(",absLU_deg="));
    Serial.print(fabsf(d), 4);
  }
  Serial.println();
}

void resetCaptures() {
  g_lower = CaptureStats{};
  g_center = CaptureStats{};
  g_upper = CaptureStats{};
  Serial.println(F("OK,reset"));
}

void handleCommand(const char* line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (*line == '\0') {
    return;
  }

  const char c = line[0];
  switch (c) {
    case 'h':
    case '?':
      printHelp();
      return;
    case 't':
      g_stream_enabled = !g_stream_enabled;
      Serial.print(F("OK,stream="));
      Serial.println(g_stream_enabled ? F("1") : F("0"));
      return;
    case 's':
      printOneSample();
      return;
    case 'L':
    case 'l':
      capture("LOWER", g_lower);
      return;
    case 'C':
    case 'c':
      capture("CENTER", g_center);
      return;
    case 'U':
    case 'u':
      capture("UPPER", g_upper);
      return;
    case 'P':
    case 'p':
      printSummary();
      return;
    case 'R':
    case 'r':
      resetCaptures();
      return;
    default:
      break;
  }

  Serial.print(F("ERR,unknown_command,cmd="));
  Serial.println(c);
  Serial.println(F("INFO,try=h"));
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      g_cmd_buf[g_cmd_len] = '\0';
      handleCommand(g_cmd_buf);
      g_cmd_len = 0;
      continue;
    }
    if (g_cmd_len + 1 < sizeof(g_cmd_buf)) {
      g_cmd_buf[g_cmd_len++] = c;
    }
  }
}

void maybeStream() {
  if (!g_stream_enabled) {
    return;
  }
  const uint32_t now_ms = millis();
  if (static_cast<uint32_t>(now_ms - g_last_stream_ms) < 50u) {  // 20 Hz
    return;
  }
  g_last_stream_ms = now_ms;

  uint16_t raw = 0;
  const bool ok = readRawMedian3(raw);
  Serial.print(F("AS5600_STREAM,ts_ms="));
  Serial.print(now_ms);
  Serial.print(F(",ok="));
  Serial.print(ok ? F("1") : F("0"));
  Serial.print(F(",raw="));
  Serial.print(raw);
  Serial.print(F(",deg="));
  Serial.print(rawToDeg(raw), 4);
  Serial.print(F(",i2c_err="));
  Serial.println(g_i2c_err);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("AS5600_CHECK_BOOT"));

  Wire.begin();
  Wire.setClock(400000);

  // Probe: attempt one read.
  uint16_t raw = 0;
  const bool ok = readRawOnce(raw);
  Serial.print(F("AS5600_PROBE,ok="));
  Serial.print(ok ? F("1") : F("0"));
  Serial.print(F(",raw="));
  Serial.print(raw);
  Serial.print(F(",deg="));
  Serial.print(rawToDeg(raw), 4);
  Serial.print(F(",i2c_err="));
  Serial.println(g_i2c_err);

  printHelp();
}

void loop() {
  pollSerial();
  maybeStream();
}

