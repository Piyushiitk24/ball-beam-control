// HC-SR04 Sonar Check — standalone diagnostic firmware.
//
// Two modes:
//   Mode A (default) — PCINT edge capture (same ISR path as main firmware).
//   Mode B ('b')     — pulseIn() polling (blocking, like Arduino examples).
//
// Timer1 stress test: 'T' toggles a dummy Timer1 CTC ISR at 40 kHz to replicate
// the step-pulse interrupt load of the real stepper driver.
//
// Commands:
//   h   help
//   s   single shot (current mode)
//   t   toggle continuous stream (10 Hz)
//   a   switch to Mode A (PCINT)
//   b   switch to Mode B (pulseIn)
//   T   toggle Timer1 40 kHz stress ISR
//   R   reset counters

#include <Arduino.h>
#include <util/atomic.h>

#include "pins.h"

namespace {

// ----- Sonar timing constants (same as main firmware) -----
constexpr uint32_t kTriggerPeriodUs = 40000UL;
constexpr uint32_t kEchoTimeoutUs   = 25000UL;
constexpr float    kUsPerCmOneWay   = 58.0f;  // ~1/0.0343/2

// ----- Mode selection -----
enum class Mode : uint8_t { PCINT_ISR = 0, PULSE_IN = 1 };
Mode g_mode = Mode::PCINT_ISR;

// ----- Streaming -----
bool g_stream = false;

// ----- Counters -----
uint32_t g_ping_count    = 0;
uint32_t g_valid_count   = 0;
uint32_t g_timeout_count = 0;
uint32_t g_miss_count    = 0;  // echo armed but no fall before timeout

// ----- PCINT state (ISR-shared) -----
volatile bool     g_echo_armed   = false;
volatile bool     g_awaiting_fall = false;
volatile bool     g_sample_ready = false;
volatile uint32_t g_rise_us      = 0;
volatile uint32_t g_pulse_us     = 0;

// ----- Timer1 stress -----
bool     g_timer1_stress = false;
volatile uint32_t g_timer1_ticks = 0;

// ----- Timing -----
uint32_t g_last_trigger_us = 0;
bool     g_waiting_echo    = false;

// ----- Serial command buffer -----
char   g_cmd_buf[32];
size_t g_cmd_len = 0;

// ----- Pin register for fast PCINT reads -----
volatile uint8_t* g_echo_pin_reg  = nullptr;
uint8_t           g_echo_pin_mask = 0;

// ========== Timer1 40 kHz stress ISR ==========
void startTimer1Stress() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A  = 49;               // 16 MHz / 8 / (49+1) = 40 kHz
    TCCR1B = (1 << WGM12) | (1 << CS11);  // CTC, prescaler 8
    TIMSK1 |= (1 << OCIE1A);
  }
}

void stopTimer1Stress() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TIMSK1 &= ~(1 << OCIE1A);
    TCCR1B = 0;
  }
}

// ========== Trigger pulse ==========
void fireTrigger() {
  // Arm PCINT echo capture before trigger.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_echo_armed    = true;
    g_awaiting_fall = false;
    g_sample_ready  = false;
  }

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  g_last_trigger_us = micros();
  g_waiting_echo    = true;
  ++g_ping_count;
}

// ========== PCINT-based measurement ==========
float measurePcint(bool& valid) {
  valid = false;

  uint32_t pw = 0;
  bool ready = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (g_sample_ready) {
      g_sample_ready = false;
      pw    = g_pulse_us;
      ready = true;
    }
  }

  if (!ready) {
    return 0.0f;
  }

  g_waiting_echo = false;

  if (pw == 0 || pw >= kEchoTimeoutUs) {
    ++g_timeout_count;
    return 0.0f;
  }

  valid = true;
  ++g_valid_count;
  return static_cast<float>(pw) / kUsPerCmOneWay;
}

// ========== pulseIn-based measurement ==========
float measurePulseIn(bool& valid) {
  valid = false;

  // Disable PCINT to avoid conflict.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_echo_armed = false;
  }

  // Send trigger.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  ++g_ping_count;

  const unsigned long pw = pulseIn(PIN_ECHO, HIGH, kEchoTimeoutUs);
  if (pw == 0) {
    ++g_timeout_count;
    return 0.0f;
  }

  valid = true;
  ++g_valid_count;
  return static_cast<float>(pw) / kUsPerCmOneWay;
}

// ========== Output helpers ==========
void printResult(uint32_t ts, bool valid, float cm, uint32_t pulse_us) {
  Serial.print(F("SONAR_CHK,ts_ms="));
  Serial.print(ts);
  Serial.print(F(",mode="));
  Serial.print(g_mode == Mode::PCINT_ISR ? F("PCINT") : F("PULSEIN"));
  Serial.print(F(",ok="));
  Serial.print(valid ? F("1") : F("0"));
  Serial.print(F(",cm="));
  Serial.print(cm, 2);
  Serial.print(F(",pulse_us="));
  Serial.print(pulse_us);
  Serial.print(F(",timer1="));
  Serial.print(g_timer1_stress ? F("1") : F("0"));
  Serial.print(F(",pings="));
  Serial.print(g_ping_count);
  Serial.print(F(",valid="));
  Serial.print(g_valid_count);
  Serial.print(F(",timeout="));
  Serial.print(g_timeout_count);
  Serial.print(F(",miss="));
  Serial.println(g_miss_count);
}

void printHelp() {
  Serial.println(F("HCSR04_CHECK_HELP"));
  Serial.println(F("Commands:"));
  Serial.println(F("  h  help"));
  Serial.println(F("  s  single shot"));
  Serial.println(F("  t  toggle stream (10 Hz)"));
  Serial.println(F("  a  Mode A: PCINT ISR (default)"));
  Serial.println(F("  b  Mode B: pulseIn (blocking)"));
  Serial.println(F("  T  toggle Timer1 40kHz stress"));
  Serial.println(F("  R  reset counters"));
}

void printCounters() {
  Serial.print(F("SONAR_COUNTERS,pings="));
  Serial.print(g_ping_count);
  Serial.print(F(",valid="));
  Serial.print(g_valid_count);
  Serial.print(F(",timeout="));
  Serial.print(g_timeout_count);
  Serial.print(F(",miss="));
  Serial.print(g_miss_count);
  Serial.print(F(",timer1_ticks="));
  Serial.println(g_timer1_ticks);
}

void resetCounters() {
  g_ping_count    = 0;
  g_valid_count   = 0;
  g_timeout_count = 0;
  g_miss_count    = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { g_timer1_ticks = 0; }
  Serial.println(F("OK,counters_reset"));
}

// ========== Single shot (works in both modes) ==========
void doSingleShot() {
  bool valid = false;
  float cm = 0.0f;
  uint32_t pw = 0;

  if (g_mode == Mode::PULSE_IN) {
    cm = measurePulseIn(valid);
  } else {
    // For PCINT mode: fire trigger, then poll until sample or timeout.
    fireTrigger();
    const uint32_t deadline = micros() + kEchoTimeoutUs + 2000UL;
    while (!g_sample_ready && static_cast<int32_t>(micros() - deadline) < 0) {
      // spin
    }
    cm = measurePcint(valid);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pw = g_pulse_us; }
  }

  printResult(millis(), valid, cm, pw);
}

// ========== Command handler ==========
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
    case 's':
      doSingleShot();
      return;
    case 't':
      g_stream = !g_stream;
      Serial.print(F("OK,stream="));
      Serial.println(g_stream ? F("1") : F("0"));
      return;
    case 'a':
      g_mode = Mode::PCINT_ISR;
      Serial.println(F("OK,mode=PCINT"));
      return;
    case 'b':
      g_mode = Mode::PULSE_IN;
      Serial.println(F("OK,mode=PULSEIN"));
      return;
    case 'T':
      g_timer1_stress = !g_timer1_stress;
      if (g_timer1_stress) {
        startTimer1Stress();
        Serial.println(F("OK,timer1_stress=ON"));
      } else {
        stopTimer1Stress();
        Serial.println(F("OK,timer1_stress=OFF"));
      }
      return;
    case 'R':
      resetCounters();
      return;
    case 'C':
      printCounters();
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

}  // namespace

// ========== PCINT ISR (echo pin on D9 = PCINT1 / PB1) ==========
ISR(PCINT0_vect) {
  const uint32_t now_us = micros();
  if (!g_echo_armed) {
    return;
  }

  const bool high = (*g_echo_pin_reg & g_echo_pin_mask) != 0;
  if (high) {
    g_rise_us       = now_us;
    g_awaiting_fall = true;
    return;
  }

  if (!g_awaiting_fall) {
    return;
  }

  g_pulse_us      = static_cast<uint32_t>(now_us - g_rise_us);
  g_sample_ready  = true;
  g_awaiting_fall = false;
  g_echo_armed    = false;
}

// ========== Timer1 stress ISR ==========
ISR(TIMER1_COMPA_vect) {
  ++g_timer1_ticks;
  // Minimal work — just increment. Simulates the overhead of the real stepper ISR.
}

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("HCSR04_CHECK_BOOT"));

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // Set up pin register for fast ISR reads (D9 = PB1).
  g_echo_pin_reg  = &PINB;
  g_echo_pin_mask = (1 << PB1);

  // Enable PCINT for echo pin (D9 = PCINT1, in PCINT0 group).
  PCMSK0 |= (1 << PCINT1);
  PCICR  |= (1 << PCIE0);

  // Quick boot self-test: one ping in each mode.
  Serial.println(F("BOOT_TEST,mode=PULSEIN"));
  g_mode = Mode::PULSE_IN;
  doSingleShot();
  delay(60);

  Serial.println(F("BOOT_TEST,mode=PCINT"));
  g_mode = Mode::PCINT_ISR;
  doSingleShot();
  delay(60);

  resetCounters();
  Serial.println(F("BOOT,ready"));
  printHelp();
}

void loop() {
  pollSerial();

  if (!g_stream) {
    return;
  }

  // Stream mode: PCINT continuous at ~25 Hz trigger rate.
  if (g_mode == Mode::PCINT_ISR) {
    const uint32_t now_us = micros();

    // Check for missed echoes (armed but timed out).
    if (g_waiting_echo && static_cast<uint32_t>(now_us - g_last_trigger_us) > kEchoTimeoutUs) {
      g_waiting_echo = false;
      ++g_miss_count;
      ++g_timeout_count;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_echo_armed    = false;
        g_awaiting_fall = false;
        g_sample_ready  = false;
      }
    }

    // Fire new trigger if interval elapsed.
    if (!g_waiting_echo && static_cast<uint32_t>(now_us - g_last_trigger_us) >= kTriggerPeriodUs) {
      fireTrigger();
    }

    // Check for completed sample.
    bool valid = false;
    const float cm = measurePcint(valid);
    if (valid || g_sample_ready) {
      uint32_t pw = 0;
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pw = g_pulse_us; }
      printResult(millis(), valid, cm, pw);
    }
  } else {
    // pulseIn mode: blocking, ~10 Hz.
    bool valid = false;
    const float cm = measurePulseIn(valid);
    printResult(millis(), valid, cm, 0);
    delay(60);
  }
}
