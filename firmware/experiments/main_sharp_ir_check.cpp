// Sharp GP2Y0A21YK0F standalone check with optional Timer1 stress.
//
// Wiring: Vo (yellow) -> A0, VCC (red) -> 5V, GND (black) -> GND
//
// Stream output:
//   SHARP_IR,raw_adc=<val>,voltage=<V>,distance_cm=<d>
//
// Commands:
//   h   help
//   s   single sample
//   t   toggle continuous stream
//   T   toggle Timer1 40 kHz stress ISR
//   R   reset counters
//   C   print counters

#include <Arduino.h>
#include <util/atomic.h>

namespace {

constexpr uint8_t PIN_SHARP_IR = A0;
constexpr uint32_t SAMPLE_PERIOD_MS = 40UL;
constexpr uint32_t SERIAL_BAUD_RATE = 115200UL;

bool g_stream = true;
bool g_timer1_stress = false;
uint32_t g_sample_count = 0;
volatile uint32_t g_timer1_ticks = 0;
char g_cmd_buf[32];
size_t g_cmd_len = 0;

void startTimer1Stress() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 49;  // 16 MHz / 8 / (49 + 1) = 40 kHz
    TCCR1B = (1 << WGM12) | (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
  }
}

void stopTimer1Stress() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TIMSK1 &= ~(1 << OCIE1A);
    TCCR1B = 0;
  }
}

void printHelp() {
  Serial.println(F("SHARP_IR_CHECK_HELP"));
  Serial.println(F("Commands:"));
  Serial.println(F("  h  help"));
  Serial.println(F("  s  single sample"));
  Serial.println(F("  t  toggle stream"));
  Serial.println(F("  T  toggle Timer1 40kHz stress"));
  Serial.println(F("  R  reset counters"));
  Serial.println(F("  C  print counters"));
}

void printCounters() {
  Serial.print(F("SHARP_COUNTERS,samples="));
  Serial.print(g_sample_count);
  Serial.print(F(",timer1="));
  Serial.print(g_timer1_stress ? F("1") : F("0"));
  Serial.print(F(",timer1_ticks="));
  Serial.println(g_timer1_ticks);
}

void resetCounters() {
  g_sample_count = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_timer1_ticks = 0;
  }
  Serial.println(F("OK,counters_reset"));
}

void printSample() {
  const uint16_t adc_raw = static_cast<uint16_t>(analogRead(PIN_SHARP_IR));
  const float voltage = static_cast<float>(adc_raw) * (5.0f / 1024.0f);

  float distance_cm = 0.0f;
  if (adc_raw >= 10 && voltage >= 0.05f) {
    distance_cm = 29.988f * powf(voltage, -1.173f);
  }

  ++g_sample_count;
  Serial.print(F("SHARP_IR,raw_adc="));
  Serial.print(adc_raw);
  Serial.print(F(",voltage="));
  Serial.print(voltage, 3);
  Serial.print(F(",distance_cm="));
  Serial.println(distance_cm, 1);
}

void handleCommand(const char* line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (*line == '\0') {
    return;
  }

  switch (line[0]) {
    case 'h':
    case '?':
      printHelp();
      return;
    case 's':
      printSample();
      return;
    case 't':
      g_stream = !g_stream;
      Serial.print(F("OK,stream="));
      Serial.println(g_stream ? F("1") : F("0"));
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
      Serial.print(F("ERR,unknown_command,cmd="));
      Serial.println(line[0]);
      Serial.println(F("INFO,try=h"));
      return;
  }
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

ISR(TIMER1_COMPA_vect) {
  ++g_timer1_ticks;
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(200);
  Serial.println(F("SHARP_IR_CHECK_BOOT"));
  Serial.println(F("Wiring: Vo(yellow)->A0  VCC(red)->5V  GND(black)->GND"));
  Serial.println(F("Move target 10-80 cm from sensor..."));
  Serial.println(F("==========================================="));
  pinMode(PIN_SHARP_IR, INPUT);
  printHelp();
}

void loop() {
  static uint32_t last_ms = 0;
  const uint32_t now_ms = millis();

  pollSerial();

  if (!g_stream) {
    return;
  }
  if (static_cast<uint32_t>(now_ms - last_ms) < SAMPLE_PERIOD_MS) {
    return;
  }

  last_ms = now_ms;
  printSample();
}
