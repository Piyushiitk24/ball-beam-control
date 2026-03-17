#pragma once

#include <Arduino.h>

class HCSR04 {
 public:
  HCSR04(uint8_t trig_pin, uint8_t echo_pin)
      : trig_pin_(trig_pin), echo_pin_(echo_pin), initialized_(false) {}

  double dist() {
    beginIfNeeded();

    digitalWrite(trig_pin_, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin_, LOW);

    const unsigned long pulse_width_us = pulseIn(echo_pin_, HIGH, 25000UL);
    if (pulse_width_us == 0UL) {
      return 0.0;
    }

    return static_cast<double>(pulse_width_us) * 0.0343 * 0.5;
  }

 private:
  void beginIfNeeded() {
    if (initialized_) {
      return;
    }
    pinMode(trig_pin_, OUTPUT);
    pinMode(echo_pin_, INPUT);
    digitalWrite(trig_pin_, LOW);
    delayMicroseconds(5);
    initialized_ = true;
  }

  uint8_t trig_pin_;
  uint8_t echo_pin_;
  bool initialized_;
};
