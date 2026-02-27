#pragma once

#include <Arduino.h>

namespace bb {

constexpr uint8_t PIN_STEP = 2;
constexpr uint8_t PIN_DIR = 3;
constexpr uint8_t PIN_EN = 4;

constexpr uint8_t PIN_TRIG = 8;
constexpr uint8_t PIN_ECHO = 9;
constexpr uint8_t PIN_TFMINI_RX = 10;
constexpr uint8_t PIN_TFMINI_TX = 11;

constexpr uint8_t PIN_I2C_SDA = A4;
constexpr uint8_t PIN_I2C_SCL = A5;

constexpr uint8_t AS5600_I2C_ADDR = 0x36;

}  // namespace bb
