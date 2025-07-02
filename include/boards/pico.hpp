#pragma once
#include <Arduino.h>

// Maybe some more are required, for pico2 and pico1w
#if defined(RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W) || \
    defined(ARDUINO_ARCH_RP2040)

#include <Wire.h>
#include <hardware/i2c.h>

const auto A4 = 2047;
const auto A5 = 2047;
const auto A6 = 2047;
const auto A7 = 2047;
const auto A8 = 2047;
const auto A9 = 2047;
const auto A10 = 2047;
const auto A11 = 2047;
const auto A12 = 2047;
const auto A13 = 2047;
const auto A14 = 2047;
const auto A15 = 2047;
const auto A16 = 2047;
const auto A17 = 2047;
const auto A18 = 2047;
const auto A19 = 2047;
#define SECOND_I2C_PORT 1
// Change the pins to match SDA and SCL for your board
#define SECOND_I2C_PORT_SDA 10
#define SECOND_I2C_PORT_SCL 11
#if !defined(ARDUINO_RASPBERRY_PI_PICO_2W)
extern TwoWire Wire2;

#define MAX_SERVOS 12 // according to the servo lib
#else
#define MAX_SERVOS 0
extern TwoWire Wire2; // Use GPIO 10 and 11 for I2C on Pico 2W
#endif
#define I2C_COUNT 2

void hw_init();
#endif