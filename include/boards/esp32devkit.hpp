#pragma once

#if defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#include <Wire.h>
// This board does not have a normal list of analog pins
#define A1 2047
#define A2 2047
#define A8 2047
#define A9 2047
void hw_init();

#define SECOND_I2C_PORT 1
#define SECOND_I2C_PORT_SDA 16
#define SECOND_I2C_PORT_SCL 17
extern TwoWire Wire2; // Use GPIO 16 and 17 for I2C on ESP32
#endif