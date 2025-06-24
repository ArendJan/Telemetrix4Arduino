#pragma once

#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
// This board does not have a normal list of analog pins
// this shifts the analog pins
#define A1 2047
#define A2 2047
#define A8 2047
#define A9 2047

#endif