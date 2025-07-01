#pragma once

#include <Arduino.h>

#if defined(ARDUINO_ARCH_SAMD) && defined(ARDUINO_ITSYBITSY_M4)
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

const auto MAX_SERVOS = 0;
void hw_init() {
  analogWriteResolution(8);
  analogReadResolution(10);
}
#endif