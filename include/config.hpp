#pragma once
#include "boards/esp32devkit.hpp"
#include "boards/itsybitsy_m4.hpp"
#include "boards/nanoatmega.hpp"
#include "boards/pico.hpp"
#include "boards/stm32blackpill.hpp"
#ifndef MAX_SONARS
#define MAX_SONARS 6
#endif

#ifndef ENABLE_ADAFRUIT_WATCHDOG
#define ENABLE_ADAFRUIT_WATCHDOG 1
#endif