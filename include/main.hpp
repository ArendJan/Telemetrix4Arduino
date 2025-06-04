#pragma once

#include <Arduino.h>
#include "boards/esp32devkit.hpp"
#include "boards/nanoatmega.hpp"
#include "boards/stm32blackpill.hpp"
#include "boards/itsybitsy_m4.hpp"
void get_unique_id();

// Old code
extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void pwm_write();

extern void modify_reporting();

extern void get_firmware_version();

extern void are_you_there();

extern void servo_attach();

extern void servo_write();

extern void servo_detach();

extern void sonar_new();

extern void dht_new();

extern void encoder_new();

extern void stop_all_reports();

extern void set_analog_scanning_interval();

extern void enable_all_reports();

extern void reset_data();

extern void init_pin_structures();

void ping();

void feature_detection();

enum PIN_MODES : uint8_t {
  NOT_SET = 255,
  INPUT_MODE = 0,
  OUTPUT_MODE = 1,
  PWM = 2,
  INPUT_PULL_UP = 3,
  INPUT_PULL_DOWN = 4,
  ANALOG_INPUT = 5,
  SONAR_MODE = 7,
  DHT_MODE = 8
};
