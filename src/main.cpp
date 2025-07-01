// #include "Telemetrix4Arduino.h"
// #include <Cpp_Standard_Library.h>
#include "main.hpp"
#include "Telemetrix4Arduino.h"
#include "commands.hpp"
#include "config.hpp"
#include "i2c.hpp"
#include <Arduino.h>
#include <NewPing.h>
#include <OpticalEncoder.h>

#if MAX_SERVOS > 0
#include <Servo.h>
#endif
#include <Wire.h>
#include <dhtnew.h>
// #include <vector>
/*
  Copyright (c) 2020 Alan Yorinks All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
  Version 3 as published by the Free Software Foundation; either
  or (at your option) any later version.
  This library is distributed in the hope that it will be useful,f
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSEf
  along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// We define these here to provide a forward reference.
// If you add a new command, you must add the command handler
// here as well.

// #include <array>
template <size_t N> void send_message(const uint8_t (&message)[N]);
// uncomment out the next line to create a 2nd i2c port
//#define SECOND_I2C_PORT

#ifdef SECOND_I2C_PORT
// Change the pins to match SDA and SCL for your board
#define SECOND_I2C_PORT_SDA PB11
#define SECOND_I2C_PORT_SCL PB10

TwoWire Wire2(SECOND_I2C_PORT_SDA, SECOND_I2C_PORT_SCL);
#endif

// This value must be the same as specified when instantiating the
// telemetrix client. The client defaults to a value of 1.
// This value is used for the client to auto-discover and to
// connect to a specific board regardless of the current com port
// it is currently connected to.
#define ARDUINO_ID 1

// When adding a new command update the command_table.
// The command length is the number of bytes that follow
// the command byte itself, and does not include the command
// byte in its length.
// The command_func is a pointer the command's function.
// using command_descriptor = void *(void);
typedef void (*command_descriptor)();
// not implemented functions
// auto pwm_write = nullptr;
// auto get_unique_id = nullptr;
auto reset_board = nullptr;
auto init_neo_pixels = nullptr;
auto show_neo_pixels = nullptr;
auto set_neo_pixel = nullptr;
auto clear_all_neo_pixels = nullptr;
auto fill_neo_pixels = nullptr;
auto init_spi = nullptr;
auto write_blocking_spi = nullptr;
auto read_blocking_spi = nullptr;
auto set_format_spi = nullptr;
auto spi_cs_control = nullptr;
auto set_scan_delay = nullptr;
auto sensor_new = nullptr;
// auto ping = nullptr;
auto module_new = nullptr;
auto module_data = nullptr;
auto get_id = nullptr;
auto set_id = nullptr;
// If you add new commands, make sure to extend the siz of this
// array.
constexpr command_descriptor command_table[] = {
    &serial_loopback,      // 0
    &set_pin_mode,         // 1
    &digital_write,        // 2, checked
    pwm_write,             // 3
    &modify_reporting,     // 4, checked
    &get_firmware_version, // 5, checked
    &get_unique_id,        // 6, checked
    &servo_attach,         // 7
    &servo_write,          // 8
    &servo_detach,         // 9
    &i2c_begin,
    &i2c_read,
    &i2c_write,
    &sonar_new, // 13, checked
    &dht_new,
    &stop_all_reports,   // 15, checked
    &enable_all_reports, // 16, checked
    &reset_data,
    reset_board,
    init_neo_pixels,
    show_neo_pixels,
    set_neo_pixel,
    clear_all_neo_pixels,
    fill_neo_pixels,
    init_spi,
    write_blocking_spi,
    read_blocking_spi,
    set_format_spi,
    spi_cs_control,
    set_scan_delay,
    &encoder_new, // 30, checked
    sensor_new,
    ping, // 32,  checked, not impelemented
    module_new,
    module_data,
    get_id,
    set_id,
    &feature_detection};
const size_t command_table_size =
    sizeof(command_table) / sizeof(command_descriptor);
// Input pin reporting control sub commands (modify_reporting)
#define REPORTING_DISABLE_ALL 0
#define REPORTING_ANALOG_ENABLE 1
#define REPORTING_DIGITAL_ENABLE 2
#define REPORTING_ANALOG_DISABLE 3
#define REPORTING_DIGITAL_DISABLE 4

// Pin mode definitions

// INPUT defined in Arduino.h = 0
// OUTPUT defined in Arduino.h = 1
// INPUT_PULLUP defined in Arduino.h = 2
// The following are defined for arduino_telemetrix (AT)
// #define AT_ANALOG 3
// #define NOT_SET 255

// Reports - sent from this sketch
// #define DIGITAL_REPORT DIGITAL_WRITE
// #define ANALOG_REPORT ANALOG_WRITE
// #define FIRMWARE_REPORT 5
// #define I_AM_HERE 6
// #define SERVO_UNAVAILABLE 7
// #define I2C_TOO_FEW_BYTES_RCVD 8
// #define I2C_TOO_MANY_BYTES_RCVD 9
// #define I2C_READ_REPORT 10
// #define SONAR_DISTANCE 11
// #define DHT_REPORT 12
// #define DEBUG_PRINT 99

// #define ENCODER_REPORT 13

// DHT Report sub-types
#define DHT_DATA 0
#define DHT_READ_ERROR 1

// firmware version - update this when bumping the version
#define FIRMWARE_MAJOR 1
#define FIRMWARE_MINOR 11

// A buffer to hold i2c report data
byte i2c_report_message[64];

bool stop_reports = false; // a flag to stop sending all report messages

// Analog input pin numbers are defined from
// A0 - A7. Since we do not know if the board
// in use also supports higher analog pin numbers
// we need to define those pin numbers to allow
// the program to compile, even though the
// pins may not exist for the board in use.
// The boards header file defines the analog pins that are not available
// To translate a pin number from an integer value to its analog pin number
// equivalent, this array is used to look up the value to use for the pin.
constexpr int analog_read_pins[20] = {A0,  A1,  A2,  A3,  A4,  A5,  A6,
                                      A7,  A8,  A9,  A10, A11, A12, A13,
                                      A14, A15, A16, A17, A18, A19};
constexpr int analog_read_pins_size =
    sizeof(analog_read_pins) / sizeof(analog_read_pins[0]);
constexpr int ANALOG_PIN_OFFSET = A0;
constexpr int get_total_pins_not(const int *pins, int size, int not_value) {
  return size > 0 ? (pins[size - 1] != not_value
                         ? get_total_pins_not(pins, size - 1, not_value) + 1
                         : get_total_pins_not(pins, size - 1, not_value))
                  : 0;
}
// constexpr int get_total_pins_c(const int *pins, int size, int not_value, int
// c) {
//   return c<size ? (pins[c] !=not_value ? get_total_pins_c(pins, size,
//   not_value, c+1) + c : 0) : 0;
// }

constexpr auto MAX_ANALOG_PINS_SUPPORTED =
    get_total_pins_not(analog_read_pins, analog_read_pins_size, 2047);

// maximum number of pins supported
constexpr auto MAX_PINS_SUPPORTED =
    NUM_DIGITAL_PINS +
    MAX_ANALOG_PINS_SUPPORTED; // probably too high but good enough
// #define
// a descriptor for digital pins
struct pin_descriptor {
  byte pin_number;
  PIN_MODES pin_mode;
  bool digital_reporting_enabled; // If true, then send reports if an input pin
  bool analog_reporting_enabled;  // If true, then send reports if an input pin
  int last_value;                 // Last value read for input mode
  int differential; // difference between current and last value needed
  // to generate a report
};

// an array of digital_pin_descriptors
pin_descriptor the_digital_pins[MAX_PINS_SUPPORTED];

// a descriptor for digital pins
// struct analog_pin_descriptor {
//   byte pin_number;
//   PIN_MODES pin_mode;
//   bool reporting_enabled; // If true, then send reports if an input pin
//   int last_value;         // Last value read for input mode
// };

// // an array of analog_pin_descriptors
// analog_pin_descriptor the_analog_pins[40];

unsigned long current_millis;  // for analog input loop
unsigned long previous_millis; // for analog input loop
uint8_t analog_sampling_interval = 19;

#if MAX_SERVOS > 0
// servo management
Servo servos[MAX_SERVOS]; // max set by servo library
// this array allows us to retrieve the servo object
// associated with a specific pin number
byte servo_index_to_pin_map[MAX_SERVOS];
#endif
// HC-SR04 Sonar Management

struct Sonar {
  uint8_t trigger_pin;
  unsigned int last_value;
  NewPing *usonic;
};

// an array of sonar objects
Sonar sonars[MAX_SONARS];

byte sonars_index = 0; // index into sonars struct

// used for scanning the sonar devices.
byte last_sonar_visited = 0;

unsigned long sonar_current_millis;  // for analog input loop
unsigned long sonar_previous_millis; // for analog input loop
uint8_t sonar_scan_interval = 33;    // Milliseconds between sensor pings
// (29ms is about the min to avoid = 19;

// DHT Management
#define MAX_DHTS 6               // max number of devices
#define READ_FAILED_IN_SCANNER 0 // read request failed when scanning
#define READ_IN_FAILED_IN_SETUP                                                \
  1 // read request failed when initially setting up

struct DHT {
  uint8_t pin;
  unsigned int last_value;
  DHTNEW *dht_sensor;
};

// an array of dht objects]
DHT dhts[MAX_DHTS];

byte dht_index = 0; // index into dht struct

unsigned long dht_current_millis;      // for analog input loop
unsigned long dht_previous_millis;     // for analog input loop
unsigned int dht_scan_interval = 2000; // scan dht's every 2 seconds

// Optical encoder handling
struct OptEncoder {
  uint8_t pin;
  long last_value;
  OpticalEncoder *optEnc_sensor;
};

#define MAX_ENCODERS 2

OptEncoder optEnc[MAX_ENCODERS];
uint8_t optEncoder_ix = 0; // number of Optical Encoders attached

typedef void (*intCB)(void); // lambda definition for interrupt callback

intCB interruptMap[MAX_ENCODERS] = {
    [] { optEnc[0].optEnc_sensor->handleInterrupt(); },
    [] { optEnc[1].optEnc_sensor->handleInterrupt(); },
    // [] { optEnc[2].optEnc_sensor->handleInterrupt(); },
    // [] { optEnc[3].optEnc_sensor->handleInterrupt(); }
};

unsigned long optenc_current_millis;   // for analog input loop
unsigned long optenc_previous_millis;  // for analog input loop
unsigned int optenc_scan_interval = 0; // scan encoders every x ms

// buffer to hold incoming command data
uint8_t command_buffer[MAX_COMMAND_LENGTH];

// A method to send debug data across the serial link
void send_debug_info(byte id, int value) {
  byte debug_buffer[] = {(byte)DEBUG_PRINT, 0, 0, 0};
  debug_buffer[1] = id;
  debug_buffer[2] = highByte(value);
  debug_buffer[3] = lowByte(value);
  // while(Serial.availableForWrite() < 5) {
  //   delay(1);
  // }
  // Serial.write(debug_buffer, 5);
  send_message(debug_buffer);
}

// command functions
void serial_loopback() {
  byte loop_back_buffer[] = {(byte)SERIAL_LOOP_BACK, command_buffer[0]};
  // Serial.write(loop_back_buffer, 3);
  send_message(loop_back_buffer);
}

void set_pin_mode() {
  byte pin;
  PIN_MODES mode;
  pin = command_buffer[0];
  mode = (PIN_MODES)command_buffer[1];
  // Serial2.println("Setting pin mode: " + String(pin) + " to " +
  //  String(mode));
  switch (mode) {
  case INPUT_PULL_DOWN:
    the_digital_pins[pin].pin_mode = mode;
    the_digital_pins[pin].digital_reporting_enabled = command_buffer[2];
    the_digital_pins[pin].last_value = -1;
#ifndef INPUT_PULLDOWN // for boards that do not support INPUT_PULLDOWN, fall
                       // back to INPUT
#define INPUT_PULLDOWN INPUT
#endif
    pinMode(pin, INPUT_PULLDOWN);
    break;
  case INPUT_MODE: //[SET_PIN_MODE = 1, pin, digital_in_type, report_enable]
    the_digital_pins[pin].pin_mode = mode;
    the_digital_pins[pin].digital_reporting_enabled = command_buffer[2];
    the_digital_pins[pin].last_value = -1;
    pinMode(pin, INPUT);
    break;
  case INPUT_PULL_UP:
    the_digital_pins[pin].pin_mode = mode;
    the_digital_pins[pin].digital_reporting_enabled = command_buffer[2];
    the_digital_pins[pin].last_value = -1;
    pinMode(pin, INPUT_PULLUP);
    break;
  case OUTPUT_MODE:
    the_digital_pins[pin].pin_mode = mode;
    pinMode(pin, OUTPUT);
    break;
  case ANALOG_INPUT: // [SET_PIN_MODE = 1, adc_pin, ANALOG_IN = 5, diff_high,
                     // diff_low, report_enable ]
    pinMode(pin, INPUT);
    the_digital_pins[pin].pin_mode = mode;
    the_digital_pins[pin].differential =
        (command_buffer[2] << 8) + command_buffer[3];
    the_digital_pins[pin].analog_reporting_enabled = command_buffer[4];
    the_digital_pins[pin].last_value = -1;
    send_debug_info(pin, the_digital_pins[pin].differential);
    break;
  case PWM:
    pinMode(pin, OUTPUT);
    break;
  default:
    break;
  }
}

void set_analog_scanning_interval() {
  analog_sampling_interval = command_buffer[0];
}

void digital_write() {
  byte pin;
  byte value;
  pin = command_buffer[0];
  value = command_buffer[1];
  digitalWrite(pin, value);
}

void pwm_write() {
  // [PWM_WRITE = 3, pin, value_high, value_low]

  // command_buffer[0] = PIN, command_buffer[1] = value_msb,
  // command_buffer[2] = value_lsb
  byte pin; // command_buffer[0]
  unsigned int value;
  pin = command_buffer[0];

  value = (command_buffer[1] << 8) + command_buffer[2];

  send_debug_info(3, pin);
  send_debug_info(4, value);
  analogWrite(pin, value);
}

void modify_reporting() {
  // [MODIFY_REPORTING = 4, modify_type, pin]

  int pin = command_buffer[1];

  switch (command_buffer[0]) {
  case REPORTING_DISABLE_ALL:
    for (uint8_t i = 0; i < MAX_PINS_SUPPORTED; i++) {
      the_digital_pins[i].digital_reporting_enabled = false;
      the_digital_pins[i].analog_reporting_enabled = false;
    }
    break;
  case REPORTING_ANALOG_ENABLE:
    if (the_digital_pins[pin].pin_mode != NOT_SET) {
      the_digital_pins[pin].analog_reporting_enabled = true;
    }
    break;
  case REPORTING_ANALOG_DISABLE:
    if (the_digital_pins[pin].pin_mode != NOT_SET) {
      the_digital_pins[pin].analog_reporting_enabled = false;
    }
    break;
  case REPORTING_DIGITAL_ENABLE:
    if (the_digital_pins[pin].pin_mode != NOT_SET) {
      the_digital_pins[pin].digital_reporting_enabled = true;
    }
    break;
  case REPORTING_DIGITAL_DISABLE:
    if (the_digital_pins[pin].pin_mode != NOT_SET) {
      the_digital_pins[pin].digital_reporting_enabled = false;
    }
    break;
  default:
    break;
  }
}

void get_firmware_version() {
  byte report_message[] = {FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR};
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // Serial.write(report_message, 4);
  send_message<3>(report_message);
}

// void are_you_there() {
//   byte report_message[3] = {2, I_AM_HERE, ARDUINO_ID};
//   Serial.write(report_message, 3);
// }

/***************************************************
   Servo Commands
 **************************************************/

// Find the first servo that is not attached to a pin
// This is a helper function not called directly via the API
int find_servo() {
#if MAX_SERVOS > 0
  int index = -1;
  for (int i = 0; i < MAX_SERVOS; i++) {
    if (servos[i].attached() == false) {
      index = i;
      break;
    }
  }
  return index;
#else
  return -1; // no servos supported
#endif
}

void servo_attach() {
#if MAX_SERVOS > 0
  byte pin = command_buffer[0];
  int servo_found = -1;

  int minpulse = (command_buffer[1] << 8) + command_buffer[2];
  int maxpulse = (command_buffer[3] << 8) + command_buffer[4];

  // find the first available open servo
  servo_found = find_servo();
  if (servo_found != -1) {
    servo_index_to_pin_map[servo_found] = pin;
    servos[servo_found].attach(pin, minpulse, maxpulse);
  } else {
    // no open servos available, send a report back to client
    byte report_message[] = {SERVO_UNAVAILABLE, pin};
    // Serial.write(report_message, 2);
    send_message(report_message);
  }
#endif
}

// set a servo to a given angle
void servo_write() {
#if MAX_SERVOS > 0
  byte pin = command_buffer[0];
  int angle = command_buffer[1] << 8 | command_buffer[2];
  // find the servo object for the pin
  for (int i = 0; i < MAX_SERVOS; i++) {
    if (servo_index_to_pin_map[i] == pin) {
      servos[i].write(angle);
      return;
    }
  }
#endif
}

// detach a servo and make it available for future use
void servo_detach() {
#if MAX_SERVOS > 0
  byte pin = command_buffer[0];

  // find the servo object for the pin
  for (int i = 0; i < MAX_SERVOS; i++) {
    if (servo_index_to_pin_map[i] == pin) {

      servo_index_to_pin_map[i] = -1;
      servos[i].detach();
    }
  }
#else
#warning "No servos supported, servos will not do anything"
#endif
}

/***********************************
   HC-SR04 adding a new device
 **********************************/

void sonar_new() {
  send_debug_info(9, sonars_index);
  // [SONAR_NEW = 13, trigger_pin, echo_pin]
  if (sonars_index >= MAX_SONARS) {
    return;
  }

  sonars[sonars_index].usonic =
      new NewPing((uint8_t)command_buffer[0], (uint8_t)command_buffer[1], 400);
  sonars[sonars_index].trigger_pin = command_buffer[0];
  sonars_index++; // next index, so it is the number of sonars
  sonar_scan_interval = 100 / sonars_index; // always scan all sonars in 100ms
}

/***********************************
   DHT adding a new device
 **********************************/

void dht_new() {
  int d_read;
  // report consists of:
  // 0 - byte count
  // 1 - report type
  // 2 - dht report subtype
  // 3 - pin number
  // 4 - error value

  // pre-build an error report in case of a read error
  byte report_message[] = {(byte)DHT_REPORT, (byte)DHT_READ_ERROR, (byte)0,
                           (byte)0};

  dhts[dht_index].dht_sensor = new DHTNEW((uint8_t)command_buffer[0]);
  dhts[dht_index].dht_sensor->setType();

  dhts[dht_index].pin = command_buffer[0];
  d_read = dhts[dht_index].dht_sensor->read();

  // if read return == zero it means no errors.
  if (d_read == 0) {
    dht_index++;
  } else {
    // error found
    // send report and release the dht object

    report_message[2] = command_buffer[0]; // pin number
    report_message[3] = d_read;
    // Serial.write(report_message, 5);
    send_message(report_message);
    delete (dhts[dht_index].dht_sensor);
  }
}

/***********************************
   Adding a new encoder device
 **********************************/
void encoder_new() {
  // [ENCODER_NEW = 30, encoder_type, pin_A, pin_B]
  // TODO: encoder_type is ignored for now, only single encoder for now.
  // TODO: convert this to a time triggered encoder system.
  // create new encoder object and get interrupt calback method from map
  auto pinA = command_buffer[1];
  optEnc[optEncoder_ix].optEnc_sensor = new OpticalEncoder();
  intCB callbackMethod = interruptMap[optEncoder_ix];
  optEnc[optEncoder_ix].optEnc_sensor->setup(pinA, callbackMethod, 0, 0);
  optEnc[optEncoder_ix].pin = pinA;
  optEncoder_ix++;
}

void stop_all_reports() {
  stop_reports = true;
  // delay(20);
  // Serial.flush();
}

void enable_all_reports() {
  // Serial.flush();
  stop_reports = false;
  // delay(20);
}

void get_next_command() {
  byte command;
  byte packet_length;
  command_descriptor command_entry;

  // clear the command buffer
  memset(command_buffer, 0, sizeof(command_buffer));

  // if there is no command waiting, then return
  if (!Serial.available()) {
    return;
  }
  // get the packet length
  packet_length = (byte)Serial.read();
  // send_debug_info(0, packet_length);
  auto max_wait = 10; // wait for the command to be available
  while (max_wait-- > 0 && !Serial.available()) {
    delayMicroseconds(10);
  }
  if (packet_length == 0) {
    return; // no command to process, reset bytes
  }
  // get the command byte
  command = (byte)Serial.read();

  if (command >= command_table_size) { // discard the command
    for (auto i = 0; i < packet_length - 1; i++) {
      max_wait = 10; // wait for the next byte to be available
      while (max_wait-- > 0 && !Serial.available()) {
        delayMicroseconds(10);
      }
      Serial.read();
    }
    // TODO: send a message to the client that the command is not supported
    return;
  }
  command_entry = command_table[command];
  // Serial2.print("Command: ");
  // Serial2.println(command);
  if (packet_length > 1) {
    // get the data for that command
    for (int i = 0; i < packet_length - 1; i++) {
      max_wait = 10; // wait for the next byte to be available
      // need this delay or data read is not correct
      while (max_wait-- > 0 && !Serial.available()) {
        delayMicroseconds(10);
      }
      command_buffer[i] = (byte)Serial.read();
      // uncomment out to see each of the bytes following the command
      // send_debug_info(i, command_buffer[i]);
    }
  }
  command_entry();
}

void scan_digital_inputs() {
  byte value;

  // report message

  // [DIGITAL_REPORT = 2, pin, value]

  byte report_message[4] = {DIGITAL_REPORT, 0, 0};

  for (uint8_t i = 0; i < MAX_PINS_SUPPORTED; i++) {
    if (the_digital_pins[i].pin_mode == INPUT_MODE ||
        the_digital_pins[i].pin_mode == INPUT_PULL_UP ||
        the_digital_pins[i].pin_mode == INPUT_PULL_DOWN) {
      if (the_digital_pins[i].digital_reporting_enabled) {
        // if the value changed since last read
        value = (byte)digitalRead(the_digital_pins[i].pin_number);
        if (value != the_digital_pins[i].last_value) {
          the_digital_pins[i].last_value = value;
          report_message[1] = (byte)i;
          report_message[2] = value;
          // Serial.write(report_message, 4);
          send_message(report_message);
        }
      }
    }
  }
}

void scan_analog_inputs() {
  int value;

  // report message

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = high order byte of value
  // byte 4 = low order byte of value
  // [ANALOG_REPORT = 3, adc_pin, value_high, value_low]

  byte report_message[] = {ANALOG_REPORT, 0, 0, 0};
  uint8_t adjusted_pin_number;
  int differential;

  current_millis = millis();
  if (current_millis - previous_millis > analog_sampling_interval) {
    previous_millis += analog_sampling_interval;
    for (uint8_t i = 0; i < MAX_PINS_SUPPORTED; i++) {
      if (the_digital_pins[i].pin_mode == ANALOG_INPUT) {
        if (the_digital_pins[i].analog_reporting_enabled) {
          // if the value changed since last read
          // adjust pin number for the actual read
          adjusted_pin_number = i; // (uint8_t)(analog_read_pins[i]);
          value = analogRead(adjusted_pin_number);
          differential = abs(value - the_digital_pins[i].last_value);
          if (differential >= the_digital_pins[i].differential) {
            // send_debug_info(i, differential);
            // trigger value achieved, send out the report
            the_digital_pins[i].last_value = value;
            // input_message[1] = the_analog_pins[i].pin_number;
            report_message[1] = (byte)adjusted_pin_number;
            report_message[2] = highByte(value); // get high order byte
            report_message[3] = lowByte(value);
            // Serial.write(report_message, 5);
            send_message(report_message);
            // Serial2.print("Analog pin: ");
            // Serial2.print(adjusted_pin_number);
            // Serial2.print(" value: ");
            // Serial2.println(value);
            // delay(1);
          }
        }
      }
    }
  }
}

void scan_sonars() {
  unsigned int distance;

  if (sonars_index) {
    sonar_current_millis = millis();
    if (sonar_current_millis - sonar_previous_millis > sonar_scan_interval) {
      // send_debug_info(10, sonar_current_millis);
      sonar_previous_millis += sonar_scan_interval;
      distance = sonars[last_sonar_visited].usonic->ping() / US_ROUNDTRIP_CM;
      if (distance != sonars[last_sonar_visited].last_value || true) {
        sonars[last_sonar_visited].last_value = distance;

        // [SONAR_REPORT = 11, trigger_pin, distance_m, distance_cm]

        byte report_message[] = {
            SONAR_DISTANCE, sonars[last_sonar_visited].trigger_pin,
            (byte)(distance / 100), (byte)(distance % 100)};
        // Serial.write(report_message, 5);
        // send_debug_info(0, distance);
        send_message(report_message);
      }
      last_sonar_visited++;
      if (last_sonar_visited == sonars_index) {
        last_sonar_visited = 0;
      }
    }
  }
}

void scan_dhts() {
  // prebuild report for valid data
  // reuse the report if a read command fails

  // data returned is in floating point form - 4 bytes
  // each for humidity and temperature

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = report sub type - DHT_DATA or DHT_ERROR
  // btye 3 = pin number
  // byte 4 = humidity high order byte for data or error value
  // byte 5 = humidity byte 2
  // byte 6 = humidity byte 3
  // byte 7 = humidity byte 4
  // byte 8 = temperature high order byte for data or
  // byte 9 = temperature byte 2
  // byte 10 = temperature byte 3
  // byte 11 = temperature byte 4
  byte report_message[] = {DHT_REPORT, DHT_DATA, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  byte d_read;

  float dht_data;

  // are there any dhts to read?
  if (dht_index) {
    // is it time to do the read? This should occur every 2 seconds
    dht_current_millis = millis();
    if (dht_current_millis - dht_previous_millis > dht_scan_interval) {
      // update for the next scan
      dht_previous_millis += dht_scan_interval;

      // read and report all the dht sensors
      for (int i = 0; i < dht_index; i++) {
        report_message[2] = dhts[i].pin;
        // get humidity
        dht_data = dhts[i].dht_sensor->getHumidity();
        memcpy(&report_message[3], &dht_data, sizeof dht_data);

        // get temperature
        dht_data = dhts[i].dht_sensor->getTemperature();
        memcpy(&report_message[6], &dht_data, sizeof dht_data);

        // Serial.write(report_message, 12);
        send_message(report_message);

        // now read do a read for this device for next go around
        d_read = dhts[i].dht_sensor->read();

        if (d_read) {
          // error found
          // send report
          // send_debug_info(1, 1);
          // report_message[0] = 4;
          // report_message[1] = DHT_REPORT;
          report_message[1] = DHT_READ_ERROR;
          report_message[2] = dhts[i].pin; // pin number
          report_message[3] = d_read;
          // Serial.write(report_message, 5);
          send_message(report_message);
        }
      }
    }
  }
}

void scan_encoders() {
  // [ENCODER_REPORT = 14, pin_A, steps]

  byte report_message[] = {ENCODER_REPORT, 0, 0};

  long optEnc_return_val = 0;

  if (optEncoder_ix) {

    for (int i = 0; i < optEncoder_ix; ++i) {
      auto enc = optEnc[i].optEnc_sensor;
      optEnc_return_val = enc->getPosition();

      if (optEnc_return_val != 0) {
        enc->resetPosition();
        report_message[1] = optEnc[i].pin;
        report_message[2] = optEnc_return_val & 0xFF;

        // Serial.write(report_message, 4);
        send_message(report_message);
      }
    }
  }
}

void reset_data() {
  // reset the data structures

  // fist stop all reporting
  stop_all_reports();

  current_millis = 0;  // for analog input loop
  previous_millis = 0; // for analog input loop
  analog_sampling_interval = 19;

// detach any attached servos
#if MAX_SERVOS > 0
  for (int i = 0; i < MAX_SERVOS; i++) {
    if (servos[i].attached() == true) {
      servos[i].detach();
    }
  }
#endif
  sonars_index = 0; // reset the index into the sonars array

  sonar_current_millis = 0;  // for analog input loop
  sonar_previous_millis = 0; // for analog input loop
  sonar_scan_interval = 33;  // Milliseconds between sensor pings

  dht_index = 0; // index into dht array

  dht_current_millis = 0;   // for analog input loop
  dht_previous_millis = 0;  // for analog input loop
  dht_scan_interval = 2000; // scan dht's every 2 seconds

  // Reset optical encoder timers and index
  optEncoder_ix = 0;
  optenc_current_millis = 0;  // for analog input loop
  optenc_previous_millis = 0; // for analog input loop
  optenc_scan_interval = 0;   // scan encoders every x ms

  init_pin_structures();

  memset(sonars, 0, sizeof(sonars));
  memset(dhts, 0, sizeof(dhts));
  memset(optEnc, 0, sizeof(optEnc));
  enable_all_reports();
}

void init_pin_structures() {
  // create an array of pin_descriptors for 100 pins
  // establish the digital pin array
  for (byte i = 0; i < MAX_PINS_SUPPORTED; i++) {
    the_digital_pins[i].pin_number = i;
    the_digital_pins[i].pin_mode = NOT_SET;
    the_digital_pins[i].digital_reporting_enabled = false;
    the_digital_pins[i].analog_reporting_enabled = false;
    the_digital_pins[i].last_value = -1;
    the_digital_pins[i].differential = 0; // no differential by default
  }
}
#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  init_pin_structures();
  hw_init();
  for (auto i = 0; i < 0xFF; i++) {
    Serial.write((uint8_t)0);
  }
  // get_firmware_version();
}

void loop() {
  // keep processing incoming commands
  get_next_command();
  static decltype(millis()) last_scan = 0;
  static decltype(millis()) scan_delay = 10;
  if (!stop_reports) { // stop reporting
    if (millis() - last_scan >= (scan_delay)) {
      // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      // Serial.println("Scanning inputs...");
      // send_debug_info(10, 10);
      last_scan += scan_delay;

      scan_digital_inputs();
      scan_analog_inputs();
      scan_sonars();
      scan_dhts();
      scan_encoders();
    }
  }
}
static_assert(command_table[32] == &ping, "command_table[32] must be ping");
static_assert(sizeof(command_buffer) == MAX_COMMAND_LENGTH,
              "command_buffer size must be equal to MAX_COMMAND_LENGTH");
static_assert(command_table[37] == &feature_detection,
              "command_table[37] must be feature_detection");

void feature_detection() {
  // in message: [FEATURE_CHECK = 37, message_type_to_check]
  // out message: [3, FEATURE_CHECK, 0/1]
  uint8_t report_message[6 + analog_read_pins_size] = {
      FEATURE_CHECK, 0, 0, 0, 0, 0, 0};
  // byte report_message[3] = {2, FEATURE_CHECK, 0};
  auto message_type = command_buffer[0];
  report_message[1] = message_type;
  if (command_table_size <= message_type) {
    report_message[2] = 0;
  } else {
    auto cmd = command_table[message_type];
    if (cmd != nullptr) {
      report_message[2] = 1;
      if (cmd == &encoder_new) {
        report_message[3] = MAX_ENCODERS; // encoder
        report_message[4] = 1;            // single encoder
      } else if (cmd == &sonar_new) {
        report_message[3] = MAX_SONARS; // sonar
      } else if (cmd == &set_pin_mode) {
        report_message[3] = NUM_DIGITAL_PINS;
        report_message[4] = MAX_ANALOG_PINS_SUPPORTED;
        // report_message[5] = ADC_RESOLUTION; // ADC resolution
        // report_message[6] = PWM_RESOLUTION; // PWM resolution
        // report_message[5] = ANALOG_PIN_OFFSET;
        for (auto i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
          report_message[6 + i] = (uint8_t)analog_read_pins[i];
        }
      } else if (cmd == &servo_attach) {
        report_message[3] = MAX_SERVOS;
      } else if (cmd == &dht_new) {
        report_message[3] = MAX_DHTS;
      } else if (cmd == &get_firmware_version) {
        report_message[3] = FIRMWARE_MAJOR;
        report_message[4] = FIRMWARE_MINOR;
      } else if (cmd == &get_unique_id) {
        report_message[3] = 0; // TODO: implement
      }
    } else {
      report_message[2] = 0; // command not supported
    }
  }
  send_message(report_message);
}

template <size_t N> void send_message(const uint8_t (&message)[N]) {
  // while (Serial.availableForWrite() < (int)N + 3) {
  //   Serial.println("Waiting for serial write...");
  //   delayMicroseconds(10);
  // }
  Serial.write((uint8_t)N); // send msg len
  for (size_t i = 0; i < N; i++) {
    Serial.write((uint8_t)message[i]); // send msg len
  }
  // Serial.write(message, N); // send message
}

void get_unique_id() {
  // in message: [GET_UNIQUE_ID = 6]
  // out message: [REPORT_UNIQUE_ID = 6, id[0], id[1], id[2],
  // id[3],id[4],id[5],id[6],id[7] ]
  uint8_t report_message[] = {GET_UNIQUE_ID, 0}; // TODO: implement
  send_message(report_message);
}

#if ENABLE_ADAFRUIT_WATCHDOG
#include <Adafruit_SleepyDog.h>
#define ENABLE_WATCHDOG 1
#endif
// TODO: other watchdog implementations

#if ENABLE_WATCHDOG == 0
#warning "Watchdog not enabled, please enable it in the code"
#endif
bool watchdog_enabled = false;
uint32_t last_ping = 0;
void ping() {
  static uint8_t random = -1;
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  auto special_num = command_buffer[0];
  if (!watchdog_enabled) {
#if ENABLE_ADAFRUIT_WATCHDOG

    Watchdog.enable(4000);
#endif
    // watchdog_enable(WATCHDOG_TIME,
    // 1); // Add watchdog requiring trigger every 5s
    watchdog_enabled = true;
    // srand(millis());
    // random = rand() % 100; // create some random number to let computer side
    //                        // know it is the same run
    random = 0x1B;
  }
  uint8_t out[] = {PONG_REPORT, // write type
                   special_num, random, 0, 0, 0, 0};
  // out[0] = out.size() - 1; // dont count the packet length
  // send_debug_info(1, special_num);
  // send_debug_info(2, random);
  // // Serial2.println("Pinging...");

  send_message(out);
  if (true) {
    // watchdog_update();
#if ENABLE_ADAFRUIT_WATCHDOG
    Watchdog.reset();
#endif

    last_ping = millis();
  }
}