#include <Arduino.h>
#include "Telemetrix4Arduino.h"
#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#include <dhtnew.h>
#include <OpticalEncoder.h>
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

extern void serial_loopback();

extern void set_pin_mode();

extern void digital_write();

extern void analog_write();

extern void modify_reporting();

extern void get_firmware_version();

extern void are_you_there();

extern void servo_attach();

extern void servo_write();

extern void servo_detach();

extern void i2c_begin();

extern void i2c_read();

extern void i2c_write();

extern void sonar_new();

extern void dht_new();

extern void encoder_new();

extern void stop_all_reports();

extern void set_analog_scanning_interval();

extern void enable_all_reports();

extern void reset_data();

extern void init_pin_structures();


// uncomment out the next line to create a 2nd i2c port
//#define SECOND_I2C_PORT

#ifdef SECOND_I2C_PORT
// Change the pins to match SDA and SCL for your board
#define SECOND_I2C_PORT_SDA PB11
#define SECOND_I2C_PORT_SCL PB10

TwoWire Wire2(SECOND_I2C_PORT_SDA, SECOND_I2C_PORT_SCL);
#endif

// a pointer to an active TwoWire object
TwoWire *current_i2c_port;


// This value must be the same as specified when instantiating the
// telemetrix client. The client defaults to a value of 1.
// This value is used for the client to auto-discover and to
// connect to a specific board regardless of the current com port
// it is currently connected to.
#define ARDUINO_ID 1

// Commands -received by this sketch
// Add commands retaining the sequential numbering.
// The order of commands here must be maintained in the command_table.
#define SERIAL_LOOP_BACK 0
#define SET_PIN_MODE 1
#define DIGITAL_WRITE 2
#define ANALOG_WRITE 3
#define MODIFY_REPORTING 4 // mode(all, analog, or digital), pin, enable or disable
#define GET_FIRMWARE_VERSION 5
#define ARE_U_THERE 6
#define SERVO_ATTACH 7
#define SERVO_WRITE 8
#define SERVO_DETACH 9
#define I2C_BEGIN 10
#define I2C_READ 11
#define I2C_WRITE 12
#define SONAR_NEW 13
#define DHT_NEW 14
#define STOP_ALL_REPORTS 15
#define SET_ANALOG_SCANNING_INTERVAL 16
#define ENABLE_ALL_REPORTS 17
#define RESET 18

// Custom rotary encoder support
#define ENCODER_NEW 19

// When adding a new command update the command_table.
// The command length is the number of bytes that follow
// the command byte itself, and does not include the command
// byte in its length.
// The command_func is a pointer the command's function.
struct command_descriptor
{
  // a pointer to the command processing function
  void (*command_func)(void);
};

// An array of pointers to the command functions

// If you add new commands, make sure to extend the siz of this
// array.
command_descriptor command_table[20] =
{
  {&serial_loopback},
  {&set_pin_mode},
  {&digital_write},
  {&analog_write},
  {&modify_reporting},
  {&get_firmware_version},
  {&are_you_there},
  {&servo_attach},
  {&servo_write},
  {&servo_detach},
  {&i2c_begin},
  {&i2c_read},
  {&i2c_write},
  {&sonar_new},
  {dht_new},
  {stop_all_reports},
  {set_analog_scanning_interval},
  {enable_all_reports},
  {reset_data},
  {&encoder_new}
};

// Input pin reporting control sub commands (modify_reporting)
#define REPORTING_DISABLE_ALL 0
#define REPORTING_ANALOG_ENABLE 1
#define REPORTING_DIGITAL_ENABLE 2
#define REPORTING_ANALOG_DISABLE 3
#define REPORTING_DIGITAL_DISABLE 4

// maximum length of a command in bytes
#define MAX_COMMAND_LENGTH 30

// Pin mode definitions

// INPUT defined in Arduino.h = 0
// OUTPUT defined in Arduino.h = 1
// INPUT_PULLUP defined in Arduino.h = 2
// The following are defined for arduino_telemetrix (AT)
#define AT_ANALOG 3
#define AT_MODE_NOT_SET 255

// maximum number of pins supported
#define MAX_DIGITAL_PINS_SUPPORTED 100
#define MAX_ANALOG_PINS_SUPPORTED 15

// Reports - sent from this sketch
#define DIGITAL_REPORT DIGITAL_WRITE
#define ANALOG_REPORT ANALOG_WRITE
#define FIRMWARE_REPORT 5
#define I_AM_HERE 6
#define SERVO_UNAVAILABLE 7
#define I2C_TOO_FEW_BYTES_RCVD 8
#define I2C_TOO_MANY_BYTES_RCVD 9
#define I2C_READ_REPORT 10
#define SONAR_DISTANCE 11
#define DHT_REPORT 12
#define DEBUG_PRINT 99

#define ENCODER_REPORT 13

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

#ifndef A8
#define A8 2047
#endif

#ifndef A9
#define A9 2047
#endif

#ifndef A10
#define A10 2047
#endif

#ifndef PIN_A11
#define A11 2047
#endif

#ifndef PIN_A12
#define A12 2047
#endif

#ifndef PIN_A13
#define A13 2047
#endif

#ifndef PIN_A14
#define A14 2047
#endif

#ifndef PIN_A15
#define A15 2047
#endif

// To translate a pin number from an integer value to its analog pin number
// equivalent, this array is used to look up the value to use for the pin.
int analog_read_pins[20] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

// a descriptor for digital pins
struct pin_descriptor
{
  byte pin_number;
  byte pin_mode;
  bool reporting_enabled; // If true, then send reports if an input pin
  int last_value;         // Last value read for input mode
};

// an array of digital_pin_descriptors
pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];

// a descriptor for digital pins
struct analog_pin_descriptor
{
  byte pin_number;
  byte pin_mode;
  bool reporting_enabled; // If true, then send reports if an input pin
  int last_value;         // Last value read for input mode
  int differential;       // difference between current and last value needed
  // to generate a report
};

// an array of analog_pin_descriptors
analog_pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

unsigned long current_millis;  // for analog input loop
unsigned long previous_millis; // for analog input loop
uint8_t analog_sampling_interval = 19;

// servo management
Servo servos[MAX_SERVOS];

// this array allows us to retrieve the servo object
// associated with a specific pin number
byte pin_to_servo_index_map[MAX_SERVOS];

// HC-SR04 Sonar Management
#define MAX_SONARS 6

struct Sonar
{
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
#define MAX_DHTS 6                // max number of devices
#define READ_FAILED_IN_SCANNER 0  // read request failed when scanning
#define READ_IN_FAILED_IN_SETUP 1 // read request failed when initially setting up

struct DHT
{
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
struct OptEncoder
{
  uint8_t pin;
  long last_value;
  OpticalEncoder *optEnc_sensor;
};

#define MAX_ENCODERS 4

OptEncoder optEnc[MAX_ENCODERS];
uint8_t optEncoder_ix = 0 ; // number of Optical Encoders attached

typedef void (*intCB)(void); // lambda definition for interrupt callback

intCB interruptMap[MAX_ENCODERS] = {
  []{optEnc[0].optEnc_sensor->handleInterrupt();},
  []{optEnc[1].optEnc_sensor->handleInterrupt();},
  []{optEnc[2].optEnc_sensor->handleInterrupt();},
  []{optEnc[3].optEnc_sensor->handleInterrupt();}
};


unsigned long optenc_current_millis;      // for analog input loop
unsigned long optenc_previous_millis;     // for analog input loop
unsigned int optenc_scan_interval = 0; // scan encoders every x ms

// buffer to hold incoming command data
byte command_buffer[MAX_COMMAND_LENGTH];

// A method to send debug data across the serial link
void send_debug_info(byte id, int value)
{
  byte debug_buffer[5] = {(byte)4, (byte)DEBUG_PRINT, 0, 0, 0};
  debug_buffer[2] = id;
  debug_buffer[3] = highByte(value);
  debug_buffer[4] = lowByte(value);
  Serial.write(debug_buffer, 5);
}

// command functions
void serial_loopback()
{
  byte loop_back_buffer[3] = {2, (byte)SERIAL_LOOP_BACK, command_buffer[0]};
  Serial.write(loop_back_buffer, 3);
}

void set_pin_mode()
{
  byte pin;
  byte mode;
  pin = command_buffer[0];
  mode = command_buffer[1];

  switch (mode)
  {
    case INPUT:
      the_digital_pins[pin].pin_mode = mode;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      the_digital_pins[pin].last_value = -1;
      pinMode(pin, INPUT);
      break;
    case INPUT_PULLUP:
      the_digital_pins[pin].pin_mode = mode;
      the_digital_pins[pin].reporting_enabled = command_buffer[2];
      the_digital_pins[pin].last_value = -1;
      pinMode(pin, INPUT_PULLUP);
      break;
    case OUTPUT:
      the_digital_pins[pin].pin_mode = mode;
      pinMode(pin, OUTPUT);
      break;
    case AT_ANALOG:
      the_analog_pins[pin].pin_mode = mode;
      the_analog_pins[pin].differential = (command_buffer[2] << 8) + command_buffer[3];
      the_analog_pins[pin].reporting_enabled = command_buffer[4];
      the_analog_pins[pin].last_value = -1;
      break;
    default:
      break;
  }
}

void set_analog_scanning_interval()
{
  analog_sampling_interval = command_buffer[0];
}

void digital_write()
{
  byte pin;
  byte value;
  pin = command_buffer[0];
  value = command_buffer[1];
  digitalWrite(pin, value);
}

void analog_write()
{
  // command_buffer[0] = PIN, command_buffer[1] = value_msb,
  // command_buffer[2] = value_lsb
  byte pin; // command_buffer[0]
  unsigned int value;

  pin = command_buffer[0];

  value = (command_buffer[1] << 8) + command_buffer[2];
  analogWrite(pin, value);
}

void modify_reporting()
{
  int pin = command_buffer[1];

  switch (command_buffer[0])
  {
    case REPORTING_DISABLE_ALL:
      for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++)
      {
        the_digital_pins[i].reporting_enabled = false;
      }
      for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++)
      {
        the_analog_pins[i].reporting_enabled = false;
      }
      break;
    case REPORTING_ANALOG_ENABLE:
      if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET)
      {
        the_analog_pins[pin].reporting_enabled = true;
      }
      break;
    case REPORTING_ANALOG_DISABLE:
      if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET)
      {
        the_analog_pins[pin].reporting_enabled = false;
      }
      break;
    case REPORTING_DIGITAL_ENABLE:
      if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET)
      {
        the_digital_pins[pin].reporting_enabled = true;
      }
      break;
    case REPORTING_DIGITAL_DISABLE:
      if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET)
      {
        the_digital_pins[pin].reporting_enabled = false;
      }
      break;
    default:
      break;
  }
}

void get_firmware_version()
{
  byte report_message[4] = {3, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR};
  Serial.write(report_message, 4);
}

void are_you_there()
{
  byte report_message[3] = {2, I_AM_HERE, ARDUINO_ID};
  Serial.write(report_message, 3);
}

/***************************************************
   Servo Commands
 **************************************************/

// Find the first servo that is not attached to a pin
// This is a helper function not called directly via the API
int find_servo()
{
  int index = -1;
  for (int i = 0; i < MAX_SERVOS; i++)
  {
    if (servos[i].attached() == false)
    {
      index = i;
      break;
    }
  }
  return index;
}

void servo_attach()
{

  byte pin = command_buffer[0];
  int servo_found = -1;

  int minpulse = (command_buffer[1] << 8) + command_buffer[2];
  int maxpulse = (command_buffer[3] << 8) + command_buffer[4];

  // find the first available open servo
  servo_found = find_servo();
  if (servo_found != -1)
  {
    pin_to_servo_index_map[servo_found] = pin;
    servos[servo_found].attach(pin, minpulse, maxpulse);
  }
  else
  {
    // no open servos available, send a report back to client
    byte report_message[2] = {SERVO_UNAVAILABLE, pin};
    Serial.write(report_message, 2);
  }
}

// set a servo to a given angle
void servo_write()
{
    byte pin = command_buffer[0];
    int angle = command_buffer[1];
    // find the servo object for the pin
    for (int i = 0; i < MAX_SERVOS; i++)
    {
        if (pin_to_servo_index_map[i] == pin)
        {

            servos[i].write(angle);
            return;
        }
    }
}

// detach a servo and make it available for future use
void servo_detach()
{
  byte pin = command_buffer[0];

  // find the servo object for the pin
  for (int i = 0; i < MAX_SERVOS; i++)
  {
    if (pin_to_servo_index_map[i] == pin)
    {

      pin_to_servo_index_map[i] = -1;
      servos[i].detach();
    }
  }
}

/***********************************
   i2c functions
 **********************************/

void i2c_begin()
{
  byte i2c_port = command_buffer[0];
  if (! i2c_port)
  {
    Wire.begin();
#if defined(__AVR_ATmega328P__) 
    Wire.setWireTimeout(10,false);
    Wire.clearWireTimeoutFlag();
#endif
  }

#ifdef SECOND_I2C_PORT
  else
  {
    Wire2.begin();
#if defined(__AVR_ATmega328P__) 
    Wire2.setWireTimeout(10,false);
    Wire2.clearWireTimeoutFlag();
#endif
  }
#endif
}

void i2c_read()
{
  // data in the incoming message:
  // address, [0]
  // register, [1]
  // number of bytes, [2]
  // stop transmitting flag [3]
  // i2c port [4]

  int message_size = 0;
  byte address = command_buffer[0];
  byte the_register = command_buffer[1];

  // set the current i2c port if this is for the primary i2c
  if (command_buffer[4] == 0)
  {
    current_i2c_port = &Wire;
  }

#ifdef SECOND_I2C_PORT
  // this is for port 2
  if (command_buffer[4] == 1)
  {
    current_i2c_port = &Wire2;
  }
#endif

  current_i2c_port->beginTransmission(address);
  current_i2c_port->write((byte)the_register);
  current_i2c_port->endTransmission(command_buffer[3]);      // default = true
  current_i2c_port->requestFrom(address, command_buffer[2]); // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (command_buffer[2] < current_i2c_port->available())
  {
    byte report_message[4] = {3, I2C_TOO_FEW_BYTES_RCVD, 1, address};
    Serial.write(report_message, 4);
    return;
  }
  else if (command_buffer[2] > current_i2c_port->available())
  {
    byte report_message[4] = {3, I2C_TOO_MANY_BYTES_RCVD, 1, address};
    Serial.write(report_message, 4);
    return;
  }

  // packet length
  i2c_report_message[0] = command_buffer[2] + 5;

  // report type
  i2c_report_message[1] = I2C_READ_REPORT;

  // i2c_port
  i2c_report_message[2] = command_buffer[4];

  // number of bytes read
  i2c_report_message[3] = command_buffer[2]; // number of bytes

  // device address
  i2c_report_message[4] = address;

  // device register
  i2c_report_message[5] = the_register;

  // append the data that was read
  for (message_size = 0; message_size < command_buffer[2] && current_i2c_port->available(); message_size++)
  {
    i2c_report_message[6 + message_size] = current_i2c_port->read();
  }
  // send slave address, register and received bytes

  for (int i = 0; i < message_size + 6; i++)
  {
    Serial.write(i2c_report_message[i]);
  }
}

void i2c_write()
{
  // command_buffer[0] is the number of bytes to send
  // command_buffer[1] is the device address
  // command_buffer[2] is the i2c port
  // additional bytes to write= command_buffer[3..];

  // set the current i2c port if this is for the primary i2c
  if (command_buffer[2] == 0)
  {
    current_i2c_port = &Wire;
  }

#ifdef SECOND_I2C_PORT
  // this is for port 2
  if (command_buffer[2] == 1)
  {
    current_i2c_port = &Wire2;
  }
#endif
#if defined(__AVR_ATmega328P__) 
  if (current_i2c_port->getWireTimeoutFlag())
  {
    return;
  }
#endif
  current_i2c_port->beginTransmission(command_buffer[1]);

  // write the data to the device
  for (int i = 0; i < command_buffer[0]; i++)
  {
    current_i2c_port->write(command_buffer[i + 3]);
  }
  current_i2c_port->endTransmission();
  // delayMicroseconds(70);
}

/***********************************
   HC-SR04 adding a new device
 **********************************/

void sonar_new()
{
  // command_buffer[0] = trigger pin,  command_buffer[1] = echo pin
  sonars[sonars_index].usonic = new NewPing((uint8_t)command_buffer[0], (uint8_t)command_buffer[1],
      2000);
  sonars[sonars_index].trigger_pin = command_buffer[0];
  sonars_index++;
}

/***********************************
   DHT adding a new device
 **********************************/

void dht_new()
{
  int d_read;
  // report consists of:
  // 0 - byte count
  // 1 - report type
  // 2 - dht report subtype
  // 3 - pin number
  // 4 - error value

  // pre-build an error report in case of a read error
  byte report_message[5] = {4, (byte)DHT_REPORT, (byte)DHT_READ_ERROR, (byte)0, (byte)0};

  dhts[dht_index].dht_sensor = new DHTNEW((uint8_t)command_buffer[0]);
  dhts[dht_index].dht_sensor->setType();

  dhts[dht_index].pin = command_buffer[0];
  d_read = dhts[dht_index].dht_sensor->read();

  // if read return == zero it means no errors.
  if (d_read == 0)
  {
    dht_index++;
  }
  else
  {
    // error found
    // send report and release the dht object

    report_message[3] = command_buffer[0]; // pin number
    report_message[4] = d_read;
    Serial.write(report_message, 5);
    delete (dhts[dht_index].dht_sensor);
  }
}


/***********************************
   Adding a new encoder device
 **********************************/
void encoder_new()
{
  // create new encoder object and get interrupt calback method from map
  optEnc[optEncoder_ix].optEnc_sensor = new OpticalEncoder();
  intCB callbackMethod = interruptMap[optEncoder_ix];

  // command_buffer[0] = encoder pin,  command_buffer[1] = Interrupt trigger mode,  command_buffer[2] = nr of ticks per rotation
  optEnc[optEncoder_ix].optEnc_sensor->setup(command_buffer[0], callbackMethod, command_buffer[1], command_buffer[2]);
  optEnc[optEncoder_ix].pin = command_buffer[0];
  optEncoder_ix++;
}

void stop_all_reports()
{
  stop_reports = true;
  delay(20);
  Serial.flush();
}

void enable_all_reports()
{
  Serial.flush();
  stop_reports = false;
  delay(20);
}

void get_next_command()
{
  byte command;
  byte packet_length;
  command_descriptor command_entry;

  // clear the command buffer
  memset(command_buffer, 0, sizeof(command_buffer));

  // if there is no command waiting, then return
  if (! Serial.available())
  {
    return;
  }
  // get the packet length
  packet_length = (byte)Serial.read();

  while (! Serial.available())
  {
    delay(1);
  }

  // get the command byte
  command = (byte)Serial.read();

  // uncomment the next line to see the packet length and command
  //send_debug_info(packet_length, command);
  command_entry = command_table[command];

  if (packet_length > 1)
  {
    // get the data for that command
    for (int i = 0; i < packet_length - 1; i++)
    {
      // need this delay or data read is not correct
      while (! Serial.available())
      {
        delay(1);
      }
      command_buffer[i] = (byte)Serial.read();
      // uncomment out to see each of the bytes following the command
      //send_debug_info(i, command_buffer[i]);
    }
  }
  command_entry.command_func();
}

void scan_digital_inputs()
{
  byte value;

  // report message

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = value
  byte report_message[4] = {3, DIGITAL_REPORT, 0, 0};

  for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++)
  {
    if (the_digital_pins[i].pin_mode == INPUT ||
        the_digital_pins[i].pin_mode == INPUT_PULLUP)
    {
      if (the_digital_pins[i].reporting_enabled)
      {
        // if the value changed since last read
        value = (byte)digitalRead(the_digital_pins[i].pin_number);
        if (value != the_digital_pins[i].last_value)
        {
          the_digital_pins[i].last_value = value;
          report_message[2] = (byte)i;
          report_message[3] = value;
          Serial.write(report_message, 4);
        }
      }
    }
  }
}

void scan_analog_inputs()
{
  int value;

  // report message

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = high order byte of value
  // byte 4 = low order byte of value

  byte report_message[5] = {4, ANALOG_REPORT, 0, 0, 0};
  uint8_t adjusted_pin_number;
  int differential;

  current_millis = millis();
  if (current_millis - previous_millis > analog_sampling_interval)
  {
    previous_millis += analog_sampling_interval;
    for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++)
    {
      if (the_analog_pins[i].pin_mode == AT_ANALOG)
      {
        if (the_analog_pins[i].reporting_enabled)
        {
          // if the value changed since last read
          // adjust pin number for the actual read
          adjusted_pin_number = (uint8_t)(analog_read_pins[i]);
          value = analogRead(adjusted_pin_number);
          differential = abs(value - the_analog_pins[i].last_value);
          if (differential >= the_analog_pins[i].differential)
          {
            //trigger value achieved, send out the report
            the_analog_pins[i].last_value = value;
            // input_message[1] = the_analog_pins[i].pin_number;
            report_message[2] = (byte)i;
            report_message[3] = highByte(value); // get high order byte
            report_message[4] = lowByte(value);
            Serial.write(report_message, 5);
            delay(1);
          }
        }
      }
    }
  }
}

void scan_sonars()
{
  unsigned int distance;

  if (sonars_index)
  {
    sonar_current_millis = millis();
    if (sonar_current_millis - sonar_previous_millis > sonar_scan_interval)
    {
      sonar_previous_millis += sonar_scan_interval;
      distance = sonars[last_sonar_visited].usonic->ping() / US_ROUNDTRIP_CM;
      if (distance != sonars[last_sonar_visited].last_value)
      {
        sonars[last_sonar_visited].last_value = distance;

        // byte 0 = packet length
        // byte 1 = report type
        // byte 2 = trigger pin number
        // byte 3 = distance high order byte
        // byte 4 = distance low order byte
        byte report_message[5] = {4, SONAR_DISTANCE, sonars[last_sonar_visited].trigger_pin,
                                  (byte)(distance >> 8), (byte)(distance & 0xff)
                                 };
        Serial.write(report_message, 5);
      }
      last_sonar_visited++;
      if (last_sonar_visited == sonars_index)
      {
        last_sonar_visited = 0;
      }
    }
  }
}

void scan_dhts()
{
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
  byte report_message[12] = {11, DHT_REPORT, DHT_DATA, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  byte d_read;

  float dht_data;

  // are there any dhts to read?
  if (dht_index)
  {
    // is it time to do the read? This should occur every 2 seconds
    dht_current_millis = millis();
    if (dht_current_millis - dht_previous_millis > dht_scan_interval)
    {
      // update for the next scan
      dht_previous_millis += dht_scan_interval;

      // read and report all the dht sensors
      for (int i = 0; i < dht_index; i++)
      {
        report_message[3] = dhts[i].pin;
        // get humidity
        dht_data = dhts[i].dht_sensor->getHumidity();
        memcpy(&report_message[4], &dht_data, sizeof dht_data);

        // get temperature
        dht_data = dhts[i].dht_sensor->getTemperature();
        memcpy(&report_message[8], &dht_data, sizeof dht_data);

        Serial.write(report_message, 12);

        // now read do a read for this device for next go around
        d_read = dhts[i].dht_sensor->read();

        if (d_read)
        {
          // error found
          // send report
          //send_debug_info(1, 1);
          report_message[0] = 4;
          report_message[1] = DHT_REPORT;
          report_message[2] = DHT_READ_ERROR;
          report_message[3] = dhts[i].pin; // pin number
          report_message[4] = d_read;
          Serial.write(report_message, 5);
        }
      }
    }
  }
}

void scan_encoders()
{
  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = encoder ticks high order byte for data or error value
  // byte 4 = encoder ticks byte 2
  // byte 5 = encoder ticks byte 3
  // byte 6 = encoder ticks byte 4
  byte report_message[7] = {6, ENCODER_REPORT, 0, 0, 0, 0, 0};

  long optEnc_return_val = 0;

  if (optEncoder_ix) {
    optenc_current_millis = millis();
    if (optenc_current_millis - optenc_previous_millis > optenc_scan_interval) {
      optenc_previous_millis += optenc_scan_interval;

      for (int i = 0; i < optEncoder_ix; ++i) {
        optEnc_return_val = optEnc[i].optEnc_sensor->getPosition();

        if (optEnc_return_val != optEnc[i].last_value)
        {
          optEnc[i].last_value = optEnc_return_val;
          report_message[2] = optEnc[i].pin;

          //copy bytes to message
          memcpy(&report_message[3], &optEnc_return_val, sizeof optEnc_return_val);
          Serial.write(report_message, 7);
        }
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
  for (int i = 0; i < MAX_SERVOS; i++)
  {
    if (servos[i].attached() == true)
    {
      servos[i].detach();
    }
  }

  sonars_index = 0; // reset the index into the sonars array

  sonar_current_millis = 0;  // for analog input loop
  sonar_previous_millis = 0; // for analog input loop
  sonar_scan_interval = 33;  // Milliseconds between sensor pings

  dht_index = 0; // index into dht array

  dht_current_millis = 0;      // for analog input loop
  dht_previous_millis = 0;     // for analog input loop
  dht_scan_interval = 2000;    // scan dht's every 2 seconds

  // Reset optical encoder timers and index
  optEncoder_ix = 0;
  optenc_current_millis = 0;      // for analog input loop
  optenc_previous_millis = 0;     // for analog input loop
  optenc_scan_interval = 0; // scan encoders every x ms

  init_pin_structures();

  memset(sonars, 0, sizeof(sonars));
  memset(dhts, 0, sizeof(dhts));
  memset(optEnc, 0, sizeof(optEnc));
  enable_all_reports();
}

void init_pin_structures() {
  // create an array of pin_descriptors for 100 pins
  // establish the digital pin array
  for (byte i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++)
  {
    the_digital_pins[i].pin_number = i;
    the_digital_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_digital_pins[i].reporting_enabled = false;
    the_digital_pins[i].last_value = -1;
  }

  // establish the analog pin array
  for (byte i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++)
  {
    the_analog_pins[i].pin_number = i;
    the_analog_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_analog_pins[i].reporting_enabled = false;
    the_analog_pins[i].last_value = -1;
    the_analog_pins[i].differential = 0;
  }
}

void setup()
{
  // initialize the servo allocation map table
  init_pin_structures();

  Serial.begin(1000000);
}

void loop()
{
  // keep processing incoming commands
  get_next_command();

  if (!stop_reports)
  { // stop reporting
    scan_digital_inputs();
    scan_analog_inputs();
    scan_sonars();
    scan_dhts();
    scan_encoders();
  }
}
