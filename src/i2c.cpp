#include "i2c.hpp"
#include "Telemetrix4Arduino.h"
#include "commands.hpp"
#include <Arduino.h>
#include <Wire.h>
#include "config.hpp"

/***********************************
   i2c functions
 **********************************/

void i2c_begin() {
  byte i2c_port = command_buffer[0];
  if (!i2c_port) {
    Wire.begin();
#if defined(__AVR_ATmega328P__)
    Wire.setWireTimeout(10, false);
    Wire.clearWireTimeoutFlag();
#endif
  }

#ifdef SECOND_I2C_PORT
  else {
    Wire2.begin();
#if defined(__AVR_ATmega328P__)
    Wire2.setWireTimeout(10, false);
    Wire2.clearWireTimeoutFlag();
#endif
  }
#endif
}

void i2c_read() {
  TwoWire *current_i2c_port;

  // data in the incoming message:
  // address, [0]
  // register, [1]
  // number of bytes, [2]
  // stop transmitting flag [3]
  // i2c port [4]

  int message_size = 0;
  byte address = command_buffer[0];
  byte the_register = command_buffer[1];
  uint8_t message_id = command_buffer[I2C_READ_MESSAGE_ID];

  // set the current i2c port if this is for the primary i2c
  if (command_buffer[4] == 0) {
    current_i2c_port = &Wire;
  }

#ifdef SECOND_I2C_PORT
  // this is for port 2
  if (command_buffer[4] == 1) {
    current_i2c_port = &Wire2;
  }
#endif
  uint8_t i2c_report_message[64];

  current_i2c_port->beginTransmission(address);
  current_i2c_port->write((byte)the_register);
  current_i2c_port->endTransmission(command_buffer[3]); // default = true
  current_i2c_port->requestFrom(
      address, command_buffer[2]); // all bytes are returned in requestFrom
  // check to be sure correct number of bytes were returned by slave
  auto bytes = command_buffer[2];
  if (bytes != current_i2c_port->available()) {
    i2c_report_message[I2C_PACKET_LENGTH] =
        I2C_ERROR_REPORT_LENGTH;                         // length of the packet
    i2c_report_message[I2C_REPORT_ID] = I2C_READ_FAILED; // report ID
    i2c_report_message[I2C_REPORT_PORT] = command_buffer[4];
    i2c_report_message[I2C_REPORT_DEVICE_ADDRESS] = address;
    Serial.write(i2c_report_message, I2C_ERROR_REPORT_LENGTH);
    return;
  }
  for (int i = 0; i < bytes; i++) {
    i2c_report_message[i + I2C_READ_START_OF_DATA] = current_i2c_port->read();
  }
  // length of the packet
  i2c_report_message[I2C_PACKET_LENGTH] =
      (uint8_t)(bytes + I2C_READ_DATA_BASE_BYTES);

  i2c_report_message[I2C_REPORT_ID] = I2C_READ_REPORT;

  // i2c_port
  i2c_report_message[I2C_REPORT_PORT] = command_buffer[I2C_PORT];

  // i2c_address
  i2c_report_message[I2C_REPORT_DEVICE_ADDRESS] =
      command_buffer[I2C_DEVICE_ADDRESS];

  // i2c register
  i2c_report_message[I2C_REPORT_READ_REGISTER] =
      command_buffer[I2C_READ_REGISTER];

  // number of bytes read from i2c device
  i2c_report_message[I2C_REPORT_READ_NUMBER_DATA_BYTES] = (uint8_t)bytes;

  i2c_report_message[I2C_READ_MESSAGE_ID] = message_id;

  for (int i = 0; i < message_size + 6; i++) {
    Serial.write(i2c_report_message[i]);
  }
}

void i2c_write() {
  TwoWire *current_i2c_port;

  // command_buffer[0] is the number of bytes to send
  // command_buffer[1] is the device address
  // command_buffer[2] is the i2c port
  // additional bytes to write= command_buffer[3..];

  // set the current i2c port if this is for the primary i2c
  if (command_buffer[2] == 0) {
    current_i2c_port = &Wire;
  }

#ifdef SECOND_I2C_PORT
  // this is for port 2
  if (command_buffer[2] == 1) {
    current_i2c_port = &Wire2;
  }
#endif
#if defined(__AVR_ATmega328P__)
  if (current_i2c_port->getWireTimeoutFlag()) {
    return;
  }
#endif
  current_i2c_port->beginTransmission(command_buffer[1]);

  // write the data to the device
  for (int i = 0; i < command_buffer[0]; i++) {
    current_i2c_port->write(command_buffer[i + 3]);
  }
  current_i2c_port->endTransmission();
  // delayMicroseconds(70);
}
