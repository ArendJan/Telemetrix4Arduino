#pragma once
#include <config.hpp>
#include <Wire.h>
void i2c_begin();
void i2c_read();
void i2c_write();

// i2c report buffer offsets
#define I2C_PACKET_LENGTH 0
#define I2C_REPORT_ID 1
#define I2C_REPORT_PORT 2
#define I2C_REPORT_DEVICE_ADDRESS 3
#define I2C_REPORT_READ_REGISTER 4
#define I2C_REPORT_READ_NUMBER_DATA_BYTES 5

#define I2C_ERROR_REPORT_LENGTH 4
#define I2C_ERROR_REPORT_NUM_OF_BYTE_TO_SEND 5
// i2c_common
#define I2C_PORT 1
#define I2C_DEVICE_ADDRESS 2 // read and write

// i2c_init
#define I2C_SDA_GPIO_PIN 2
#define I2C_SCL_GPIO_PIN 3

// i2c_read
#define I2C_READ_MESSAGE_ID 3
#define I2C_READ_REGISTER 4
#define I2C_READ_LENGTH 5
#define I2C_READ_NO_STOP_FLAG 6

// I2c_write
#define I2C_WRITE_MESSAGE_ID 3
#define I2C_WRITE_NUMBER_OF_BYTES 4
#define I2C_WRITE_NO_STOP_FLAG 5
#define I2C_WRITE_BYTES_TO_WRITE 6

// This defines how many bytes there are
// that precede the first byte read position
// in the i2c report message buffer.
#define I2C_READ_DATA_BASE_BYTES 6

// Start of i2c data read within the message buffer
#define I2C_READ_START_OF_DATA 7

// Indicator that no i2c register is being specified in the command
#define I2C_NO_REGISTER 254



extern TwoWire* i2c_buses[I2C_COUNT];


bool write_i2c(int i2c_port, int device_address, const uint8_t* data, size_t length);
bool read_i2c(int i2c_port, int device_address, const uint8_t*  registers, size_t register_length, uint8_t* data, size_t data_length);