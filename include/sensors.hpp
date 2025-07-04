#pragma once
#include <Arduino.h>
#ifndef MAX_SENSORS_COUNT
#define MAX_SENSORS_COUNT 4 // Max number of modules that can be added
#endif // MAX_MODULES_COUNT


#if MAX_SENSORS_COUNT > 0
/*****************************************************************************/
/****SENSORS*/
/*****************************************************************************/

enum SENSOR_TYPES : uint8_t { // Max 255 sensors, but will always fit in a
                              // single byte!
  GPS = 0,
  LOAD_CELL = 1,
  MPU_9250 = 2,
  TOF_VL53 = 3,
  VEML6040 = 4, // Color sensor
  ADXL345 = 5,  // 3 axis accel
  INA226a = 6,
  HMC5883l = 7,
  AS5600_t = 8, // Magnetic angle sensor
  MAX_SENSORS
};

class Sensor {
public:
  virtual void readSensor() = 0;
  virtual void resetSensor() = 0;
  bool stop = false;
  void writeSensorData(const uint8_t data[], size_t size);
  int num;
  SENSOR_TYPES type = SENSOR_TYPES::MAX_SENSORS;
};

const int SENSORS_MAX_SETTINGS_A = 6;
void sensor_new_i(uint8_t command_buffer[], size_t packet_size);
void readSensors();


#else

// No sensors supported, so we define empty functions
void sensor_new_i(uint8_t command_buffer[], size_t packet_size);
void readSensors();
#endif