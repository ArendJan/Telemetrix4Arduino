#pragma once

#include <sensors.hpp>

#if MAX_SENSORS_COUNT > 0
#include <Wire.h>
#include <i2c.hpp>
class VEML6040_Sensor : public Sensor {
public:
  VEML6040_Sensor(uint8_t settings[], size_t settings_size) {
    // Initialize the sensor with the provided settings
    if (settings_size > 0) {
      i2c_port = settings[0]; // Assuming first byte is I2C port
    }
    init_sequence();
  }
  void readSensor()  {
  if (this->stop) {
    return;
  }
  uint8_t data[8];
  uint8_t sensor_data[2];
  bool ok = true;
  size_t data_offset = 0;
  for (uint8_t reg = 0x08; reg <= 0x0B;
       reg++) { // read the 4 registers and add the data to the full data vector
        uint8_t regs[1] = {reg};
    ok &= read_i2c(this->i2c_port, this->i2c_addr, regs, 1, sensor_data, 2);
    // Flip the order to make it match the rest.
    data[0+data_offset] = sensor_data[1];
    data[1+data_offset] = sensor_data[0];
    data_offset += 2;
  }

  this->writeSensorData(data, 8);
  if (!ok) {
    this->stop = true;
  }
}
  void resetSensor() {};

private:
  void init_sequence() {
    uint8_t data[2] = {0, 0x21};
    bool ok = write_i2c(this->i2c_port, this->i2c_addr,
                      data, 2);
  delay(10);
  data[1] = 0x20; // 0x20 = 0b0010'0000, 100 time, no stop bit
  ok &= write_i2c(this->i2c_port, this->i2c_addr,
                  data, 2);
  if (!ok) {
    this->stop = true;
  }
  }
  int i2c_port = 0;
  int i2c_addr = 0x10;
};






#endif