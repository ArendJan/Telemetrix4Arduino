#include "sensors.hpp"
#if MAX_SENSORS_COUNT > 0
#include "commands.hpp"
#include "main.hpp"
#include "sensors/veml6040.hpp" // Include the VEML6040

void Sensor::writeSensorData(const uint8_t data[], size_t size) {
  uint8_t out[30] = {
      SENSOR_REPORT,      // write type
      (uint8_t)this->num, // write num
      this->type,         // write sensor type
  };
  for (size_t i = 0; i < size && i < sizeof(out) - 3; i++) {
    out[i + 3] = data[i]; // copy data to the output buffer
  }
  send_message(out, size + 3);
}
Sensor *sensors[MAX_SENSORS_COUNT] = {}; // Array of pointers to sensors
size_t sensors_count = 0;                // Number of sensors in the array

void sensor_new_i(uint8_t command_buffer[], size_t packet_size) {
  const SENSOR_TYPES type = (SENSOR_TYPES)command_buffer[1];
  const uint8_t sensor_num = command_buffer[0];
  uint8_t *sensor_data = command_buffer + 2; // data starts after type and num
  size_t sensor_data_size = packet_size - 2; // size of the data
  if (type >= SENSOR_TYPES::MAX_SENSORS) {
    return;
  }
  Sensor *sensor = nullptr;
  if (type == SENSOR_TYPES::VEML6040) {
    sensor = new VEML6040_Sensor(sensor_data, sensor_data_size);
  } else if (type == SENSOR_TYPES::TOF_VL53) {
    // sensor = new VL53L0X_Sensor(sensor_data,sensor_data_size);
  } else if (type == SENSOR_TYPES::MPU_9250) {
    // sensor = new MPU9250_Sensor(sensor_data,sensor_data_size);
  } else if (type == SENSOR_TYPES::LOAD_CELL) {
    // sensor = new HX711_Sensor(sensor_data,sensor_data_size);
  } else if (type == SENSOR_TYPES::INA226a) {
    // sensor = new INA226_Sensor(sensor_data,sensor_data_size);
  } else if (type == SENSOR_TYPES::HMC5883l) {
    // sensor = new HMC5883L_Sensor(sensor_data,sensor_data_size);
  } else if (type == SENSOR_TYPES::AS5600_t) {
    // sensor = new AS5600_Sensor(sensor_data,sensor_data_size);
  } else {
    return;
  }

  sensor->type = type;
  sensor->num = sensor_num;

  sensors[sensor_num] = sensor;
  sensors_count++;
}

void readSensors() {
  for (size_t i = 0; i < sensors_count; i++) {
    if (sensors[i] == nullptr || sensors[i]->stop) {
      continue; // skip if sensor is not initialized or stopped
    }
    sensors[i]->readSensor();
  }
}

#else
// No sensors supported, so we define empty functions
void sensor_new_i(uint8_t command_buffer[], size_t packet_size) {}
void readSensors() {}

#endif // MAX_SENSORS_COUNT > 0