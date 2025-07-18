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

typedef Sensor *(*SensorFunc)(uint8_t[], size_t);

SensorFunc sensor_funcs[] = {
    nullptr,                 // 0 - GPS
    nullptr,                 // 1 - LOAD_CELL
    nullptr,                 // 2 - MPU_9250
    nullptr,                 // 3 - TOF_VL53
    VEML6040_Sensor::create, // 4 - VEML6040
    nullptr,                 // 5 - ADXL345
    nullptr,                 // 6 - INA226a
    nullptr,                 // 7 - HMC5883l
    nullptr,                 // 8 - AS5600_t
};

void sensor_new_i(uint8_t command_buffer[], size_t packet_size) {
  const uint8_t sensor_cmd = command_buffer[0];

  if (sensor_cmd == 1) {
    const SENSOR_TYPES type = (SENSOR_TYPES)command_buffer[2];
    const uint8_t sensor_num = command_buffer[1];

    uint8_t *sensor_data = command_buffer + 3; // data starts after type and num
    size_t sensor_data_size = packet_size - 3; // size of the data
    if (type >= SENSOR_TYPES::MAX_SENSORS) {
      return;
    }
    Sensor *sensor = nullptr;
    if (sensor_funcs[type] != nullptr) {
      sensor = sensor_funcs[type](sensor_data, sensor_data_size);
      if (sensor == nullptr) {
        // If sensor creation failed, we just increment the count and return
        sensors_count++;
        return;
      }
    } else {
      sensors_count++; // Just increment counter, to keep the same index on mcu
                       // and computer.
      return;
    }
    sensor->type = type;
    sensor->num = sensor_num;

    sensors[sensor_num] = sensor;
    sensors_count++;
  } else if (sensor_cmd == 0) {
    // This is a feature detection command, check if the sensor type is
    // supported
    const SENSOR_TYPES type = (SENSOR_TYPES)command_buffer[1];
    uint8_t found =
        (type < SENSOR_TYPES::MAX_SENSORS && sensor_funcs[type] != nullptr);

    uint8_t message[4] = {
        SENSOR_MAIN_REPORT, // message type
        0,                  // feature check
        (uint8_t)type,      // sensor type
        found               // found or not
    };
    send_message(message, 4);
  }
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