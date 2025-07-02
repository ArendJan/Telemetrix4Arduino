#include <config.hpp>


#if defined(ARDUINO_ARCH_ESP32)
 void hw_init() {

  analogWriteResolution(8);
  analogReadResolution(10);
    Wire1.setPins(SECOND_I2C_PORT_SDA, SECOND_I2C_PORT_SCL);
}

TwoWire Wire2 = Wire1; // Use GPIO 16 and 17 for I2C on ESP32
#endif