#include <config.hpp>

#if  defined(ARDUINO_RASPBERRY_PI_PICO_2W) || defined(ARDUINO_RASPBERRY_PI_PICO_2)
TwoWire Wire2(i2c1, SECOND_I2C_PORT_SDA, SECOND_I2C_PORT_SCL);
#endif

#if defined(ARDUINO_RASPBERRY_PI_PICO)
TwoWire Wire2(SECOND_I2C_PORT_SDA, SECOND_I2C_PORT_SCL);
#endif

#if defined(ARDUINO_ARCH_RP2040)
void hw_init() {
  analogWriteResolution(8);
  analogReadResolution(10);
}
#endif
