#include <config.hpp>

#if defined(ARDUINO_ARCH_SAMD) && defined(ARDUINO_ITSYBITSY_M4)
void hw_init() {
  analogWriteResolution(8); // Set default PWM resolution to 8 bits
  analogReadResolution(10); // Set default ADC resolution to 10 bits
}

#endif