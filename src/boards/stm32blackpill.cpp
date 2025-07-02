
#include <config.hpp>
#if defined(ARDUINO_ARCH_STM32) && defined(ARDUINO_BLACKPILL_F103C8)

void hw_init() {
  analogWriteResolution(8);
  analogReadResolution(10);
}
#endif

#if defined(ARDUINO_ARCH_STM32) && defined(ARDUINO_BLACKPILL_F303CC)
void hw_init() {
  analogWriteResolution(8);
  analogReadResolution(10);
}
#endif
