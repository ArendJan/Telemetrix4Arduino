#pragma once
#include <Arduino.h>
#define MAX_MODULES_COUNT 4 // Max number of modules that can be added
/***********************************************/
/***************MODULES*************************/
void module_new_i(uint8_t command_buffer[], size_t packet_size); 

void module_data_i(uint8_t command_buffer[], size_t packet_size);


enum MODULE_TYPES : uint8_t { // Max 255 modules, but will always fit in a
                              // single byte!
  PCA9685 = 0,                // 16x 12bit PWM
  HIWONDER_SERVO = 1,
  SHUTDOWN_RELAY = 2,
  TMX_SSD1306 = 3,
  MAX_MODULES
};

class Module {
public:
  virtual void readModule() = 0;
  virtual void writeModule(uint8_t data[], size_t size) = 0;
  virtual void resetModule() = 0;
  bool stop = false;
  void publishData(const uint8_t data[], size_t size);

  int num = 0;
  MODULE_TYPES type = MODULE_TYPES::MAX_MODULES;
  // called at every loop, only used when needed (Oled update)
  virtual void updModule() {};
};

extern Module *modules[MAX_MODULES_COUNT]; // Array of pointers to modules
extern size_t module_count; // Number of modules in the array


#include "modules/ssd1306.hpp" // Include the SSD1306 module header