#pragma once

#include <main.hpp>
#include <modules.hpp>
class TmxSSD1306 : public Module {
public:
  TmxSSD1306(uint8_t data[], size_t packet_size) {
    num = data[0];
    type = MODULE_TYPES::TMX_SSD1306;
    // TODO: this is just for testing the module system.
  }
  void readModule() override {
    // probably nothing to read
  }
  void writeModule(uint8_t data[], size_t size) override {
    // // TODO: implement writing to the SSD1306 display
    // send_debug_info(30, size);
    // send_debug_info(31, data[0]);
    if (data[0] == 1) { // test reply on the "booting..." text commands.
      uint8_t display_data[32] = {1, 10, 10, 1};

      this->publishData(display_data, 4);
    }
  }
  void resetModule() override {}
  void updModule() override {
    // MAYBE: write a few bytes to the display, or not, depends on lib.
  }
};