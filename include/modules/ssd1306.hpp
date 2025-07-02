#pragma once

#include <modules.hpp>
#include <main.hpp>
class TmxSSD1306 : public Module {
public:
  TmxSSD1306(uint8_t data[], size_t packet_size) {
    num = data[0];
    type = MODULE_TYPES::TMX_SSD1306;
    // Initialize the SSD1306 display with the provided data
    // For example, you might set the I2C address or other parameters
    // depending on the data received.
    // This is just a placeholder, actual initialization code will depend on
    // the SSD1306 library you are using.
  }
    void readModule() override {
        // Implement reading from the SSD1306 display
        // This could involve reading display data or status
    }
    void writeModule(uint8_t data[], size_t size) override {
        // Implement writing to the SSD1306 display
        // This could involve sending commands or data to the display
        send_debug_info(30, size);
        send_debug_info(31, data[0]);
        if(data[0] == 1) {
            uint8_t display_data[32] = {
                1, 13, 10, 1
            };
            // display_data[0] = 1; // Length of the data to be sent
            this->publishData(data, 4);
        }
    }
    void resetModule() override {
        // Implement resetting the SSD1306 display
        // This could involve sending a reset command or reinitializing the display
    }
    void updModule() override {
        // Implement any periodic updates needed for the SSD1306 display
        // This could involve refreshing the display or updating its content
    }
};