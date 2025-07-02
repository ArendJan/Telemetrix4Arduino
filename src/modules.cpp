#include "modules.hpp"
#include "main.hpp"
#include "commands.hpp"

typedef Module * (*ModuleFunc)(uint8_t[], size_t);

Module * create_ssd1306_module(uint8_t data[], size_t packet_size) {
  return new TmxSSD1306(data, packet_size);
}

size_t module_count = 0; // Initialize module count
Module *modules[MAX_MODULES_COUNT] = {nullptr}; // Array of pointers to modules

void module_new_i(uint8_t command_buffer[], size_t packet_size) {

  const auto msg_type = command_buffer[0];
  send_debug_info(10, msg_type);
  send_debug_info(11, packet_size);
  send_debug_info(12, command_buffer[1]);
  const ModuleFunc module_funcs[] =
      {
        nullptr, //   {MODULE_TYPES::PCA9685, [](std::vector<uint8_t>& data) { return new PCA9685_Module(data); }},
        nullptr,//   {MODULE_TYPES::HIWONDER_SERVO, []( std::vector<uint8_t>& data) { return new Hiwonder_Servo(data); }},
          nullptr,// {MODULE_TYPES::SHUTDOWN_RELAY, [](const std::vector<uint8_t>& data) { return nullptr; /* not implemented */ }},
          create_ssd1306_module, // {MODULE_TYPES::TMX_SSD1306, create_ssd1306_module},
      };
if(module_count >= MAX_MODULES_COUNT) {
    return; // no more modules can be added
  }
  if(msg_type==1) {
    const MODULE_TYPES type = (MODULE_TYPES)command_buffer[2];
    const uint8_t module_num = command_buffer[1];
    // std::vector<uint8_t> data;
    
    // data.insert(data.end(), &command_buffer[4], &command_buffer[packet_size]);

    if (type >= MODULE_TYPES::MAX_MODULES) {
      return;
    }
    Module *module = nullptr;
    // if (type == MODULE_TYPES::PCA9685) {
    //   module = new PCA9685_Module(data);
    // } else if (type == MODULE_TYPES::HIWONDER_SERVO) {
    //   module = new Hiwonder_Servo(data);
    // } else if (type == MODULE_TYPES::SHUTDOWN_RELAY) {
    //   return; // not implemented
    //   // module = new Shutdown_Relay(data);
    // } else if (type == MODULE_TYPES::TMX_SSD1306) {
    //   module = new TmxSSD1306(data);
    // } else {
    //   return;
    // }
    auto func = module_funcs[type];
    if (func == nullptr) {
      return; // module type not supported
    }
    module = func(command_buffer+3, packet_size-3);
    module->type = type;
    module->num = module_num;

    modules[module_num] = module;
    module_count++;
  } else if(msg_type==0) { // check module type feature detection
    bool found = false;
    const uint8_t module_type_target = command_buffer[1];
    send_debug_info(0, module_type_target);
    if (module_type_target < MODULE_TYPES::MAX_MODULES) {
      found = (module_funcs[module_type_target] != nullptr);
      send_debug_info(1, found ? 1 : 0);
    }
    uint8_t message[4] = {
      MODULE_MAIN_REPORT, // message type
      0, // feature check
      module_type_target, // module type
      (uint8_t)(found ? 1u : 0u) // found or not
  };
    send_message(message, 4);
  }
}

void module_data_i(uint8_t command_buffer[], size_t packet_size) {
  const uint8_t module_num = command_buffer[0];
  if (module_num > module_count || module_num >= MAX_MODULES_COUNT) {
    return;
  }
//   std::vector<uint8_t> data;
//   data.insert(data.end(), &command_buffer[2], &command_buffer[packet_size]);
  modules[module_num]->writeModule(command_buffer+1, packet_size-1);
}


void Module::publishData(const uint8_t data[], size_t size) {
  uint8_t out[30] = {
    //   0,                  // write len
      MODULE_REPORT,      // write type
      (uint8_t)this->num, // write num
      this->type,         // write sensor type
  };
  for(size_t i = 0; i < size && i < sizeof(out) - 4; i++) {
    out[i + 4] = data[i]; // copy data to the output buffer
  }
  
  send_message(out, size + 4); // send the message with the data
}