#include "modules.hpp"
#include "commands.hpp"
#include "main.hpp"

typedef Module *(*ModuleFunc)(uint8_t[], size_t);

Module *create_ssd1306_module(uint8_t data[], size_t packet_size) {
  return new TmxSSD1306(data, packet_size);
}

size_t module_count = 0;                        // Initialize module count
Module *modules[MAX_MODULES_COUNT] = {nullptr}; // Array of pointers to modules

void module_new_i(uint8_t command_buffer[], size_t packet_size) {

  const auto msg_type = command_buffer[0];
  const ModuleFunc module_funcs[] = {
      nullptr, //   {MODULE_TYPES::PCA9685, [](std::vector<uint8_t>& data) {
               //   return new PCA9685_Module(data); }},
      nullptr, //   {MODULE_TYPES::HIWONDER_SERVO, []( std::vector<uint8_t>&
               //   data) { return new Hiwonder_Servo(data); }},
      nullptr, // {MODULE_TYPES::SHUTDOWN_RELAY, [](const std::vector<uint8_t>&
               // data) { return nullptr; /* not implemented */ }},
      create_ssd1306_module, // {MODULE_TYPES::TMX_SSD1306,
                             // create_ssd1306_module},
  };
  if (module_count >= MAX_MODULES_COUNT) {
    return; // no more modules can be added
  }
  if (msg_type == 1) {
    const MODULE_TYPES type = (MODULE_TYPES)command_buffer[2];
    const uint8_t module_num = command_buffer[1];
    // std::vector<uint8_t> data;

    // data.insert(data.end(), &command_buffer[4],
    // &command_buffer[packet_size]);

    if (type >= MODULE_TYPES::MAX_MODULES) {
      return;
    }
    Module *module = nullptr;
    auto func = module_funcs[type];
    if (func == nullptr) {
      module_count++; // Just increment counter, to keep the same index on mcu
                      // and computer.
      return;         // module type not supported
    }
    module = func(command_buffer + 3, packet_size - 3);
    module->type = type;
    module->num = module_num;

    modules[module_num] = module;
    module_count++;
  } else if (msg_type == 0) { // check module type feature detection
    bool found = false;
    const uint8_t module_type_target = command_buffer[1];
    if (module_type_target < MODULE_TYPES::MAX_MODULES) {
      found = (module_funcs[module_type_target] != nullptr);
    }
    uint8_t message[4] = {
        MODULE_MAIN_REPORT,        // message type
        0,                         // feature check
        module_type_target,        // module type
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
  //   data.insert(data.end(), &command_buffer[2],
  //   &command_buffer[packet_size]);
  modules[module_num]->writeModule(command_buffer + 1, packet_size - 1);
}

void Module::publishData(const uint8_t data[], size_t size) {
  uint8_t out[30] = {
      //   0,                  // write len
      MODULE_REPORT,      // write type
      (uint8_t)this->num, // write num
      this->type,         // write sensor type
  };
  for (size_t i = 0; i < size && i < sizeof(out) - 4; i++) {
    out[i + 3] = data[i]; // copy data to the output buffer
  }

  send_message(out, size + 4); // send the message with the data
  // TODO: check dit
}

void scan_modules() {
  for (size_t i = 0; i < module_count; i++) {
    if (modules[i] != nullptr && !modules[i]->stop) {
      modules[i]->readModule();
    }
  }
}

void upd_modules() {
  for (size_t i = 0; i < module_count; i++) {
    if (modules[i] != nullptr && !modules[i]->stop) {
      modules[i]->updModule();
    }
  }
}