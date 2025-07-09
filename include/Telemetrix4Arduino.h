#pragma once
#include <Arduino.h>
// #include <array>
// maximum length of a command in bytes
#define MAX_COMMAND_LENGTH 30

extern uint8_t command_buffer[MAX_COMMAND_LENGTH];

template <size_t N> void send_message(const uint8_t (&message)[N]);