#pragma once

#include <cstdint>
#include <limits>

// constants
const int SIZE_X = 320;
const int SIZE_Y = 240;

#define BACKGROUND_COLOR 0x0000
#define DEFAULT_Z_BUFFER ((float)10000)

[[maybe_unused]] static uint16_t RGB_MAKE(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t red = (uint16_t)((r & 0xF8) << 8);   // bits 11..15
  uint16_t green = (uint16_t)((g & 0xFC) << 3); // bits 5..10
  uint16_t blue = (uint16_t)((b & 0xF8) >> 3);  // bits 0..4
  uint16_t val = (uint16_t)(red | green | blue);
  // // convert RGB888 to RGB565
  // uint16_t val = ((uint16_t)(xB >> 3) << 11) | ((uint16_t)(xG >> 2) << 5) | ((uint16_t)(xR >> 3)
  // << 0); then swap endianess
  return ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8);
}
