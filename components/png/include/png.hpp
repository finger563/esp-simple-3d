#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>

#include "esp_heap_caps.h"

class Png {
public:
  Png() {}
  ~Png() {
    if (encoded_data_) {
      heap_caps_free(encoded_data_);
      encoded_data_ = nullptr;
    }
    if (decoded_data_) {
      heap_caps_free(decoded_data_);
      decoded_data_ = nullptr;
    }
  }

  // Decode from file path
  bool decode(const char *filename);

  // Decode from memory buffer (PNG bytes)
  bool decode_memory(const uint8_t *data, size_t length);

  int get_width() { return image_width_; }
  int get_height() { return image_height_; }
  uint8_t *get_decoded_data() { return decoded_data_; }
  int get_size() { return image_size_; }

protected:
  void open(const char *filename, int32_t *size) {
    if (imgfile_.is_open()) {
      imgfile_.close();
    }
    imgfile_.open(filename, std::ios::binary | std::ios::ate);
    if (!imgfile_.is_open()) {
      *size = 0;
      return;
    }
    *size = (size_t)imgfile_.tellg();
    imgfile_.seekg(0, std::ios::beg);
  }

  void close() {
    if (imgfile_.is_open()) {
      imgfile_.close();
    }
  }

  int32_t read(uint8_t *buffer, int32_t length) {
    if (!imgfile_.is_open()) {
      return 0;
    }
    imgfile_.read((char *)buffer, length);
    return imgfile_.gcount();
  }

  static uint8_t *encoded_data_;
  static uint8_t *decoded_data_;
  static int image_width_;
  static int image_height_;
  static int image_size_;

  std::ifstream imgfile_;
};
