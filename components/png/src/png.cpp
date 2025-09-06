#include "png.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

#include "png.h"

uint8_t *Png::encoded_data_ = nullptr;
uint8_t *Png::decoded_data_ = nullptr;
int Png::image_width_ = 0;
int Png::image_height_ = 0;
int Png::image_size_ = 0;

static void png_read_from_memory(png_structp png_ptr, png_bytep out_bytes,
                                 png_size_t byte_count_to_read) {
  auto *buf = (std::pair<const uint8_t *, size_t> *)png_get_io_ptr(png_ptr);
  if (byte_count_to_read > buf->second) {
    png_error(png_ptr, "png_read_from_memory: read beyond end");
    return;
  }
  memcpy(out_bytes, buf->first, byte_count_to_read);
  buf->first += byte_count_to_read;
  buf->second -= byte_count_to_read;
}

// Pack into RGB565 to match engine's RGB_MAKE macro mapping
static inline uint16_t pack_rgb565(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t red = (uint16_t)((r & 0xF8) << 8);   // bits 11..15
  uint16_t green = (uint16_t)((g & 0xFC) << 3); // bits 5..10
  uint16_t blue = (uint16_t)((b & 0xF8) >> 3);  // bits 0..4
  return (uint16_t)(red | green | blue);
}

static bool decode_png_to_rgb565(const uint8_t *data, size_t length, uint8_t *&out_pixels,
                                 int &out_w, int &out_h) {
  if (!data || length < 8)
    return false;
  if (png_sig_cmp((png_bytep)data, 0, 8))
    return false;

  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png_ptr)
    return false;
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_read_struct(&png_ptr, nullptr, nullptr);
    return false;
  }
  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    return false;
  }

  std::pair<const uint8_t *, size_t> io_ctx{data, length};
  png_set_read_fn(png_ptr, &io_ctx, png_read_from_memory);
  png_set_sig_bytes(png_ptr, 0);

  png_read_info(png_ptr, info_ptr);
  png_uint_32 width, height;
  int bit_depth, color_type;
  png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth, &color_type, nullptr, nullptr,
               nullptr);

  // Transforms to 8-bit RGBA
  if (bit_depth == 16)
    png_set_strip_16(png_ptr);
  if (color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_palette_to_rgb(png_ptr);
  if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
    png_set_expand_gray_1_2_4_to_8(png_ptr);
  if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
    png_set_tRNS_to_alpha(png_ptr);
  if (color_type == PNG_COLOR_TYPE_RGB || color_type == PNG_COLOR_TYPE_GRAY ||
      color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_filler(png_ptr, 0xFF, PNG_FILLER_AFTER);
  if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
    png_set_gray_to_rgb(png_ptr);

  png_read_update_info(png_ptr, info_ptr);

  out_w = (int)width;
  out_h = (int)height;
  size_t rowbytes = png_get_rowbytes(png_ptr, info_ptr);

  // Allocate temp RGBA row buffer and final RGB565 buffer
  std::vector<uint8_t> row(rowbytes);
  out_pixels = (uint8_t *)heap_caps_malloc(out_w * out_h * sizeof(uint16_t),
                                           MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  if (!out_pixels) {
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    return false;
  }

  uint16_t *dst = (uint16_t *)out_pixels;
  for (int y = 0; y < out_h; ++y) {
    png_bytep row_ptr = row.data();
    png_read_row(png_ptr, row_ptr, nullptr);
    for (int x = 0; x < out_w; ++x) {
      uint8_t r = row_ptr[x * 4 + 0];
      uint8_t g = row_ptr[x * 4 + 1];
      uint8_t b = row_ptr[x * 4 + 2];
      uint16_t v = pack_rgb565(r, g, b);
      // Store as big-endian in memory to match JPEG path and LCD expectations
      uint16_t be = (uint16_t)((v << 8) | (v >> 8));
      dst[y * out_w + x] = be;
    }
  }

  png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
  return true;
}

bool Png::decode(const char *filename) {
  // open file and read all bytes
  int32_t encoded_length = 0;
  open(filename, &encoded_length);
  if (encoded_length <= 0) {
    close();
    return false;
  }
  if (encoded_data_) {
    heap_caps_free(encoded_data_);
    encoded_data_ = nullptr;
  }
  encoded_data_ = (uint8_t *)heap_caps_malloc(encoded_length, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  if (!encoded_data_) {
    close();
    return false;
  }
  read(encoded_data_, encoded_length);
  close();

  // decode from memory buffer
  if (decoded_data_) {
    heap_caps_free(decoded_data_);
    decoded_data_ = nullptr;
  }
  uint8_t *pixels = nullptr;
  int w = 0, h = 0;
  if (!decode_png_to_rgb565(encoded_data_, (size_t)encoded_length, pixels, w, h)) {
    return false;
  }
  decoded_data_ = pixels;
  image_width_ = w;
  image_height_ = h;
  image_size_ = w * h * sizeof(uint16_t);
  return true;
}

bool Png::decode_memory(const uint8_t *data, size_t length) {
  if (decoded_data_) {
    heap_caps_free(decoded_data_);
    decoded_data_ = nullptr;
  }
  uint8_t *pixels = nullptr;
  int w = 0, h = 0;
  if (!decode_png_to_rgb565(data, length, pixels, w, h)) {
    return false;
  }
  decoded_data_ = pixels;
  image_width_ = w;
  image_height_ = h;
  image_size_ = w * h * sizeof(uint16_t);
  return true;
}
