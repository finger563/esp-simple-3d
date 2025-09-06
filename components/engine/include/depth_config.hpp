#pragma once

// Depth buffer configuration
// Enable 16-bit UNORM depth storage for bandwidth savings
#ifndef ENGINE_DEPTH_USE_UNORM16
#define ENGINE_DEPTH_USE_UNORM16 0
#endif

// Depth semantics
// When using float depth: smaller is nearer (z < zbuffer)
// Optional: enable inverse-depth stepping to avoid per-pixel divide
#ifndef ENGINE_DEPTH_USE_INVERSE
#define ENGINE_DEPTH_USE_INVERSE 0
#endif

// Helper macros to map float depth z in [0, ZMAX] to UNORM16
#ifndef ENGINE_DEPTH_Z_MAX
#define ENGINE_DEPTH_Z_MAX 100000.0f
#endif

#if ENGINE_DEPTH_USE_UNORM16
typedef uint16_t depth_t;
#else
typedef float depth_t;
#endif

static inline uint16_t depth_float_to_unorm16(float z) {
  float zn = z / ENGINE_DEPTH_Z_MAX;
  if (zn < 0.0f)
    zn = 0.0f;
  else if (zn > 1.0f)
    zn = 1.0f;
  return (uint16_t)(zn * 65535.0f);
}

// Inverse-depth normalization (zinv / ZINV_MAX)
#ifndef ENGINE_DEPTH_ZINV_MAX
#define ENGINE_DEPTH_ZINV_MAX 1.0f
#endif

static inline uint16_t depth_inv_to_unorm16(float zinv) {
  float zn = zinv / ENGINE_DEPTH_ZINV_MAX;
  if (zn < 0.0f)
    zn = 0.0f;
  else if (zn > 1.0f)
    zn = 1.0f;
  return (uint16_t)(zn * 65535.0f);
}

// Clear values depending on storage and semantics
#if ENGINE_DEPTH_USE_UNORM16
#if ENGINE_DEPTH_USE_INVERSE
#define ENGINE_DEPTH_CLEAR_UNORM16 0
#else
#define ENGINE_DEPTH_CLEAR_UNORM16 65535
#endif
#else
#if ENGINE_DEPTH_USE_INVERSE
#define ENGINE_DEPTH_CLEAR_FLOAT 0.0f
#else
#define ENGINE_DEPTH_CLEAR_FLOAT ENGINE_DEPTH_Z_MAX
#endif
#endif
