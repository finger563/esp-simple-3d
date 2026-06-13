#include "polygon.hpp"
#include "render_layout.hpp"
#include <algorithm>
#include <cmath>

namespace {

constexpr float kRasterEpsilon = 1e-6f;

inline float compute_depth(float ez, float hw) {
#if ENGINE_DEPTH_USE_INVERSE
  (void)ez;
  return hw;
#else
  return ez / hw;
#endif
}

inline bool depth_test_and_store(depth_t &slot, float depth) {
#if ENGINE_DEPTH_USE_UNORM16
#if ENGINE_DEPTH_USE_INVERSE
  const uint16_t encoded = depth_inv_to_unorm16(depth);
  if (encoded > slot) {
    slot = encoded;
    return true;
  }
  return false;
#else
  const uint16_t encoded = depth_float_to_unorm16(depth);
  if (encoded < slot) {
    slot = encoded;
    return true;
  }
  return false;
#endif
#else
#if ENGINE_DEPTH_USE_INVERSE
  if (depth > slot) {
    slot = depth;
    return true;
  }
  return false;
#else
  if (depth < slot) {
    slot = depth;
    return true;
  }
  return false;
#endif
#endif
}

} // namespace

// Helper to rasterize a single triangle (no Poly construction)
void RasterizeTriangle(const Vertex &a, const Vertex &b, const Vertex &c, RenderType rt,
                       const unsigned short *texture, int texwidth, int texheight, float cr,
                       float cg, float cb) {
  // With both projected axes inverted, front-facing triangles have negative signed screen area.
  if (rt != WIREFRAME) {
    const float signedArea =
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (signedArea >= -kRasterEpsilon)
      return;
  }

  // Copy and y-sort vertices ascending (v0 at top, v2 at bottom)
  Vertex v0 = a, v1 = b, v2 = c;
  if (v1.y < v0.y)
    std::swap(v0, v1);
  if (v2.y < v1.y)
    std::swap(v1, v2);
  if (v1.y < v0.y)
    std::swap(v0, v1);

  const int stripStart = render_target_y_offset;
  const int stripEnd = stripStart + render_target_height;

  // Quick reject if triangle fully outside the current strip or screen
  if (v2.y < 0 || v0.y >= SIZE_Y || v2.y < stripStart || v0.y >= stripEnd)
    return;
  const float minX = std::min({v0.x, v1.x, v2.x});
  const float maxX = std::max({v0.x, v1.x, v2.x});
  if (maxX < 0.0f || minX >= SIZE_X)
    return;
  // Degenerate check
  if (std::fabs(v2.y - v0.y) < kRasterEpsilon)
    return;

  // Dedicated wireframe path using Bresenham's algorithm for edges only
  if (rt == WIREFRAME) {
    auto draw_edge = [&](const Vertex &p0, const Vertex &p1) {
      int x0 = (int)std::lround(p0.x), y0 = (int)std::lround(p0.y);
      int x1 = (int)std::lround(p1.x), y1 = (int)std::lround(p1.y);
      int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
      int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
      int err = dx - dy;

      // steps = number of plotted pixels; denom guards division for coincident points
      const int steps = std::max(dx, dy) + 1;
      const float denom = (float)std::max(steps - 1, 1);
      float ez = p0.ez, hw = p0.hw;
      const float dez = (p1.ez - p0.ez) / denom;
      const float dhw = (p1.hw - p0.hw) / denom;

      const uint16_t color =
          RGB_MAKE((uint8_t)(cr * 255.0f), (uint8_t)(cg * 255.0f), (uint8_t)(cb * 255.0f));

      for (int i = 0;; ++i) {
        if ((unsigned)y0 < (unsigned)SIZE_Y && (unsigned)x0 < (unsigned)SIZE_X && y0 >= stripStart &&
            y0 < stripEnd) {
          const int localY = y0 - stripStart;
          depth_t *zrow = z_buffer + localY * SIZE_X;
          uint16_t *drow = display_buffer + localY * SIZE_X;
          const float depth = compute_depth(ez, hw);
          if (depth_test_and_store(zrow[x0], depth)) {
            drow[x0] = color;
          }
        }
        if (x0 == x1 && y0 == y1)
          break;

        int e2 = 2 * err;
        if (e2 > -dy) {
          err -= dy;
          x0 += sx;
        }
        if (e2 < dx) {
          err += dx;
          y0 += sy;
        }

        // advance depth each plotted pixel
        ez += dez;
        hw += dhw;
      }
    };

    draw_edge(v0, v1);
    draw_edge(v1, v2);
    draw_edge(v2, v0);
    return;
  }

  struct EdgeState {
    float x;
    float ez;
    float hw;
    float u;
    float v;
    float xStep;
    float ezStep;
    float hwStep;
    float uStep;
    float vStep;
  };

  auto setup_edge = [&](const Vertex &from, const Vertex &to, int startY, EdgeState &edge) {
    const float dy = to.y - from.y;
    if (std::fabs(dy) < kRasterEpsilon)
      return false;
    const float invDy = 1.0f / dy;
    edge.xStep = (to.x - from.x) * invDy;
    edge.ezStep = (to.ez - from.ez) * invDy;
    edge.hwStep = (to.hw - from.hw) * invDy;
    edge.uStep = (to.u - from.u) * invDy;
    edge.vStep = (to.v - from.v) * invDy;
    const float yOffset = static_cast<float>(startY) - from.y;
    edge.x = from.x + edge.xStep * yOffset;
    edge.ez = from.ez + edge.ezStep * yOffset;
    edge.hw = from.hw + edge.hwStep * yOffset;
    edge.u = from.u + edge.uStep * yOffset;
    edge.v = from.v + edge.vStep * yOffset;
    return true;
  };

  auto draw_span = [&](int y, EdgeState left, EdgeState right) {
    if (left.x > right.x)
      std::swap(left, right);

    int xStart = (int)std::ceil(left.x);
    int xEnd = (int)std::floor(right.x);
    if (xEnd < 0 || xStart >= SIZE_X)
      return;
    if (xStart < 0)
      xStart = 0;
    if (xEnd >= SIZE_X)
      xEnd = SIZE_X - 1;

    const float spanDx = right.x - left.x;
    float invSpan = 0.0f;
    if (std::fabs(spanDx) >= kRasterEpsilon)
      invSpan = 1.0f / spanDx;

    const float t0 = (std::fabs(spanDx) >= kRasterEpsilon) ? ((float)xStart - left.x) * invSpan : 0.0f;
    float ez = left.ez + (right.ez - left.ez) * t0;
    float hw = left.hw + (right.hw - left.hw) * t0;
    float u = left.u + (right.u - left.u) * t0;
    float v = left.v + (right.v - left.v) * t0;
    const float dez = (right.ez - left.ez) * invSpan;
    const float dhw = (right.hw - left.hw) * invSpan;
    const float du = (right.u - left.u) * invSpan;
    const float dv = (right.v - left.v) * invSpan;

    const uint16_t flatColor =
        RGB_MAKE((uint8_t)(cr * 255.0f), (uint8_t)(cg * 255.0f), (uint8_t)(cb * 255.0f));
    const int localY = y - stripStart;
    depth_t *zrow = z_buffer + localY * SIZE_X;
    uint16_t *drow = display_buffer + localY * SIZE_X;

    if (rt == TEXTURED && texture) {
      const float uScale = (float)(texwidth - 1) * 65536.0f;
      const float vScale = (float)(texheight - 1) * 65536.0f;
      for (int x = xStart; x <= xEnd; ++x) {
        const float depth = compute_depth(ez, hw);
        if (depth_test_and_store(zrow[x], depth)) {
          const float recipHw = 1.0f / hw;
          float pu = u * recipHw;
          float pv = v * recipHw;
          int u_lt0 = pu < 0.0f, u_gt1 = pu > 1.0f;
          int u_in = !(u_lt0 | u_gt1);
          int v_lt0 = pv < 0.0f, v_gt1 = pv > 1.0f;
          int v_in = !(v_lt0 | v_gt1);
          pu = u_in * pu + u_gt1 * 1.0f + u_lt0 * 0.0f;
          pv = v_in * pv + v_gt1 * 1.0f + v_lt0 * 0.0f;
          const int32_t ufx = (int32_t)(pu * uScale);
          const int32_t vfx = (int32_t)(pv * vScale);
          const int tx = ufx >> 16;
          const int ty = vfx >> 16;
          drow[x] = texture[tx + ty * texwidth];
        }
        ez += dez;
        hw += dhw;
        u += du;
        v += dv;
      }
    } else {
      for (int x = xStart; x <= xEnd; ++x) {
        const float depth = compute_depth(ez, hw);
        if (depth_test_and_store(zrow[x], depth)) {
          drow[x] = flatColor;
        }
        ez += dez;
        hw += dhw;
        u += du;
        v += dv;
      }
    }
  };

  const float splitT = (v1.y - v0.y) / (v2.y - v0.y);
  const float splitX = v0.x + (v2.x - v0.x) * splitT;
  const bool midIsLeft = v1.x < splitX;

  auto rasterize_half = [&](int yStart, int yEnd, const Vertex &shortA, const Vertex &shortB,
                            const Vertex &longA, const Vertex &longB, bool shortIsLeft) {
    if (yEnd < 0 || yStart >= SIZE_Y || yEnd < stripStart || yStart >= stripEnd)
      return;
    yStart = std::max(yStart, stripStart);
    yEnd = std::min(yEnd, stripEnd - 1);
    if (yStart > yEnd)
      return;

    EdgeState left{}, right{};
    if (shortIsLeft) {
      if (!setup_edge(shortA, shortB, yStart, left) || !setup_edge(longA, longB, yStart, right))
        return;
    } else {
      if (!setup_edge(longA, longB, yStart, left) || !setup_edge(shortA, shortB, yStart, right))
        return;
    }

    for (int y = yStart; y <= yEnd; ++y) {
      draw_span(y, left, right);
      left.x += left.xStep;
      left.ez += left.ezStep;
      left.hw += left.hwStep;
      left.u += left.uStep;
      left.v += left.vStep;
      right.x += right.xStep;
      right.ez += right.ezStep;
      right.hw += right.hwStep;
      right.u += right.uStep;
      right.v += right.vStep;
    }
  };

  if (v1.y > v0.y) {
    rasterize_half((int)std::ceil(v0.y), (int)std::floor(v1.y), v0, v1, v0, v2, midIsLeft);
  }
  if (v2.y > v1.y) {
    rasterize_half((int)std::ceil(v1.y), (int)std::floor(v2.y), v1, v2, v0, v2, midIsLeft);
  }
}
