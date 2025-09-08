#include "polygon.hpp"
#include "render_layout.hpp"
#include <algorithm>
#include <cmath>

// Helper to rasterize a single triangle (no Poly construction)
void RasterizeTriangle(const Vertex &a, const Vertex &b, const Vertex &c, RenderType rt,
                       const unsigned short *texture, int texwidth, int texheight, float cr,
                       float cg, float cb) {
  // Backface culling using camera-space positions reconstructed as (ex/ey/ez)/(hw)
  if (rt != WIREFRAME) {
    float ax = a.ex / std::max(a.hw, 1e-12f);
    float ay = a.ey / std::max(a.hw, 1e-12f);
    float az = a.ez / std::max(a.hw, 1e-12f);
    float bx = b.ex / std::max(b.hw, 1e-12f);
    float by = b.ey / std::max(b.hw, 1e-12f);
    float bz = b.ez / std::max(b.hw, 1e-12f);
    float cx = c.ex / std::max(c.hw, 1e-12f);
    float cy = c.ey / std::max(c.hw, 1e-12f);
    float cz = c.ez / std::max(c.hw, 1e-12f);
    Vector3D p0(ax, ay, az), p1(bx, by, bz), p2(cx, cy, cz);
    Vector3D e1 = p1 - p0;
    Vector3D e2 = p2 - p0;
    Vector3D n = Cross(e1, e2);
    Vector3D eye(0, 0, -1);
    Vector3D cull = eye - p0;
    float test = cull * n;
    // Use a tolerance scaled by depth to avoid popping due to precision
    float depthScale = std::max({std::fabs(az), std::fabs(bz), std::fabs(cz), 1.0f});
    float eps = 1e-3f * depthScale;
    if (test < -eps) {
      return; // confidently backfacing
    }
  }

  // Copy and y-sort vertices ascending (v0 at top, v2 at bottom)
  Vertex v0 = a, v1 = b, v2 = c;
  if (v1.y < v0.y)
    std::swap(v0, v1);
  if (v2.y < v1.y)
    std::swap(v1, v2);
  if (v1.y < v0.y)
    std::swap(v0, v1);

  // Quick reject if triangle fully outside screen vertically
  if (v2.y < 0 || v0.y >= SIZE_Y)
    return;
  // Degenerate check
  if (std::fabs(v2.y - v0.y) < 1e-6f)
    return;

  // Dedicated wireframe path using Bresenham's algorithm for edges only
  if (rt == WIREFRAME) {
    auto draw_edge = [&](const Vertex &p0, const Vertex &p1) {
      int x0 = (int)std::lround(p0.x);
      int y0 = (int)std::lround(p0.y);
      int x1 = (int)std::lround(p1.x);
      int y1 = (int)std::lround(p1.y);
      int dx = std::abs(x1 - x0);
      int dy = std::abs(y1 - y0);
      int sx = (x0 < x1) ? 1 : -1;
      int sy = (y0 < y1) ? 1 : -1;
      int err = dx - dy;
      int steps = std::max(dx, dy);
      if (steps <= 0)
        steps = 1;
      float dez = (p1.ez - p0.ez) / (float)steps;
      float dhw = (p1.hw - p0.hw) / (float)steps;
      float ez = p0.ez;
      float hw = p0.hw;
      uint16_t color =
          RGB_MAKE((uint8_t)(cr * 255.0f), (uint8_t)(cg * 255.0f), (uint8_t)(cb * 255.0f));
      for (int i = 0;; ++i) {
        if ((unsigned)y0 < (unsigned)SIZE_Y && (unsigned)x0 < (unsigned)SIZE_X) {
          float zval = ez / hw;
          float *zrow = z_buffer + y0 * SIZE_X;
          uint16_t *drow = display_buffer + y0 * SIZE_X;
          if (zval < zrow[x0]) {
            zrow[x0] = zval;
            drow[x0] = color;
          }
        }
        if (x0 == x1 && y0 == y1)
          break;
        int e2 = 2 * err;
        if (e2 > -dy) {
          err -= dy;
          x0 += sx;
          ez += dez;
          hw += dhw;
        }
        if (e2 < dx) {
          err += dx;
          y0 += sy;
        }
      }
    };
    draw_edge(v0, v1);
    draw_edge(v1, v2);
    draw_edge(v2, v0);
    return;
  }

  auto lerp_vertex = [](const Vertex &a, const Vertex &b, float t) {
    Vertex r;
    r.x = a.x + (b.x - a.x) * t;
    r.y = a.y + (b.y - a.y) * t;
    r.ez = a.ez + (b.ez - a.ez) * t;
    r.hw = a.hw + (b.hw - a.hw) * t;
    r.u = a.u + (b.u - a.u) * t;
    r.v = a.v + (b.v - a.v) * t;
    return r;
  };

  auto span_and_fill = [&](int yStart, int yEnd, const Vertex &leftA, const Vertex &leftB,
                           const Vertex &rightA, const Vertex &rightB) {
    if (yEnd < 0 || yStart >= SIZE_Y)
      return;
    if (yStart < 0)
      yStart = 0;
    if (yEnd >= SIZE_Y)
      yEnd = SIZE_Y - 1;
    const float dyL = (leftB.y - leftA.y);
    const float dyR = (rightB.y - rightA.y);
    if (std::fabs(dyL) < 1e-6f || std::fabs(dyR) < 1e-6f)
      return;
    for (int y = yStart; y <= yEnd; ++y) {
      float tL = ((float)y + 0.0f - leftA.y) / dyL;
      float tR = ((float)y + 0.0f - rightA.y) / dyR;
      Vertex L = lerp_vertex(leftA, leftB, tL);
      Vertex R = lerp_vertex(rightA, rightB, tR);
      if (L.x > R.x)
        std::swap(L, R);

      int xStart = (int)std::ceil(L.x);
      int xEnd = (int)std::floor(R.x);
      if (xEnd < 0 || xStart >= SIZE_X)
        continue;
      if (xStart < 0)
        xStart = 0;
      if (xEnd >= SIZE_X)
        xEnd = SIZE_X - 1;
      const float invDen = 1.0f / (R.x - L.x + 1e-12f);

      float t0 = ((float)xStart - L.x) * invDen;
      float ez = L.ez + (R.ez - L.ez) * t0;
      float hw = L.hw + (R.hw - L.hw) * t0;
      float u = L.u + (R.u - L.u) * t0;
      float v = L.v + (R.v - L.v) * t0;
      const float dez = (R.ez - L.ez) * invDen;
      const float dhw = (R.hw - L.hw) * invDen;
      const float du = (R.u - L.u) * invDen;
      const float dv = (R.v - L.v) * invDen;

      uint16_t flatColor =
          RGB_MAKE((uint8_t)(cr * 255.0f), (uint8_t)(cg * 255.0f), (uint8_t)(cb * 255.0f));
      float *zrow = z_buffer + y * SIZE_X;
      uint16_t *drow = display_buffer + y * SIZE_X;

      if (rt == WIREFRAME) {
        // Draw only edges: we emit pixels at boundaries
        // Left and right endpoints
        int lx = xStart;
        int rx = xEnd;
        if (lx >= 0 && lx < SIZE_X) {
          float zval = ez / hw;
          if (zval < zrow[lx]) {
            zrow[lx] = zval;
            drow[lx] =
                RGB_MAKE((uint8_t)(cr * 255.0f), (uint8_t)(cg * 255.0f), (uint8_t)(cb * 255.0f));
          }
        }
        // advance to the end sample
        float stepCount = (float)(rx - xStart);
        ez += dez * stepCount;
        hw += dhw * stepCount;
        u += du * stepCount;
        v += dv * stepCount;
        if (rx >= 0 && rx < SIZE_X) {
          float zval = ez / hw;
          if (zval < zrow[rx]) {
            zrow[rx] = zval;
            drow[rx] =
                RGB_MAKE((uint8_t)(cr * 255.0f), (uint8_t)(cg * 255.0f), (uint8_t)(cb * 255.0f));
          }
        }
      } else if (rt == TEXTURED && texture) {
        const float uScale = (float)(texwidth - 1) * 65536.0f;
        const float vScale = (float)(texheight - 1) * 65536.0f;
        for (int x = xStart; x <= xEnd; ++x) {
          float zval = ez / hw;
          if (zval < zrow[x]) {
            zrow[x] = zval;
            float pu = u / hw;
            float pv = v / hw;
            int u_lt0 = pu<0.0f, u_gt1 = pu> 1.0f;
            int u_in = !(u_lt0 | u_gt1);
            int v_lt0 = pv<0.0f, v_gt1 = pv> 1.0f;
            int v_in = !(v_lt0 | v_gt1);
            pu = u_in * pu + u_gt1 * 1.0f + u_lt0 * 0.0f;
            pv = v_in * pv + v_gt1 * 1.0f + v_lt0 * 0.0f;
            int32_t ufx = (int32_t)(pu * uScale);
            int32_t vfx = (int32_t)(pv * vScale);
            int tx = ufx >> 16;
            int ty = vfx >> 16;
            drow[x] = texture[tx + ty * texwidth];
          }
          ez += dez;
          hw += dhw;
          u += du;
          v += dv;
        }
      } else {
        for (int x = xStart; x <= xEnd; ++x) {
          float zval = ez / hw;
          if (zval < zrow[x]) {
            zrow[x] = zval;
            drow[x] = flatColor;
          }
          ez += dez;
          hw += dhw;
          u += du;
          v += dv;
        }
      }
    }
  };

  // Upper half v0->v1
  if (v1.y > v0.y) {
    span_and_fill((int)std::ceil(v0.y), (int)std::floor(v1.y), v0, v1, // short edge v0->v1
                  v0, v2                                               // long edge v0->v2
    );
  }
  // Lower half v1->v2
  if (v2.y > v1.y) {
    span_and_fill((int)std::ceil(v1.y), (int)std::floor(v2.y), v1, v2, // short edge v1->v2
                  v0, v2                                               // long edge v0->v2
    );
  }
}
