#pragma once

// Zero-cost abstraction for switching between AoS and SoA layouts
// Inspired by https://github.com/crosetto/SoAvsAoS

#include <cstddef>
#include <cstdint>

namespace renderlayout {

struct AoSLayout {};
struct SoALayout {};

#ifndef ENGINE_RENDER_LAYOUT_SOA
#define ENGINE_RENDER_LAYOUT_SOA 1
#endif

// Fields we need in the raster stage
struct Fields {
  enum Index : std::size_t { X = 0, Y, EZ, HW, U, V, R, G, B, COUNT };
};

// AoS pack for up to N vertices
template <std::size_t N> struct PackAoS {
  struct Vert {
    float x, y, ez, hw, u, v, r, g, b;
  } verts[N];

  inline float &x(std::size_t i) { return verts[i].x; }
  inline float &y(std::size_t i) { return verts[i].y; }
  inline float &ez(std::size_t i) { return verts[i].ez; }
  inline float &hw(std::size_t i) { return verts[i].hw; }
  inline float &u(std::size_t i) { return verts[i].u; }
  inline float &v(std::size_t i) { return verts[i].v; }
  inline float &r(std::size_t i) { return verts[i].r; }
  inline float &g(std::size_t i) { return verts[i].g; }
  inline float &b(std::size_t i) { return verts[i].b; }

  inline const float &x(std::size_t i) const { return verts[i].x; }
  inline const float &y(std::size_t i) const { return verts[i].y; }
  inline const float &ez(std::size_t i) const { return verts[i].ez; }
  inline const float &hw(std::size_t i) const { return verts[i].hw; }
  inline const float &u(std::size_t i) const { return verts[i].u; }
  inline const float &v(std::size_t i) const { return verts[i].v; }
  inline const float &r(std::size_t i) const { return verts[i].r; }
  inline const float &g(std::size_t i) const { return verts[i].g; }
  inline const float &b(std::size_t i) const { return verts[i].b; }
};

// SoA pack for up to N vertices
template <std::size_t N> struct PackSoA {
  float X[N];
  float Y[N];
  float EZ[N];
  float HW[N];
  float U[N];
  float V[N];
  float R[N];
  float G[N];
  float B[N];

  inline float &x(std::size_t i) { return X[i]; }
  inline float &y(std::size_t i) { return Y[i]; }
  inline float &ez(std::size_t i) { return EZ[i]; }
  inline float &hw(std::size_t i) { return HW[i]; }
  inline float &u(std::size_t i) { return U[i]; }
  inline float &v(std::size_t i) { return V[i]; }
  inline float &r(std::size_t i) { return R[i]; }
  inline float &g(std::size_t i) { return G[i]; }
  inline float &b(std::size_t i) { return B[i]; }

  inline const float &x(std::size_t i) const { return X[i]; }
  inline const float &y(std::size_t i) const { return Y[i]; }
  inline const float &ez(std::size_t i) const { return EZ[i]; }
  inline const float &hw(std::size_t i) const { return HW[i]; }
  inline const float &u(std::size_t i) const { return U[i]; }
  inline const float &v(std::size_t i) const { return V[i]; }
  inline const float &r(std::size_t i) const { return R[i]; }
  inline const float &g(std::size_t i) const { return G[i]; }
  inline const float &b(std::size_t i) const { return B[i]; }
};

#if ENGINE_RENDER_LAYOUT_SOA
template <std::size_t N> using RenderPack = PackSoA<N>;
using LayoutTag = SoALayout;
#else
template <std::size_t N> using RenderPack = PackAoS<N>;
using LayoutTag = AoSLayout;
#endif

} // namespace renderlayout
