#pragma once

#include <cstdint>

#include "textures.hpp"
#include "vertex.hpp"

extern float *z_buffer;
extern uint16_t *display_buffer;

// This tells the engine what type of rendering we want for this polygon
enum RenderType {
  WIREFRAME,      // Only draw the edges of the polygon
  FLAT,           // Color is polygon color
  COLORED,        // Color is interpolated between vertices
  SMOOTH,         // Normals are interpolated (smooth shading)
  TEXTURED,       // Texture Coords are interpolated (no shading)
  TEXTURED_SMOOTH // Texture coords and normals are interpolated
};

// Indexed rendering helper: rasterize one transformed triangle
void RasterizeTriangle(const Vertex &a, const Vertex &b, const Vertex &c, RenderType rt,
                       const unsigned short *texture, int texwidth, int texheight, float cr,
                       float cg, float cb);
