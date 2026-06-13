#pragma once

#include <cstdint>

#include "depth_config.hpp"
#include "textures.hpp"
#include "vertex.hpp"

extern depth_t *z_buffer;
extern uint16_t *display_buffer;
extern int render_target_y_offset;
extern int render_target_height;

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
