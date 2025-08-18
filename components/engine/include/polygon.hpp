#pragma once

#include <cstdint>

#include "textures.hpp"
#include "vertex.hpp"

extern float *z_buffer;
extern uint16_t *display_buffer;

#define POLY_MAX_VERTICES 4 // don't want anything other than tris and quads

// This tells the engine what type of rendering we want for this polygon
enum RenderType {
  FLAT,           // Color is polygon color
  COLORED,        // Color is interpolated between vertices
  SMOOTH,         // Normals are interpolated (smooth shading)
  TEXTURED,       // Texture Coords are interpolated (no shading)
  TEXTURED_SMOOTH // Texture coords and normals are interpolated
};

class Poly {
public:
  Vertex v[POLY_MAX_VERTICES];
  int numVertices;

  const uint16_t *texture; // for texturing
  int texwidth;            // width of texture
  int texheight;           // height of texture

  double r, g, b; // for flat coloring

  RenderType rType; // Method of rendering this polygon

  Vector3D normal;  // for backface culling
  bool visible;     // for backface culling
  bool doublesided; // for doublesided polygons

  // THESE ARE FOR RENDERING QUICKLY
  Vertex ySorted[POLY_MAX_VERTICES]; // may not need this; just sort v[]
  double increments[POLY_MAX_VERTICES][2]
                   [NUM_VERTEX_DATA]; // increments along edges of left & right slopes
  int edges[POLY_MAX_VERTICES];       // maintains connectivity after sorting
  int sides[POLY_MAX_VERTICES][2];    // indices for starting vertices of each edge
  int numInterps;                     // should use this for further optimization

  Poly() {
    numVertices = 0;
    texture = defaulttexture;
    texwidth = defaulttexture_width;
    texheight = defaulttexture_height;
    r = g = b = 1;
    doublesided = false;
  }
  Poly(const Vertex &_v1, const Vertex &_v2, const Vertex &_v3, const Vertex &_v4 = Vertex(),
       const int vertices = 3, const Vector3D &n = Vector3D(), const RenderType rt = FLAT,
       const Vector3D &rgb = Vector3D(1, 1, 1, 1), const unsigned short *tex = defaulttexture,
       const int width = defaulttexture_width, const int height = defaulttexture_height) {
    v[0] = _v1;
    v[1] = _v2;
    v[2] = _v3;
    v[3] = _v4;
    numVertices = vertices;
    normal = n;
    rType = rt;
    texture = tex;
    texwidth = width;
    texheight = height;
    r = rgb.x;
    b = rgb.y;
    g = rgb.z;
    doublesided = false;
  }
  ~Poly() {}

  // Polygon variable setter methods
  void SetDoubleSided(bool ds) { doublesided = ds; }
  bool GetDoubleSided(void) { return doublesided; }
  void SetTexture(const unsigned short *tex, const int width, const int height) {
    texture = tex;
    texwidth = width;
    texheight = height;
  }
  void SetTexture(const unsigned short *tex, const int width, const int height, Point2D u1,
                  Point2D u2, Point2D u3, Point2D u4) {
    texture = tex;
    texwidth = width;
    texheight = height;
    v[0].u = u1.x;
    v[0].v = u1.y;
    v[1].u = u2.x;
    v[1].v = u2.y;
    v[2].u = u3.x;
    v[2].v = u3.y;
    v[3].u = u4.x;
    v[3].v = u4.y;
  }
  void SetRenderType(const RenderType rt) { rType = rt; }
  void SetNormal(const Vector3D n) { normal = n; }
  void SetColor(const double _r, const double _g, const double _b) {
    r = _r;
    g = _g;
    b = _b;
  }
  void SetVertexColors(const double _r1, const double _g1, const double _b1, const double _r2,
                       const double _g2, const double _b2, const double _r3, const double _g3,
                       const double _b3, const double _r4 = 1, const double _g4 = 1,
                       const double _b4 = 1) {
    v[0].r = _r1;
    v[0].g = _g1;
    v[0].b = _b1;
    v[1].r = _r2;
    v[1].g = _g2;
    v[1].b = _b2;
    v[2].r = _r3;
    v[2].g = _g3;
    v[2].b = _b3;
    v[3].r = _r4;
    v[3].g = _g4;
    v[3].b = _b4;
  }

  // General Transformation Methods, only operate on x,y,z,w of vertices
  void Transform(const Matrix &_m);
  void Translate(const Vector3D &_v);
  void Translate(const double _x, const double _y, const double _z);

  // Pipeline Transformation Methods
  void TransformToCamera(const Matrix &_m);
  void TransformToPerspective(const Matrix &_m);
  void TransformToPixel(const Matrix &_m);

  // Pipeline function methods
  void Clip();
  void HomogeneousDivide();
  void SetupRasterization();

  // Rasterization Methods
  void Rasterize();
  void Rasterize(const int y);
  void RasterizeFast();
  void RasterizeFast(const int y);

  // Helper Functions
  void YSort(Vertex *temp);
  void XSort(Vertex *temp);
  void ZSort(Vertex *temp);
  double MinX();
  double MinY();
  double MinZ();
  double MaxX();
  double MaxY();
  double MaxZ();

  // Operator Overloads
  Poly &operator=(const Poly &rhs);
  bool operator==(const Poly &rhs) const; // Incomplete implementation!
};
