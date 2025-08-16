#pragma once

#include "matrix.hpp"

enum TriangleType { // left/right directions w.r.t s2
  FLAT_TOP_RIGHT,
  FLAT_TOP_LEFT,
  FLAT_BOTTOM_RIGHT,
  FLAT_BOTTOM_LEFT,
  NORMAL_RIGHT,
  NORMAL_LEFT
};

class Triangle {
public:
  Point3D a, b, c; // object coordinates for triangle
  Vector3D normal;
  short color;
  TriangleType type; // for fast rendering, pre-compute
  bool visible;      // for backface culling

  // Screen space variables for Z-buffering
  Point3D s1, s2, s3, s4;   // screen space coordinates for triangle (y-sorted)
  float m12, m13, m23;      // screen space slopes (dx/dy)
  float sx, ex;             // start and end x-pixel coordinates
  float syl, eyl, syr, eyr; // start and end y-pixel coordinates (not currently used)
  float sz, ez;             // start and end 1/z world coordinates of object (per scanline)
  float dzx, dz13, dz12, dz23, dz14, dz43; // z-slopes for 1/z-buffering

  // Variables for Texturing
  Point2D u, v, w;                         // texture coords for each point
  const unsigned short *texture;           // for texturing
  int texwidth;                            // width of texture
  Point2D u1, u2, u3, u4;                  // screen space coordinates for texture
  float stu, stv, etu, etv;                // start and end x & y texture coordinates per scanline
  float dtx13, dtx12, dtx23, dtx14, dtx43; // slopes for x texture coordinates
  float dty13, dty12, dty23, dty14, dty43; // slopes for y texture coordinates

  Triangle() {
    a = b = c = Point3D();
    normal = Vector3D();
  }
  Triangle(const Point3D _a, const Point3D _b, const Point3D _c, const Vector3D n, short clr) {
    a = _a;
    b = _b;
    c = _c;
    normal = n;
    color = clr;
  }
  Triangle(const Point3D _a, const Point3D _b, const Point3D _c, const Vector3D n, const Point2D _u,
           const Point2D _v, const Point2D _w) {
    a = _a;
    b = _b;
    c = _c;
    normal = n;
    u = _u;
    v = _v;
    w = _w;
  }
  ~Triangle() {}
  void Print() {}
  void SetTexture(const unsigned short *tex, const int width) {
    texture = tex;
    texwidth = width;
  }

  void Rotate(const Matrix &m);
  void Translate(const Vector3D &v);
  void Translate(const float x, const float y, const float z);

  void TransformToScreen(const Matrix &m);
  void DrawFilledZbuffer(const int y);
  void DrawTexturedZbuffer(const int y);
  // THIS IS FOR TESTING
  void TransformToScreenHomogeneous(const Matrix &m);
  void DrawTexturedZbufferHomogeneous(const int y);

  void DrawWireframe(const Matrix &m);
  void DrawFilled(void);
};
