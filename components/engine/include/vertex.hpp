#pragma once

#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.hpp"
#include "matrix.hpp"

#define NUM_VERTEX_DATA 16

class Vertex {
public:
  union {
    struct {
      double x, y, z, w; // 3D homogeneous coords ( get transformed into pixel space )
      double ex, ey, ez; // 3D homogenous eye/camera coords ( w for this system is hw )
      double u, v;       // 2D texture coords (range = [0,1]) ( w for this system is hw )
      double r, g, b;    // RGB color (range = [0,1]), ( w for this system is hw )
      double nx, ny, nz; // components of normal vector ( w for this system is hw )
      double hw;         // homogeneous coordinate for interpolated values ( 1/w )
    };
    double data[NUM_VERTEX_DATA];
  };

  Vertex() {
    for (int i = 0; i < NUM_VERTEX_DATA; i++)
      data[i] = 0;
    w = hw = 1;
  }
  Vertex(const double _x, const double _y, const double _z, const double _w = 1,
         const double _u = 0, const double _v = 0) {
    x = _x;
    y = _y;
    z = _z;
    w = _w;
    ex = ey = ez = 0;
    u = _u;
    v = _v;
    r = g = b = 1; // default color is white
    nx = ny = nz = 0;
    hw = _w;
  }
  ~Vertex() {}

  void Print() {}

  // Variable Setting Methods
  void SetColor(const double _r, const double _g, const double _b) {
    r = _r;
    g = _g;
    b = _b;
  }
  void SetTextureCoords(const double _u, const double _v) {
    u = _u;
    v = _v;
  }
  void SetEyeCoords(const double _x, const double _y, const double _z) {
    ex = _x;
    ey = _y;
    ez = _z;
  }
  void SetNormalCoords(const double _x, const double _y, const double _z) {
    nx = _x;
    ny = _y;
    nz = _z;
  }

  // General Transformation Methods, only operate on x,y,z,w
  void Transform(const Matrix &_m);
  void Translate(const Vector3D &_v);
  void Translate(const double _x, const double _y, const double _z);

  // Pipeline Transformation Methods
  void TransformToCamera(const Matrix &_m);
  void TransformToPerspective(const Matrix &_m);
  void HomogeneousDivide();

  // Operator Overloads
  Vertex &operator=(const Vertex &rhs);
  bool operator==(const Vertex &rhs) const;
  bool operator!=(const Vertex &rhs) const;
  double &operator[](const int i) { return data[i]; }
  double operator[](const int i) const { return data[i]; }
  Vertex operator-(const Vertex &rhs) const;
  Vertex operator+(const Vertex &rhs) const;
  Vertex operator*(const double rhs) const;
  Vertex operator/(const double rhs) const;
};
