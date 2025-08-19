#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.hpp"

// Note: 3D and 2D (e.g. Point3D and Point2D) are with respect to world space
// coordinates.  All types of points/vectors are stored in homogeneous coordinates
// which encapsulates the 2/3D point in a higher dimension to become 3/4D.
// This is done for mathematical ease of use and correctness with respect to
// projection.

class Point3D {
public:
  float x, y, z, w; // 3D coords of point
  Point3D() {
    x = y = z = 0;
    w = 1;
  }
  Point3D(const float _x, const float _y, const float _z) {
    x = _x;
    y = _y;
    z = _z;
    w = 1;
  }
  Point3D(const float _x, const float _y, const float _z, const float _w) {
    x = _x;
    y = _y;
    z = _z;
    w = _w;
  }
  Point3D(const Point3D &rhs) = default;

  ~Point3D() {}

  void Print() {}

  Point3D operator-() const { return Point3D(-x, -y, -z, w); }

  Point3D &operator=(const Point3D &rhs) = default;

  Point3D operator*(const float rhs) const;

  Point3D operator/(const float rhs) const;

  float operator*(const Point3D &rhs) const;

  Point3D operator+(const Point3D &rhs) const;

  Point3D operator-(const Point3D &rhs) const;

  bool operator!=(const Point3D &rhs) const;
};

typedef Point3D Vector3D;

inline static Vector3D Cross(const Vector3D &a, const Vector3D &b) {
  return Vector3D(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline static Vector3D normalize(const Vector3D &rhs) {
  float mag = sqrtf(rhs.x * rhs.x + rhs.y * rhs.y + rhs.z * rhs.z);

  return Vector3D(rhs.x / mag, rhs.y / mag, rhs.z / mag);
}

inline static float magnitude(const Vector3D &rhs) {
  return sqrtf(rhs.x * rhs.x + rhs.y * rhs.y + rhs.z * rhs.z);
}

inline static Vector3D crossProduct(const Vector3D &a, const Vector3D &b) {
  return Vector3D((a.y * b.z - a.z * b.y), (a.z * b.x - a.x * b.z), (a.x * b.y - a.y * b.x));
}

class Point2D {
public:
  float x, y, w; // 2D coords of point
  Point2D() {
    x = y = 0;
    w = 1;
  }
  Point2D(const Point2D &rhs) = default;
  Point2D(const float _x, const float _y) {
    x = _x;
    y = _y;
    w = 1;
  }
  Point2D(const float _x, const float _y, const float _w) {
    x = _x;
    y = _y;
    w = _w;
  }
  ~Point2D() {}

  void Print() {}

  Point2D operator-() const { return Point2D(-x, -y, w); }

  Point2D &operator=(const Point2D &rhs) = default;

  Point2D operator*(const float rhs) const;

  Point2D operator/(const float rhs) const;

  float operator*(const Point2D &rhs) const;

  Point2D operator+(const Point2D &rhs) const;

  Point2D operator-(const Point2D &rhs) const;
};

typedef Point2D Vector2D;

inline static float magnitude(const Vector2D &rhs) { return sqrtf(rhs.x * rhs.x + rhs.y * rhs.y); }
