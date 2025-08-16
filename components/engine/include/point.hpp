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
  double x, y, z, w; // 3D coords of point
  Point3D() {
    x = y = z = 0;
    w = 1;
  }
  Point3D(const double _x, const double _y, const double _z) {
    x = _x;
    y = _y;
    z = _z;
    w = 1;
  }
  Point3D(const double _x, const double _y, const double _z, const double _w) {
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

  Point3D operator*(const double rhs) const;

  Point3D operator/(const double rhs) const;

  double operator*(const Point3D &rhs) const;

  Point3D operator+(const Point3D &rhs) const;

  Point3D operator-(const Point3D &rhs) const;

  bool operator!=(const Point3D &rhs) const;
};

typedef Point3D Vector3D;

inline static Vector3D Cross(const Vector3D &a, const Vector3D &b) {
  return Vector3D(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline static Vector3D normalize(const Vector3D &rhs) {
  double mag = sqrt(rhs.x * rhs.x + rhs.y * rhs.y + rhs.z * rhs.z);

  return Vector3D(rhs.x / mag, rhs.y / mag, rhs.z / mag);
}

inline static double magnitude(const Vector3D &rhs) {
  return sqrt(rhs.x * rhs.x + rhs.y * rhs.y + rhs.z * rhs.z);
}

inline static Vector3D crossProduct(const Vector3D &a, const Vector3D &b) {
  return Vector3D((a.y * b.z - a.z * b.y), (a.z * b.x - a.x * b.z), (a.x * b.y - a.y * b.x));
}

class Point2D {
public:
  double x, y, w; // 2D coords of point
  Point2D() {
    x = y = 0;
    w = 1;
  }
  Point2D(const Point2D &rhs) = default;
  Point2D(const double _x, const double _y) {
    x = _x;
    y = _y;
    w = 1;
  }
  Point2D(const double _x, const double _y, const double _w) {
    x = _x;
    y = _y;
    w = _w;
  }
  ~Point2D() {}

  void Print() {}

  Point2D operator-() const { return Point2D(-x, -y, w); }

  Point2D &operator=(const Point2D &rhs) = default;

  Point2D operator*(const double rhs) const;

  Point2D operator/(const double rhs) const;

  double operator*(const Point2D &rhs) const;

  Point2D operator+(const Point2D &rhs) const;

  Point2D operator-(const Point2D &rhs) const;
};

typedef Point2D Vector2D;

inline static double magnitude(const Vector2D &rhs) { return sqrt(rhs.x * rhs.x + rhs.y * rhs.y); }
