#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Note: 3D and 2D (e.g. Point3D and Point2D) are with respect to world space
// coordinates.  All types of points/vectors are stored in homogeneous coordinates
// which encapsulates the 2/3D point in a higher dimension to become 3/4D.
// This is done for mathematical ease of use and correctness with respect to
// projection.

class Point3D {
public:
  float x, y, z, w; // 3D coords of point
  Point3D()
      : x(0)
      , y(0)
      , z(0)
      , w(1) {}
  Point3D(const float _x, const float _y, const float _z)
      : x(_x)
      , y(_y)
      , z(_z)
      , w(1) {}
  Point3D(const float _x, const float _y, const float _z, const float _w)
      : x(_x)
      , y(_y)
      , z(_z)
      , w(_w) {}
  Point3D(const Point3D &rhs) = default;

  float MagnitudeSquared() const { return (x * x + y * y + z * z); }
  float Magnitude() const { return sqrtf(MagnitudeSquared()); }

  float Dot(const Point3D &rhs) const { return (x * rhs.x + y * rhs.y + z * rhs.z); }
  Point3D Cross(const Point3D &rhs) const {
    return Point3D((y * rhs.z - z * rhs.y), (z * rhs.x - x * rhs.z), (x * rhs.y - y * rhs.x));
  }

  Point3D Normalize() const {
    float mag = Magnitude();
    return Point3D(x / mag, y / mag, z / mag, w);
  }

  Point3D &operator=(const Point3D &rhs) = default;
  bool operator!=(const Point3D &rhs) const;
  bool operator==(const Point3D &rhs) const { return !(*this != rhs); }

  Point3D operator-() const { return Point3D(-x, -y, -z, w); }
  Point3D operator*(const float rhs) const;
  Point3D operator/(const float rhs) const;
  float operator*(const Point3D &rhs) const;
  Point3D operator+(const Point3D &rhs) const;
  Point3D operator-(const Point3D &rhs) const;
};

typedef Point3D Vector3D;

inline static Vector3D Cross(const Vector3D &a, const Vector3D &b) { return a.Cross(b); }

inline static float magnitude_squared(const Vector3D &v) { return v.MagnitudeSquared(); }

inline static float magnitude(const Vector3D &v) { return v.Magnitude(); }

inline static Vector3D normalize(const Vector3D &v) { return v.Normalize(); }
