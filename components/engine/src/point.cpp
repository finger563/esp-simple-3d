#include "point.hpp"

bool Point3D::operator!=(const Point3D &rhs) const {
  if (x != rhs.x)
    return true;
  if (y != rhs.y)
    return true;
  if (z != rhs.z)
    return true;
  if (w != rhs.w)
    return true;
  return false;
}

Point3D Point3D::operator*(const float rhs) const {
  return Point3D(x * rhs, y * rhs, z * rhs); //,w*rhs);
}

Point3D Point3D::operator/(const float rhs) const {
  return Point3D(x / rhs, y / rhs, z / rhs, w / rhs);
}

float Point3D::operator*(const Point3D &rhs) const {
  return (x * rhs.x + y * rhs.y + z * rhs.z); // + w*rhs.w);
}

Point3D Point3D::operator+(const Point3D &rhs) const {
  return Point3D(x + rhs.x, y + rhs.y, z + rhs.z);
}

Point3D Point3D::operator-(const Point3D &rhs) const {
  return Point3D(x - rhs.x, y - rhs.y, z - rhs.z);
}

// POINT2D

Point2D Point2D::operator*(const float rhs) const { return Point2D(x * rhs, y * rhs, w * rhs); }

Point2D Point2D::operator/(const float rhs) const { return Point2D(x / rhs, y / rhs, w / rhs); }

float Point2D::operator*(const Point2D &rhs) const { return (x * rhs.x + y * rhs.y); }

Point2D Point2D::operator+(const Point2D &rhs) const { return Point2D(x + rhs.x, y + rhs.y); }

Point2D Point2D::operator-(const Point2D &rhs) const { return Point2D(x - rhs.x, y - rhs.y); }
