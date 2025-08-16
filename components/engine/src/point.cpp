#include "point.hpp"

bool Point3D::operator!=(const Point3D &rhs) const {
  if (x != rhs.x)
    return false;
  if (y != rhs.y)
    return false;
  if (z != rhs.z)
    return false;
  if (w != rhs.w)
    return false;
  return true;
}

Point3D Point3D::operator*(const double rhs) const {
  return Point3D(x * rhs, y * rhs, z * rhs); //,w*rhs);
}

Point3D Point3D::operator/(const double rhs) const {
  return Point3D(x / rhs, y / rhs, z / rhs, w / rhs);
}

double Point3D::operator*(const Point3D &rhs) const {
  return (x * rhs.x + y * rhs.y + z * rhs.z); // + w*rhs.w);
}

Point3D Point3D::operator+(const Point3D &rhs) const {
  return Point3D(x + rhs.x, y + rhs.y, z + rhs.z);
}

Point3D Point3D::operator-(const Point3D &rhs) const {
  return Point3D(x - rhs.x, y - rhs.y, z - rhs.z);
}

// POINT2D

Point2D Point2D::operator*(const double rhs) const { return Point2D(x * rhs, y * rhs, w * rhs); }

Point2D Point2D::operator/(const double rhs) const { return Point2D(x / rhs, y / rhs, w / rhs); }

double Point2D::operator*(const Point2D &rhs) const { return (x * rhs.x + y * rhs.y); }

Point2D Point2D::operator+(const Point2D &rhs) const { return Point2D(x + rhs.x, y + rhs.y); }

Point2D Point2D::operator-(const Point2D &rhs) const { return Point2D(x - rhs.x, y - rhs.y); }
