#pragma once

#include "matrix.hpp"
#include "point.hpp"

class Camera {

private:
  double theta, phi;
  Vector3D forward;
  Vector3D up;
  Vector3D right;

  Vector3D position;

public:
  Camera();

  void Normalize();

  void ComputeAxes();

  void Translate(const Vector3D &v);

  void Rotate(const double _t, const double _p);

  Matrix GetWorldToCamera();

  double GetPhi() const { return phi; }
  double GetTheta() const { return theta; }
  void SetAngles(const double _t, const double _p);

  Point3D GetPosition() const;
  void SetPosition(const double x, const double y, const double z);

  Point3D GetForward() const;
  void SetForward(const double x, const double y, const double z);

  Point3D GetUp() const;
  void SetUp(const double x, const double y, const double z);

  Point3D GetRight() const;
  void SetRight(const double x, const double y, const double z);

  bool operator==(const Camera &c) {
    if (phi != c.GetPhi())
      return false;
    if (theta != c.GetTheta())
      return false;
    if (position != c.GetPosition())
      return false;
    return true;
  }
  bool operator!=(const Camera &c) {
    if (phi != c.GetPhi())
      return true;
    if (theta != c.GetTheta())
      return true;
    if (position != c.GetPosition())
      return true;
    return false;
  }
};
