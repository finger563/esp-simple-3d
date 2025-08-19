#pragma once

#include "matrix.hpp"
#include "point.hpp"

class Camera {

private:
  float theta, phi;
  Vector3D forward;
  Vector3D up;
  Vector3D right;

  Vector3D position;

public:
  Camera();

  void Normalize();

  void ComputeAxes();

  void Translate(const Vector3D &v);

  void Rotate(const float _t, const float _p);

  Matrix GetWorldToCamera();

  float GetPhi() const { return phi; }
  float GetTheta() const { return theta; }
  void SetAngles(const float _t, const float _p);

  Point3D GetPosition() const;
  void SetPosition(const float x, const float y, const float z);

  Point3D GetForward() const;
  void SetForward(const float x, const float y, const float z);

  Point3D GetUp() const;
  void SetUp(const float x, const float y, const float z);

  Point3D GetRight() const;
  void SetRight(const float x, const float y, const float z);

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
