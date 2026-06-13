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
  Matrix viewMatrix;          // camera-to-world transform
  Matrix worldToCameraMatrix; // cached world-to-camera transform

  void ComputeAxes();
  void UpdateViewMatrix();

public:
  Camera();

  void Translate(const Vector3D &v);

  void Rotate(const float _t, const float _p);

  const Matrix &GetWorldToCamera() const { return worldToCameraMatrix; }
  const Matrix &GetViewMatrix() const { return viewMatrix; }
  void SetViewMatrix(const Matrix &m);
  void ApplyTransform(const Matrix &t);

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

  // Set camera to look from eye toward target with given world-up
  void LookAt(const Point3D &eye, const Point3D &target,
              const Vector3D &worldUp = Vector3D(0, 1, 0));

  bool operator==(const Camera &c) const {
    return phi == c.phi && theta == c.theta && position == c.position;
  }
  bool operator!=(const Camera &c) const { return !(*this == c); }
};
