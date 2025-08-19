#include "camera.hpp"

#include <math.h>

Camera::Camera()
    : theta(0)
    , phi(0)
    , forward(0, 0, 1)
    , up(0, 1, 0)
    , right(1, 0, 0)
    , position(0, 0, 0) {}

void Camera::Normalize() {
  up = normalize(up);
  forward = normalize(forward);
  right = normalize(right);
}

void Camera::Translate(const Vector3D &v) {
  position = right * (v.x) + up * (v.y) + forward * (v.z) + position;
}

void Camera::ComputeAxes() {
  float r = cosf(phi);
  float x = r * sinf(theta), y = sinf(phi), z = r * cosf(theta);
  forward = normalize(Vector3D(x, y, z));
  up = normalize(Vector3D(0, 1, 0));
  right = normalize(Cross(up, forward));
  up = normalize(Cross(forward, right));
}

void Camera::SetAngles(const float _t, const float _p) {
  theta = _t;
  if (theta > 2.0 * 3.141592) {
    theta = theta - 2.0 * 3.141592;
  } else if (theta < -2.0 * 3.141592) {
    theta = theta + 2.0 * 3.141592;
  }
  phi = _p;
  if (phi > 3.141592 / 2.0) {
    phi = 3.141592 / 2.0;
  } else if (phi < -3.141592 / 2.0) {
    phi = -3.141592 / 2.0;
  }
  ComputeAxes();
}

void Camera::Rotate(const float _t, const float _p) {
  theta += _t;
  if (theta > 2.0 * 3.141592) {
    theta = theta - 2.0 * 3.141592;
  } else if (theta < -2.0 * 3.141592) {
    theta = theta + 2.0 * 3.141592;
  }
  phi += _p;
  if (phi > 3.141592 / 2.0) {
    phi = 3.141592 / 2.0;
  } else if (phi < -3.141592 / 2.0) {
    phi = -3.141592 / 2.0;
  }
  ComputeAxes();
}

Matrix Camera::GetWorldToCamera() {
  Matrix m;
  m.data[0][0] = right.x;
  m.data[1][0] = right.y;
  m.data[2][0] = right.z;
  m.data[0][1] = up.x;
  m.data[1][1] = up.y;
  m.data[2][1] = up.z;
  m.data[0][2] = forward.x;
  m.data[1][2] = forward.y;
  m.data[2][2] = forward.z;
  return m;
}

Point3D Camera::GetPosition() const { return position; }

void Camera::SetPosition(const float x, const float y, const float z) {
  position.x = x;
  position.y = y;
  position.z = z;
}

Point3D Camera::GetForward() const { return forward; }

void Camera::SetForward(const float x, const float y, const float z) {
  forward.x = x;
  forward.y = y;
  forward.z = z;
}

Point3D Camera::GetUp() const { return up; }

void Camera::SetUp(const float x, const float y, const float z) {
  up.x = x;
  up.y = y;
  up.z = z;
}

Point3D Camera::GetRight() const { return right; }

void Camera::SetRight(const float x, const float y, const float z) {
  right.x = x;
  right.y = y;
  right.z = z;
}
