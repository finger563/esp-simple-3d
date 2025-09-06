#include "camera.hpp"

#include <math.h>

Camera::Camera()
    : theta(0)
    , phi(0)
    , forward(0, 0, 1)
    , up(0, 1, 0)
    , right(1, 0, 0)
    , position(0, 0, 0) {
  viewMatrix.SetIdentity();
}

void Camera::Translate(const Vector3D &v) {
  position = right * (v.x) + up * (v.y) + forward * (v.z) + position;
}

void Camera::UpdateViewMatrix() {
  // set the viewmatrix
  viewMatrix.SetIdentity();
  viewMatrix[0][0] = right.x;
  viewMatrix[0][1] = right.y;
  viewMatrix[0][2] = right.z;
  viewMatrix[1][0] = up.x;
  viewMatrix[1][1] = up.y;
  viewMatrix[1][2] = up.z;
  viewMatrix[2][0] = forward.x;
  viewMatrix[2][1] = forward.y;
  viewMatrix[2][2] = forward.z;
  viewMatrix[3][0] = position.x;
  viewMatrix[3][1] = position.y;
  viewMatrix[3][2] = position.z;
}

void Camera::ComputeAxes() {
  float r = cosf(phi);
  float x = r * sinf(theta), y = sinf(phi), z = r * cosf(theta);
  forward = normalize(Vector3D(x, y, z));
  up = normalize(Vector3D(0, 1, 0));
  right = normalize(Cross(up, forward));
  up = normalize(Cross(forward, right));

  UpdateViewMatrix();
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

Matrix Camera::GetWorldToCamera() { return viewMatrix.Inverse(); }

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

void Camera::LookAt(const Point3D &eyePos, const Point3D &target, const Vector3D &worldUp) {
  // Row-vector convention: forward points from camera into the scene.
  // Our TransformToCamera expects rows: right, up, forward. For a view matrix,
  // forward should align with the camera's viewing direction.
  Vector3D fwd = normalize(target - eyePos);
  Vector3D upn = normalize(worldUp);
  Vector3D rgt = normalize(Cross(upn, fwd));
  upn = normalize(Cross(fwd, rgt));

  position = eyePos;
  forward = fwd;
  right = rgt;
  up = upn;

  UpdateViewMatrix();
}

void Camera::SetViewMatrix(const Matrix &m) {
  viewMatrix = m;
  right = Vector3D(m[0][0], m[0][1], m[0][2]);
  up = Vector3D(m[1][0], m[1][1], m[1][2]);
  forward = Vector3D(m[2][0], m[2][1], m[2][2]);
  // Recover position from translation row under row-vectors convention
  float t0 = m[3][0], t1 = m[3][1], t2 = m[3][2];
  position = -(right * t0 + up * t1 + forward * t2);
}

void Camera::ApplyTransform(const Matrix &t) {
  viewMatrix = viewMatrix * t;
  SetViewMatrix(viewMatrix);
}
