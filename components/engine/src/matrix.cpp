#include "matrix.hpp"

Matrix::Matrix(const Matrix &m) {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      data[row][col] = m.data[row][col];
    }
  }
}

void Matrix::Print(int x, int y) {}

void Matrix::SetIdentity() {
  Clear();
  for (int row = 0; row < 4; row++) {
    data[row][row] = 1;
  }
}

void Matrix::SetRotation(double x, double y, double z) {
  SetIdentity();
  data[0][0] = cos(y) * cos(z);
  data[0][1] = cos(x) * sin(z) + sin(x) * sin(y) * cos(z);
  data[0][2] = sin(x) * sin(z) - cos(x) * sin(y) * cos(z);
  data[1][0] = -cos(y) * sin(z);
  data[1][1] = cos(x) * cos(z) - sin(x) * sin(y) * sin(z);
  data[1][2] = sin(x) * cos(z) + cos(x) * sin(y) * sin(z);
  data[2][0] = sin(y);
  data[2][1] = -sin(x) * cos(y);
  data[2][2] = cos(x) * cos(y);
}

void Matrix::SetRotation(double theta, Vector3D u) {
  SetIdentity();
  data[0][0] = cos(theta) + u.x * u.x * (1 - cos(theta));
  data[0][1] = u.x * u.y * (1 - cos(theta)) - u.z * sin(theta);
  data[0][2] = u.x * u.z * (1 - cos(theta)) + u.y * sin(theta);
  data[1][0] = u.y * u.x * (1 - cos(theta)) + u.z * sin(theta);
  data[1][1] = cos(theta) + u.y * u.y * (1 - cos(theta));
  data[1][2] = u.y * u.z * (1 - cos(theta)) - u.x * sin(theta);
  data[2][0] = u.z * u.x * (1 - cos(theta)) - u.y * sin(theta);
  data[2][1] = u.z * u.y * (1 - cos(theta)) + u.x * sin(theta);
  data[2][2] = cos(theta) + u.z * u.z * (1 - cos(theta));
}

void Matrix::Clear() {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      data[row][col] = 0;
    }
  }
}

Matrix Matrix::Transpose(void) const {
  Matrix m;
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m.data[row][col] = data[col][row];
    }
  }
  return m;
}

Matrix Matrix::operator-() const {
  Matrix m;
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m.data[row][col] = -data[row][col];
    }
  }
  return m;
}

Matrix &Matrix::operator=(const Matrix &rhs) {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      data[row][col] = rhs.data[row][col];
    }
  }
  return (*this);
}

bool Matrix::operator!=(const Matrix &rhs) const {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      if (data[row][col] != rhs.data[row][col])
        return false;
    }
  }
  return true;
}

Matrix Matrix::operator*(const double rhs) const {
  Matrix m(*this);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m.data[row][col] *= rhs;
    }
  }
  return m;
}

Matrix Matrix::operator/(const double rhs) const {
  Matrix m(*this);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m.data[row][col] /= rhs;
    }
  }
  return m;
}

Matrix Matrix::operator*(const Matrix &rhs) const {
  Matrix m;
  m.data[0][0] = data[0][0] * rhs.data[0][0] + data[0][1] * rhs.data[1][0] +
                 data[0][2] * rhs.data[2][0] + data[0][3] * rhs.data[3][0];
  m.data[0][1] = data[0][0] * rhs.data[0][1] + data[0][1] * rhs.data[1][1] +
                 data[0][2] * rhs.data[2][1] + data[0][3] * rhs.data[3][1];
  m.data[0][2] = data[0][0] * rhs.data[0][2] + data[0][1] * rhs.data[1][2] +
                 data[0][2] * rhs.data[2][2] + data[0][3] * rhs.data[3][2];
  m.data[0][3] = data[0][0] * rhs.data[0][3] + data[0][1] * rhs.data[1][3] +
                 data[0][2] * rhs.data[2][3] + data[0][3] * rhs.data[3][3];

  m.data[1][0] = data[1][0] * rhs.data[0][0] + data[1][1] * rhs.data[1][0] +
                 data[1][2] * rhs.data[2][0] + data[1][3] * rhs.data[3][0];
  m.data[1][1] = data[1][0] * rhs.data[0][1] + data[1][1] * rhs.data[1][1] +
                 data[1][2] * rhs.data[2][1] + data[1][3] * rhs.data[3][1];
  m.data[1][2] = data[1][0] * rhs.data[0][2] + data[1][1] * rhs.data[1][2] +
                 data[1][2] * rhs.data[2][2] + data[1][3] * rhs.data[3][2];
  m.data[1][3] = data[1][0] * rhs.data[0][3] + data[1][1] * rhs.data[1][3] +
                 data[1][2] * rhs.data[2][3] + data[1][3] * rhs.data[3][3];

  m.data[2][0] = data[2][0] * rhs.data[0][0] + data[2][1] * rhs.data[1][0] +
                 data[2][2] * rhs.data[2][0] + data[2][3] * rhs.data[3][0];
  m.data[2][1] = data[2][0] * rhs.data[0][1] + data[2][1] * rhs.data[1][1] +
                 data[2][2] * rhs.data[2][1] + data[2][3] * rhs.data[3][1];
  m.data[2][2] = data[2][0] * rhs.data[0][2] + data[2][1] * rhs.data[1][2] +
                 data[2][2] * rhs.data[2][2] + data[2][3] * rhs.data[3][2];
  m.data[2][3] = data[2][0] * rhs.data[0][3] + data[2][1] * rhs.data[1][3] +
                 data[2][2] * rhs.data[2][3] + data[2][3] * rhs.data[3][3];

  m.data[3][0] = data[3][0] * rhs.data[0][0] + data[3][1] * rhs.data[1][0] +
                 data[3][2] * rhs.data[2][0] + data[3][3] * rhs.data[3][0];
  m.data[3][1] = data[3][0] * rhs.data[0][1] + data[3][1] * rhs.data[1][1] +
                 data[3][2] * rhs.data[2][1] + data[3][3] * rhs.data[3][1];
  m.data[3][2] = data[3][0] * rhs.data[0][2] + data[3][1] * rhs.data[1][2] +
                 data[3][2] * rhs.data[2][2] + data[3][3] * rhs.data[3][2];
  m.data[3][3] = data[3][0] * rhs.data[0][3] + data[3][1] * rhs.data[1][3] +
                 data[3][2] * rhs.data[2][3] + data[3][3] * rhs.data[3][3];
  return m;
}

Matrix Matrix::operator+(const Matrix &rhs) const {
  Matrix m(*this);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m.data[row][col] += rhs.data[row][col];
    }
  }
  return m;
}

Matrix Matrix::operator-(const Matrix &rhs) const {
  Matrix m(*this);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m.data[row][col] -= rhs.data[row][col];
    }
  }
  return m;
}

Vector3D Matrix::operator*(const Vector3D rhs) const {
  Vector3D v;
  v.x = rhs.x * data[0][0] + rhs.y * data[1][0] + rhs.z * data[2][0] + rhs.w * data[3][0];
  v.y = rhs.x * data[0][1] + rhs.y * data[1][1] + rhs.z * data[2][1] + rhs.w * data[3][1];
  v.z = rhs.x * data[0][2] + rhs.y * data[1][2] + rhs.z * data[2][2] + rhs.w * data[3][2];
  v.w = rhs.x * data[0][3] + rhs.y * data[1][3] + rhs.z * data[2][3] + rhs.w * data[3][3];
  return v;
}
