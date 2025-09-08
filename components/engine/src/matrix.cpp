#include "matrix.hpp"

Matrix::Matrix(const Matrix &m) {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      data[row][col] = m[row][col];
    }
  }
}

void Matrix::SetIdentity() {
  Clear();
  for (int row = 0; row < 4; row++) {
    data[row][row] = 1;
  }
}

void Matrix::SetRotation(float x, float y, float z) {
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

void Matrix::SetRotation(float theta, const Vector3D &u) {
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

Matrix Matrix::LookVector(const Vector3D &eye, const Vector3D &look, const Vector3D &up) {

  Vector3D f = (look - eye).Normalize();
  Vector3D s = f.Cross(up).Normalize();
  Vector3D u = s.Cross(f);

  Matrix m;
  m[0][0] = s.x;
  m[0][1] = s.y;
  m[0][2] = s.z;

  m[1][0] = u.x;
  m[1][1] = u.y;
  m[1][2] = u.z;

  m[2][0] = -f.x;
  m[2][1] = -f.y;
  m[2][2] = -f.z;

  m[3][0] = -s.Dot(eye);
  m[3][1] = -u.Dot(eye);
  m[3][2] = f.Dot(eye);

  return m;
}

void Matrix::Clear() {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      data[row][col] = 0.0f;
    }
  }
}

Matrix Matrix::Transpose(void) const {
  Matrix m;
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m[row][col] = data[col][row];
    }
  }
  return m;
}

float Matrix::Determinant(void) const {
  float det = 0.0f;
  for (int col = 0; col < 4; col++) {
    det += data[0][col] * Cofactor(0, col);
  }
  return det;
}

float Matrix::Cofactor(int row, int col) const {
  Matrix minor;
  int minorRow = 0;
  for (int r = 0; r < 4; r++) {
    if (r == row)
      continue;
    int minorCol = 0;
    for (int c = 0; c < 4; c++) {
      if (c == col)
        continue;
      minor[minorRow][minorCol] = data[r][c];
      minorCol++;
    }
    minorRow++;
  }

  float det = minor[0][0] * (minor[1][1] * minor[2][2] - minor[1][2] * minor[2][1]) -
              minor[0][1] * (minor[1][0] * minor[2][2] - minor[1][2] * minor[2][0]) +
              minor[0][2] * (minor[1][0] * minor[2][1] - minor[1][1] * minor[2][0]);

  return ((row + col) % 2 == 0) ? det : -det;
}

Matrix Matrix::Inverse(void) const {

  Matrix m;
  float det = Determinant();
  if (det == 0.0f) {
    return m; // Return zero matrix if not invertible
  }

  float invDet = 1.0f / det;

  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m[col][row] = Cofactor(row, col) * invDet;
    }
  }

  return m;
}

Matrix Matrix::operator-() const {
  Matrix m;
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m[row][col] = -data[row][col];
    }
  }
  return m;
}

Matrix &Matrix::operator=(const Matrix &rhs) {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      data[row][col] = rhs[row][col];
    }
  }
  return (*this);
}

bool Matrix::operator!=(const Matrix &rhs) const {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      if (data[row][col] != rhs[row][col])
        return true;
    }
  }
  return false;
}

Matrix Matrix::operator*(const float rhs) const {
  Matrix m(*this);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m[row][col] *= rhs;
    }
  }
  return m;
}

Matrix Matrix::operator/(const float rhs) const {
  Matrix m(*this);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m[row][col] /= rhs;
    }
  }
  return m;
}

Matrix Matrix::operator*(const Matrix &rhs) const {
  Matrix m;
  m[0][0] = data[0][0] * rhs[0][0] + data[0][1] * rhs[1][0] + data[0][2] * rhs[2][0] +
            data[0][3] * rhs[3][0];
  m[0][1] = data[0][0] * rhs[0][1] + data[0][1] * rhs[1][1] + data[0][2] * rhs[2][1] +
            data[0][3] * rhs[3][1];
  m[0][2] = data[0][0] * rhs[0][2] + data[0][1] * rhs[1][2] + data[0][2] * rhs[2][2] +
            data[0][3] * rhs[3][2];
  m[0][3] = data[0][0] * rhs[0][3] + data[0][1] * rhs[1][3] + data[0][2] * rhs[2][3] +
            data[0][3] * rhs[3][3];

  m[1][0] = data[1][0] * rhs[0][0] + data[1][1] * rhs[1][0] + data[1][2] * rhs[2][0] +
            data[1][3] * rhs[3][0];
  m[1][1] = data[1][0] * rhs[0][1] + data[1][1] * rhs[1][1] + data[1][2] * rhs[2][1] +
            data[1][3] * rhs[3][1];
  m[1][2] = data[1][0] * rhs[0][2] + data[1][1] * rhs[1][2] + data[1][2] * rhs[2][2] +
            data[1][3] * rhs[3][2];
  m[1][3] = data[1][0] * rhs[0][3] + data[1][1] * rhs[1][3] + data[1][2] * rhs[2][3] +
            data[1][3] * rhs[3][3];

  m[2][0] = data[2][0] * rhs[0][0] + data[2][1] * rhs[1][0] + data[2][2] * rhs[2][0] +
            data[2][3] * rhs[3][0];
  m[2][1] = data[2][0] * rhs[0][1] + data[2][1] * rhs[1][1] + data[2][2] * rhs[2][1] +
            data[2][3] * rhs[3][1];
  m[2][2] = data[2][0] * rhs[0][2] + data[2][1] * rhs[1][2] + data[2][2] * rhs[2][2] +
            data[2][3] * rhs[3][2];
  m[2][3] = data[2][0] * rhs[0][3] + data[2][1] * rhs[1][3] + data[2][2] * rhs[2][3] +
            data[2][3] * rhs[3][3];

  m[3][0] = data[3][0] * rhs[0][0] + data[3][1] * rhs[1][0] + data[3][2] * rhs[2][0] +
            data[3][3] * rhs[3][0];
  m[3][1] = data[3][0] * rhs[0][1] + data[3][1] * rhs[1][1] + data[3][2] * rhs[2][1] +
            data[3][3] * rhs[3][1];
  m[3][2] = data[3][0] * rhs[0][2] + data[3][1] * rhs[1][2] + data[3][2] * rhs[2][2] +
            data[3][3] * rhs[3][2];
  m[3][3] = data[3][0] * rhs[0][3] + data[3][1] * rhs[1][3] + data[3][2] * rhs[2][3] +
            data[3][3] * rhs[3][3];
  return m;
}

Matrix Matrix::operator+(const Matrix &rhs) const {
  Matrix m(*this);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m[row][col] += rhs[row][col];
    }
  }
  return m;
}

Matrix Matrix::operator-(const Matrix &rhs) const {
  Matrix m(*this);
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      m[row][col] -= rhs[row][col];
    }
  }
  return m;
}

Vector3D Matrix::operator*(const Vector3D &rhs) const {
  Vector3D v;
  v.x = rhs.x * data[0][0] + rhs.y * data[1][0] + rhs.z * data[2][0] + rhs.w * data[3][0];
  v.y = rhs.x * data[0][1] + rhs.y * data[1][1] + rhs.z * data[2][1] + rhs.w * data[3][1];
  v.z = rhs.x * data[0][2] + rhs.y * data[1][2] + rhs.z * data[2][2] + rhs.w * data[3][2];
  v.w = rhs.x * data[0][3] + rhs.y * data[1][3] + rhs.z * data[2][3] + rhs.w * data[3][3];
  return v;
}
