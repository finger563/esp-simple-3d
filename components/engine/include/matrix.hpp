#pragma once

#include "point.hpp"

#include <math.h>

class Matrix {
public:
  float data[4][4];
  Matrix() { SetIdentity(); }
  Matrix(const Matrix &m);

  static Matrix RotationAxisAngle(float theta, const Vector3D &u) {
    Matrix m;
    m.SetRotation(theta, u);
    return m;
  }

  static Matrix Identity() {
    Matrix m;
    m.SetIdentity();
    return m;
  }

  static Matrix LookVector(const Vector3D &eye, const Vector3D &look, const Vector3D &up);

  static Matrix Translation(const Vector3D &t) { return Translation(t.x, t.y, t.z); }

  static Matrix Translation(float tx, float ty, float tz) {
    Matrix m;
    m.SetIdentity();
    m[3][0] = tx;
    m[3][1] = ty;
    m[3][2] = tz;
    return m;
  }

  void Clear();
  void SetIdentity();
  void SetRotation(float x, float y, float z);      // Euler angle rotation
  void SetRotation(float theta, const Vector3D &u); // Rotate by theta about vector u

  Matrix Transpose(void) const;
  float Determinant(void) const;
  float Cofactor(int row, int col) const;
  Matrix Inverse(void) const;

  Matrix &operator=(const Matrix &rhs);
  bool operator!=(const Matrix &rhs) const;

  Matrix operator-() const;
  Matrix operator*(const float rhs) const;
  Matrix operator/(const float rhs) const;
  Matrix operator*(const Matrix &rhs) const;
  Matrix operator+(const Matrix &rhs) const;
  Matrix operator-(const Matrix &rhs) const;
  Vector3D operator*(const Vector3D &rhs) const;

  // Allow access to array data as Matrix[row] by returning a pointer to the row array
  const float (&operator[](int row) const)[4] { return data[row]; }
  // Allow access to array data as Matrix(row, col) by returning a reference to
  // the element at that position
  const float &operator()(int row, int col) const { return data[row][col]; }
  // Allow access to array data as Matrix[row][col] by returning a pointer to
  // the row array
  float (&operator[](int row))[4] { return data[row]; }
  // Allow access to array data as Matrix[row, col] by returning a reference to
  // the element at that position
  float &operator[](int row, int col) { return data[row][col]; }
};

[[maybe_unused]] static Matrix operator*(const float lhs, const Matrix &rhs) { return rhs * lhs; }

[[maybe_unused]] static Vector3D operator*(const Vector3D &lhs, const Matrix &rhs) {
  return rhs * lhs;
}
