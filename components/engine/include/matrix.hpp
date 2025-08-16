#pragma once

#include "point.hpp"

#include <math.h>

class Matrix {
public:
  double data[4][4];
  Matrix() { SetIdentity(); }
  Matrix(const Matrix &m);
  ~Matrix() {}

  void Print(int x, int y);

  void SetIdentity();
  void SetRotation(double x, double y, double z); // Euler angle rotation
  void SetRotation(double theta, Vector3D u);     // Rotate by theta about vector u
  void Clear();
  Matrix Transpose(void) const;

  Matrix operator-() const;
  Matrix &operator=(const Matrix &rhs);
  Matrix operator*(const double rhs) const;
  Matrix operator/(const double rhs) const;
  Matrix operator*(const Matrix &rhs) const;
  Matrix operator+(const Matrix &rhs) const;
  Matrix operator-(const Matrix &rhs) const;

  Vector3D operator*(const Vector3D rhs) const;
  bool operator!=(const Matrix &rhs) const;
};
