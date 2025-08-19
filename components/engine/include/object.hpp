#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "polygon.hpp"

class Object {
public:
  // Constructor
  Object();

  // Alternate Constructor
  Object(const unsigned short *texture, const int texWid, const int texHgt,
         const Vector3D &vel = Vector3D(0, 0, 0), Point3D pos = Point3D(0, 0, 0), float _rx = 0,
         float _ry = 0, float _rz = 0);

  // Alternate Constructor
  Object(Poly &poly, const unsigned short *texture, const int texWid, const int texHgt,
         const Vector3D &vel = Vector3D(0, 0, 0), Point3D pos = Point3D(0, 0, 0), float _rx = 0,
         float _ry = 0, float _rz = 0);

  // Destructor
  ~Object() {}

  // Updates Temp last with any changes to the master list
  bool updateList();

  // Updates Temp list to whatever list is passed (i.e. Render list)
  bool updateList(const std::vector<Poly> &poly);

  // add polygon to lists
  void add(const Poly &poly);

  // Generates cube with with sidelength = size*2
  void GenerateCube(float size = 5);

  // Generates tetrahedron
  void GenerateTetra(float size = 5);

  // Generates a floor at depth, with sidelength = length
  void GenerateFloor(float length = 50, float depth = -10);

  // Generates a ceiling at depth, with sidelength = length
  void GenerateCeiling(float length, float depth);

  // Generates wall
  // Wall types: 0 = front: faces the player init
  ///////////// 1 = back: behind player init
  ///////////// 2 = left: the left of player init
  ///////////// 3 = right: to the right of player init
  void GenerateWall(size_t type, float length = 50, float depth = -10);

  void GenerateShot(const Vector3D &pos, float theta_, float phi_);

  void GeneratePlayer(const Vector3D &pos, float theta_, float phi_,
                      const unsigned short *texture = nullptr, const int texWid = 0,
                      const int texHgt = 0);

  // fileParser()<-- future function

  bool UpdateTime(int time);

  bool SetVelocity(const Vector3D &vector);

  bool SetPosition(const Point3D &pos);
  Point3D GetPosition(void) const;

  bool SetBoudingEllipsoid(float x, float y, float z);
  float GetRadiusX(void) const;
  float GetRadiusY(void) const;
  float GetRadiusZ(void) const;

  // sets rendertype for all polygons
  bool SetRenderType(RenderType rt);

  // Master list operations
  void RotateToHeading();
  void RotateToHeading(const Vector3D &changeUp);
  void Rotate(Matrix &m);
  void Translate(Vector3D &v);

  // Temp list operations
  void clearTemp();
  void RotateTemp(const Matrix &m);
  void TranslateTemp(const Vector3D &v);
  void RotateTempToHeading();

  // Pipeline functions
  void TransformToCamera(Matrix &m);
  void TransformToPerspective(Matrix &m);
  void TransformToPixel(Matrix &m);
  std::vector<Poly> GetRenderList() const;
  std::vector<Poly> GetTemp() const;

  void projectileInit(const Vector3D &head, const Vector3D &pos = Vector3D(0, 0, 0));

  bool CollidesWith(const Object &b);

private:
  std::vector<Poly> master;
  std::vector<Poly> temp;
  Point3D position;
  Vector3D heading, velocity;
  float theta, phi;
  float rx, ry, rz;
  const unsigned short *tex;
  int texWidth;
  int texHeight;
  size_t counter;
  bool kill;
};
