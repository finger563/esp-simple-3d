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
         Vector3D vel = Vector3D(0, 0, 0), Point3D pos = Point3D(0, 0, 0), double _rx = 0,
         double _ry = 0, double _rz = 0);

  // Alternate Constructor
  Object(Poly poly, const unsigned short *texture, const int texWid, const int texHgt,
         Vector3D vel = Vector3D(0, 0, 0), Point3D pos = Point3D(0, 0, 0), double _rx = 0,
         double _ry = 0, double _rz = 0);

  // Destructor
  ~Object() {}

  // Updates Temp last with any changes to the master list
  bool updateList();

  // Updates Temp list to whatever list is passed (i.e. Render list)
  bool updateList(std::vector<Poly> poly);

  // add polygon to lists
  void add(Poly poly);

  // Generates cube with with sidelength = size*2
  void GenerateCube(double size = 5);

  // Generates tetrahedron
  void GenerateTetra(double size = 5);

  // Generates a floor at depth, with sidelength = length
  void GenerateFloor(double length = 50, double depth = -10);

  // Generates a ceiling at depth, with sidelength = length
  void GenerateCeiling(double length, double depth);

  // Generates wall
  // Wall types: 0 = front: faces the player init
  ///////////// 1 = back: behind player init
  ///////////// 2 = left: the left of player init
  ///////////// 3 = right: to the right of player init
  void GenerateWall(size_t type, double length = 50, double depth = -10);

  void GenerateShot(Vector3D pos, double theta_, double phi_);

  void GeneratePlayer(Vector3D pos, double theta_, double phi_,
                      const unsigned short *texture = nullptr, const int texWid = 0,
                      const int texHgt = 0);

  // fileParser()<-- future function

  bool UpdateTime(int time);

  bool SetVelocity(const Vector3D &vector);

  bool SetPosition(const Point3D &pos);
  Point3D GetPosition(void) const;

  bool SetBoudingEllipsoid(double x, double y, double z);
  double GetRadiusX(void) const;
  double GetRadiusY(void) const;
  double GetRadiusZ(void) const;

  // sets rendertype for all polygons
  bool SetRenderType(RenderType rt);

  // Master list operations
  void RotateToHeading();
  void RotateToHeading(Vector3D changeUp);
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

  void projectileInit(Vector3D head, Vector3D pos = Vector3D(0, 0, 0));

  bool CollidesWith(const Object &b);

private:
  std::vector<Poly> master;
  std::vector<Poly> temp;
  Point3D position;
  Vector3D heading, velocity;
  double theta, phi;
  double rx, ry, rz;
  const unsigned short *tex;
  int texWidth;
  int texHeight;
  size_t counter;
  bool kill;
};
