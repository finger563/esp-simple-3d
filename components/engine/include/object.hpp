#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "polygon.hpp"

class Object {
public:
  struct DrawView {
    size_t baseVertex{0};
    size_t baseIndex{0};
    size_t indexCount{0};
    RenderType rType{TEXTURED};
    const unsigned short *texture{nullptr};
    int texwidth{0};
    int texheight{0};
    float r{1.0f}, g{1.0f}, b{1.0f};
  };
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

  // Utility: generate axis lines (X=red, Y=green, Z=blue) centered at origin
  void GenerateAxes(float length = 10.0f, float thickness = 0.02f);

  // Generate a rectangular prism centered at origin with half-size per axis, then
  // apply a transform (rotation/translation) before adding to master. The prism
  // is COLORED using the provided RGB in [0,1]. Does not clear existing geometry.
  void GenerateRectangularPrism(const Vector3D &halfSize, const Matrix &transform,
                                const Vector3D &color);

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
  void Transform(Matrix &m);
  void Translate(Vector3D &v);

  // Temp list operations
  void clearTemp();
  void TransformTemp(const Matrix &m);
  void TranslateTemp(const Vector3D &v);
  void RotateTempToHeading();

  // Pipeline functions
  void TransformToCamera(Matrix &m);
  void TransformToPerspective(Matrix &m);
  void TransformToPixel(Matrix &m);
  std::vector<Poly> GetRenderList() const;
  std::vector<Poly> GetTemp() const;
  // Append pointers to renderable polys in temp to avoid copies
  void AppendRenderPointers(std::vector<Poly *> &out);
  // Expose temp size for pre-reserving render pointer capacity
  size_t TempSize() const { return temp.size(); }

  // Build per-frame transformed vertices/indices and draw views for indexed meshes
  void AppendDrawItems(const Matrix &view, const Matrix &proj, const Matrix &viewport,
                       std::vector<Vertex> &outVertices, std::vector<uint32_t> &outIndices,
                       std::vector<DrawView> &outDraws) const;

  // Geometry bounds helpers
  // Returns axis-aligned bounds in object local space. Returns false if empty.
  bool GetLocalBounds(Point3D &outMin, Point3D &outMax) const;
  // Returns axis-aligned bounds in world space (local bounds offset by position)
  bool GetWorldBounds(Point3D &outMin, Point3D &outMax) const;

  // Add an indexed mesh to this object
  void AddMesh(const std::vector<Vertex> &vertices, const std::vector<uint32_t> &indices,
               RenderType rt, const unsigned short *texPtr, int texW, int texH, float cr = 1.0f,
               float cg = 1.0f, float cb = 1.0f);

  void projectileInit(const Vector3D &head, const Vector3D &pos = Vector3D(0, 0, 0));

  bool CollidesWith(const Object &b);

private:
  struct Mesh {
    std::vector<Vertex> vertices;  // unique vertices (object local space)
    std::vector<uint32_t> indices; // triangle indices (3 per face)
    RenderType rType{TEXTURED};
    const unsigned short *texture{nullptr};
    int texwidth{0};
    int texheight{0};
    float r{1.0f}, g{1.0f}, b{1.0f}; // for COLORED
  };

  std::vector<Poly> master;
  std::vector<Poly> temp;
  std::vector<Mesh> meshes; // indexed meshes (preferred)
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
