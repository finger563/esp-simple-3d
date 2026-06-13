#include "object.hpp"

namespace {

inline void TransformPointFast(const Matrix &m, float x, float y, float z, float w, float &outX,
                               float &outY, float &outZ, float &outW) {
  outX = x * m[0][0] + y * m[1][0] + z * m[2][0] + w * m[3][0];
  outY = x * m[0][1] + y * m[1][1] + z * m[2][1] + w * m[3][1];
  outZ = x * m[0][2] + y * m[1][2] + z * m[2][2] + w * m[3][2];
  outW = x * m[0][3] + y * m[1][3] + z * m[2][3] + w * m[3][3];
}

} // namespace

// Constructor
Object::Object() {
  velocity = Vector3D(0, 0, 0);
  position = Point3D(0, 0, 0);
  rx = 0;
  ry = 0;
  rz = 0;
  meshTransform.SetIdentity();
}

// Alternate texture, veloctity, heading, position
Object::Object(const unsigned short *texture, const int texWid, const int texHgt,
               const Vector3D &vel, Point3D pos, float _rx, float _ry, float _rz) {
  velocity = vel;
  position = pos;
  tex = texture;
  texWidth = texWid;
  texHeight = texHgt;
  rx = _rx;
  ry = _ry;
  rz = _rz;
}

void Object::Transform(Matrix &m) {
  // Accumulate local mesh transform (rotation/scale)
  meshTransform = meshTransform * m;
}

void Object::Translate(Vector3D &v) {
  // For meshes, keep translation in position to be applied at draw time
  position = position + v;
}

void Object::RotateToHeading() {
  float r = cosf(phi);
  float x = r * sinf(theta), y = sinf(phi), z = r * cosf(theta);
  Vector3D forward = normalize(Vector3D(x, y, z));
  Vector3D up = normalize(Vector3D(0, 1, 0));
  Vector3D right = normalize(Cross(up, forward));
  up = normalize(Cross(forward, right));
  Matrix m = Matrix();
  m[0][0] = right.x;
  m[0][1] = right.y;
  m[0][2] = right.z;
  m[1][0] = up.x;
  m[1][1] = up.y;
  m[1][2] = up.z;
  m[2][0] = forward.x;
  m[2][1] = forward.y;
  m[2][2] = forward.z;
  Transform(m);
}

void Object::RotateToHeading(const Vector3D &changeUp) {
  float r = cosf(phi);
  float x = r * sinf(theta), y = sinf(phi), z = r * cosf(theta);
  Vector3D forward = normalize(Vector3D(x, y, z));
  Vector3D up = normalize(changeUp);
  Vector3D right = normalize(Cross(up, forward));
  up = normalize(Cross(forward, right));
  Matrix m = Matrix();
  m[0][0] = right.x;
  m[0][1] = right.y;
  m[0][2] = right.z;
  m[1][0] = up.x;
  m[1][1] = up.y;
  m[1][2] = up.z;
  m[2][0] = forward.x;
  m[2][1] = forward.y;
  m[2][2] = forward.z;
  Transform(m);
}

void Object::GenerateCube(float size) {
  Matrix I;
  I.SetIdentity();
  GenerateRectangularPrism(Vector3D(size, size, size), I, Vector3D(1, 1, 1));
}

void Object::GenerateTetra(float size) {
  std::vector<Vertex> vertices;
  vertices.reserve(4);
  float s2 = size / sqrtf(2.0f);
  vertices.emplace_back(size, 0, -s2, 1.0f);
  vertices.emplace_back(-size, 0, -s2, 1.0f);
  vertices.emplace_back(0, size, s2, 1.0f);
  vertices.emplace_back(0, -size, s2, 1.0f);
  std::vector<uint32_t> indices{
      0, 1, 2, // p1,p2,p3
      1, 2, 3, // p2,p3,p4
      2, 3, 0, // p3,p4,p1
      0, 1, 3  // p1,p2,p4
  };
  AddMesh(vertices, indices, COLORED, nullptr, 0, 0, 1.0f, 1.0f, 1.0f);
}

void Object::GenerateFloor(float length, float depth) {
  // Build a textured quad at y=depth, spanning [-length,length] in X and Z
  std::vector<Vertex> vertices;
  vertices.reserve(4);
  vertices.emplace_back(-length, depth, -length, 1.0f, 0.0f, 1.0f);
  vertices.emplace_back(-length, depth, +length, 1.0f, 0.0f, 0.0f);
  vertices.emplace_back(+length, depth, +length, 1.0f, 1.0f, 0.0f);
  vertices.emplace_back(+length, depth, -length, 1.0f, 1.0f, 1.0f);
  std::vector<uint32_t> indices{0, 1, 2, 0, 2, 3};
  AddMesh(vertices, indices, (tex && texWidth > 0 && texHeight > 0) ? TEXTURED : COLORED, tex,
          texWidth, texHeight, 1.0f, 1.0f, 1.0f);
  rx = length * 0.5f;
  ry = 0.0f;
  rz = length * 0.5f;
}

// Incomplete...
void Object::GenerateCeiling(float length, float depth) {
  // Build a textured quad at y=depth
  std::vector<Vertex> vertices;
  vertices.reserve(4);
  vertices.emplace_back(-length, depth, -length, 1.0f, 0.0f, 1.0f);
  vertices.emplace_back(-length, depth, +length, 1.0f, 0.0f, 0.0f);
  vertices.emplace_back(+length, depth, +length, 1.0f, 1.0f, 0.0f);
  vertices.emplace_back(+length, depth, -length, 1.0f, 1.0f, 1.0f);
  std::vector<uint32_t> indices{0, 1, 2, 0, 2, 3};
  AddMesh(vertices, indices, (tex && texWidth > 0 && texHeight > 0) ? TEXTURED : COLORED, tex,
          texWidth, texHeight, 1.0f, 1.0f, 1.0f);
  rx = length * 0.5f;
  ry = 0.0f;
  rz = length * 0.5f;
}

// Generates wall
// Wall types: 0 = front: faces the player init
///////////// 1 = left: the left of player init
///////////// 2 = right: to the right of player init
///////////// 3 = back: behind player init
void Object::GenerateWall(size_t type, float length, float depth) {
  // Build a vertical textured quad centered at y=depth
  std::vector<Vertex> vertices;
  vertices.reserve(4);
  if (type == 0) { // front (+Z)
    vertices.emplace_back(-length, depth - length, +length, 1.0f, 0.0f, 1.0f);
    vertices.emplace_back(-length, depth + length, +length, 1.0f, 0.0f, 0.0f);
    vertices.emplace_back(+length, depth + length, +length, 1.0f, 1.0f, 0.0f);
    vertices.emplace_back(+length, depth - length, +length, 1.0f, 1.0f, 1.0f);
  } else if (type == 3) { // back (-Z)
    vertices.emplace_back(+length, depth - length, -length, 1.0f, 0.0f, 1.0f);
    vertices.emplace_back(+length, depth + length, -length, 1.0f, 0.0f, 0.0f);
    vertices.emplace_back(-length, depth + length, -length, 1.0f, 1.0f, 0.0f);
    vertices.emplace_back(-length, depth - length, -length, 1.0f, 1.0f, 1.0f);
  } else if (type == 1) { // left (-X)
    vertices.emplace_back(-length, depth - length, -length, 1.0f, 0.0f, 1.0f);
    vertices.emplace_back(-length, depth + length, -length, 1.0f, 0.0f, 0.0f);
    vertices.emplace_back(-length, depth + length, +length, 1.0f, 1.0f, 0.0f);
    vertices.emplace_back(-length, depth - length, +length, 1.0f, 1.0f, 1.0f);
  } else if (type == 2) { // right (+X)
    vertices.emplace_back(+length, depth - length, +length, 1.0f, 0.0f, 1.0f);
    vertices.emplace_back(+length, depth + length, +length, 1.0f, 0.0f, 0.0f);
    vertices.emplace_back(+length, depth + length, -length, 1.0f, 1.0f, 0.0f);
    vertices.emplace_back(+length, depth - length, -length, 1.0f, 1.0f, 1.0f);
  }
  std::vector<uint32_t> indices{0, 1, 2, 0, 2, 3};
  AddMesh(vertices, indices, (tex && texWidth > 0 && texHeight > 0) ? TEXTURED : COLORED, tex,
          texWidth, texHeight, 1.0f, 1.0f, 1.0f);
  rx = length * 0.5f;
  ry = length * 0.5f;
  rz = length * 0.5f;
}

void Object::GenerateAxes(float length, float thickness) {
  // Use rectangular prisms aligned with axes, centered at origin
  GenerateRectangularPrism(Vector3D(length / 2, thickness, thickness),
                           Matrix::Translation(Point3D(length / 2, 0, 0)), Vector3D(1, 0, 0));
  GenerateRectangularPrism(Vector3D(thickness, length / 2, thickness),
                           Matrix::Translation(Point3D(0, length / 2, 0)), Vector3D(0, 1, 0));
  GenerateRectangularPrism(Vector3D(thickness, thickness, length / 2),
                           Matrix::Translation(Point3D(0, 0, length / 2)), Vector3D(0, 0, 1));
  // negative axis
  GenerateRectangularPrism(Vector3D(length / 2, thickness, thickness),
                           Matrix::Translation(Point3D(-length / 2, 0, 0)), Vector3D(0.35, 0, 0));
  GenerateRectangularPrism(Vector3D(thickness, length / 2, thickness),
                           Matrix::Translation(Point3D(0, -length / 2, 0)), Vector3D(0, 0.35, 0));
  GenerateRectangularPrism(Vector3D(thickness, thickness, length / 2),
                           Matrix::Translation(Point3D(0, 0, -length / 2)), Vector3D(0, 0, 0.35));

  // for all the meshes, set the render type to wireframe
  for (auto &m : meshes) {
    m.rType = WIREFRAME;
  }
}

void Object::GenerateRectangularPrism(const Vector3D &halfSize, const Matrix &transform,
                                      const Vector3D &color) {
  const float x0 = -halfSize.x, x1 = halfSize.x;
  const float y0 = -halfSize.y, y1 = halfSize.y;
  const float z0 = -halfSize.z, z1 = halfSize.z;

  // 8 unique corners
  std::vector<Vertex> vertices;
  vertices.reserve(8);
  auto make_v = [&](float x, float y, float z) {
    Vertex v(x, y, z, 1.0f);
    v.SetColor(color.x, color.y, color.z);
    v.Transform(transform);
    return v;
  };
  // Index mapping
  // 0: (x0,y0,z0)  1: (x0,y0,z1)  2: (x0,y1,z0)  3: (x0,y1,z1)
  // 4: (x1,y0,z0)  5: (x1,y0,z1)  6: (x1,y1,z0)  7: (x1,y1,z1)
  vertices.emplace_back(make_v(x0, y0, z0)); // 0
  vertices.emplace_back(make_v(x0, y0, z1)); // 1
  vertices.emplace_back(make_v(x0, y1, z0)); // 2
  vertices.emplace_back(make_v(x0, y1, z1)); // 3
  vertices.emplace_back(make_v(x1, y0, z0)); // 4
  vertices.emplace_back(make_v(x1, y0, z1)); // 5
  vertices.emplace_back(make_v(x1, y1, z0)); // 6
  vertices.emplace_back(make_v(x1, y1, z1)); // 7

  // 12 triangles (two per face), shared vertices

  // clang-format off
  std::vector<uint32_t> indices = {
      // +X face (x = x1)
      5, 7, 6, 5, 6, 4,
      // +Y face (y = y1)
      3, 7, 6, 3, 6, 2,
      // +Z face (z = z1)
      1, 3, 7, 1, 7, 5,
      // -X face (x = x0)
      0, 2, 3, 0, 3, 1,
      // -Y face (y = y0)
      0, 1, 5, 0, 5, 4,
      // -Z face (z = z0)
      4, 6, 2, 4, 2, 0,
  };
  // clang-format on
  AddMesh(vertices, indices, COLORED, nullptr, 0, 0, color.x, color.y, color.z);
}

void Object::GenerateShot(const Matrix &transform) {
  std::vector<Vertex> vertices;
  vertices.reserve(3);
  vertices.emplace_back(0, 0, 4.0f);
  vertices.emplace_back(0, 2, -2.0f);
  vertices.emplace_back(0, -2, -2);
  std::vector<uint32_t> indices{0, 1, 2};
  AddMesh(vertices, indices, COLORED, tex, texWidth, texHeight, 1.0f, 1.0f, 1.0f);

  meshTransform = transform;

  rx = 0;
  ry = 0;
  rz = 0;

  RotateToHeading();
}

void Object::GenerateShot(const Vector3D &pos, float theta_, float phi_) {
  theta = theta_;
  phi = phi_;
  position = pos;
  GenerateShot(Matrix::Identity());
}

void Object::GeneratePlayer(const Matrix &transform, const unsigned short *texture,
                            const int texWid, const int texHgt) {
  tex = texture;
  texWidth = texWid;
  texHeight = texHgt;
  GenerateCube();
  meshTransform = transform;
  SetRenderType(TEXTURED);
  RotateToHeading();
}

void Object::GeneratePlayer(const Vector3D &pos, float theta_, float phi_,
                            const unsigned short *texture, const int texWid, const int texHgt) {
  theta = theta_;
  phi = phi_;
  position = pos;
  GeneratePlayer(Matrix::Identity(), texture, texWid, texHgt);
}

bool Object::Update(int time) { return true; }

bool Object::SetVelocity(const Vector3D &vector) {
  velocity = vector;
  return true;
}

bool Object::SetPosition(const Point3D &pos) {
  position = pos;
  return true;
}

Point3D Object::GetPosition(void) const { return position; }

bool Object::SetBoudingEllipsoid(float x, float y, float z) {
  rx = x;
  ry = y;
  rz = z;
  return true;
}

float Object::GetRadiusX(void) const { return rx; }

float Object::GetRadiusY(void) const { return ry; }

float Object::GetRadiusZ(void) const { return rz; }

bool Object::SetRenderType(RenderType rt) {
  for (auto &m : meshes) {
    m.rType = rt;
  }
  return true;
}

void Object::AppendDrawItems(const Matrix &view, const Matrix &proj, const Matrix &viewport,
                             std::vector<Vertex> &outVertices, std::vector<uint32_t> &outIndices,
                             std::vector<DrawView> &outDraws) const {
  const Matrix modelView =
      (meshTransform * Matrix::Translation(position.x, position.y, position.z)) * view;
  const float viewportScaleX = viewport[0][0];
  const float viewportScaleY = viewport[1][1];
  const float viewportOffsetX = viewport[3][0];
  const float viewportOffsetY = viewport[3][1];

  for (const auto &mesh : meshes) {
    const size_t baseVertex = outVertices.size();
    const size_t baseIndex = outIndices.size();
    outVertices.reserve(baseVertex + mesh.vertices.size());
    outIndices.reserve(baseIndex + mesh.indices.size());

    for (const auto &vin : mesh.vertices) {
      Vertex v;
      float camX, camY, camZ, camW;
      TransformPointFast(modelView, vin.x, vin.y, vin.z, vin.w, camX, camY, camZ, camW);

      float clipX, clipY, clipZ, clipW;
      TransformPointFast(proj, camX, camY, camZ, camW, clipX, clipY, clipZ, clipW);

      const float invW = 1.0f / clipW;
      v.x = clipX * invW * viewportScaleX + viewportOffsetX;
      v.y = clipY * invW * viewportScaleY + viewportOffsetY;
      v.z = clipZ * invW;
      v.w = 1.0f;
      v.ex = camX * invW;
      v.ey = camY * invW;
      v.ez = camZ * invW;
      v.u = vin.u * invW;
      v.v = vin.v * invW;
      v.r = vin.r;
      v.g = vin.g;
      v.b = vin.b;
      v.nx = vin.nx;
      v.ny = vin.ny;
      v.nz = vin.nz;
      v.hw = invW;
      outVertices.push_back(v);
    }

    outIndices.insert(outIndices.end(), mesh.indices.begin(), mesh.indices.end());
    DrawView d;
    d.baseVertex = baseVertex;
    d.baseIndex = baseIndex;
    d.indexCount = mesh.indices.size();
    d.rType = mesh.rType;
    d.texture = mesh.texture;
    d.texwidth = mesh.texwidth;
    d.texheight = mesh.texheight;
    d.r = mesh.r;
    d.g = mesh.g;
    d.b = mesh.b;
    outDraws.push_back(d);
  }
}

bool Object::GetLocalBounds(Point3D &outMin, Point3D &outMax) const {
  bool initialized = false;
  Point3D mn, mx;
  // Consider indexed meshes
  for (const auto &m : meshes) {
    for (const auto &v : m.vertices) {
      if (!initialized) {
        mn = mx = Point3D(v.x, v.y, v.z);
        initialized = true;
      } else {
        if (v.x < mn.x)
          mn.x = v.x;
        if (v.y < mn.y)
          mn.y = v.y;
        if (v.z < mn.z)
          mn.z = v.z;
        if (v.x > mx.x)
          mx.x = v.x;
        if (v.y > mx.y)
          mx.y = v.y;
        if (v.z > mx.z)
          mx.z = v.z;
      }
    }
  }
  if (!initialized)
    return false;
  outMin = mn;
  outMax = mx;
  return true;
}

bool Object::GetWorldBounds(Point3D &outMin, Point3D &outMax) const {
  bool initialized = false;
  Point3D mn, mx;

  // Indexed meshes: transform by meshTransform then add translation
  for (const auto &mesh : meshes) {
    for (const auto &vin : mesh.vertices) {
      Vertex v = vin;
      v.Transform(meshTransform);
      Point3D wp(v.x + position.x, v.y + position.y, v.z + position.z);
      if (!initialized) {
        mn = mx = wp;
        initialized = true;
      } else {
        if (wp.x < mn.x)
          mn.x = wp.x;
        if (wp.y < mn.y)
          mn.y = wp.y;
        if (wp.z < mn.z)
          mn.z = wp.z;
        if (wp.x > mx.x)
          mx.x = wp.x;
        if (wp.y > mx.y)
          mx.y = wp.y;
        if (wp.z > mx.z)
          mx.z = wp.z;
      }
    }
  }

  if (!initialized)
    return false;
  outMin = mn;
  outMax = mx;
  return true;
}

void Object::AddMesh(const std::vector<Vertex> &vertices, const std::vector<uint32_t> &indices,
                     RenderType rt, const unsigned short *texPtr, int texW, int texH, float cr,
                     float cg, float cb) {
  Mesh m;
  m.vertices = vertices;
  m.indices = indices;
  m.rType = rt;
  m.texture = texPtr;
  m.texwidth = texW;
  m.texheight = texH;
  m.r = cr;
  m.g = cg;
  m.b = cb;
  meshes.push_back(std::move(m));
}

void Object::projectileInit(const Vector3D &head, const Vector3D &pos) {
  // may have to create overload this to take a vector based
  // on where the mouse clicks
  heading = head;
  position = pos;
  GenerateCube(1);
}

bool Object::CollidesWith(const Object &b) const {
  // Broad-phase: AABB overlap in world space
  Point3D aMin, aMax, bMin, bMax;
  if (!GetWorldBounds(aMin, aMax) || !b.GetWorldBounds(bMin, bMax))
    return false;
  bool overlap = !(aMax.x < bMin.x || aMin.x > bMax.x || aMax.y < bMin.y || aMin.y > bMax.y ||
                   aMax.z < bMin.z || aMin.z > bMax.z);
  if (!overlap)
    return false;

  // TODO: narrow phase collision between the actual meshes?
  return false;
}
