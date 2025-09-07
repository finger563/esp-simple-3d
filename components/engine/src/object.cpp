#include "object.hpp"

// Constructor
Object::Object() {
  velocity = Vector3D(0, 0, 0);
  position = Point3D(0, 0, 0);
  kill = false;
  counter = 0;
  rx = 0;
  ry = 0;
  rz = 0;
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

// Alternate Constructor
Object::Object(Poly &poly, const unsigned short *texture, const int texWid, const int texHgt,
               const Vector3D &vel, Point3D pos, float _rx, float _ry, float _rz) {
  velocity = vel;
  position = pos;
  tex = texture;
  texWidth = texWid;
  texHeight = texHgt;

  // use Set Texture command instead...
  poly.SetTexture(texture, texWidth, texHeight);
  tex = texture;
  master.push_back(poly);
  temp.push_back(poly);
  rx = _rx;
  ry = _ry;
  rz = _rz;
}

// generate() method switch statements??

// Updates Temp list with any changes to the master list
bool Object::updateList() {
  temp.clear();
  std::copy(master.begin(), master.end(), std::back_inserter(temp));
  TranslateTemp(position);
  return true;
}

bool Object::updateList(const std::vector<Poly> &poly) {
  clearTemp();
  std::copy(poly.begin(), poly.end(), std::back_inserter(temp));
  TranslateTemp(position);
  return true;
}

void Object::Transform(Matrix &m) {
  for (auto &poly : master) {
    poly.Transform(m);
  }
}

void Object::Translate(Vector3D &v) {
  for (auto &poly : master) {
    poly.Translate(v.x, v.y, v.z);
  }
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

void Object::clearTemp() {
  // empties temp list
  temp.clear();
}

void Object::TransformTemp(const Matrix &m) {
  for (auto &poly : temp) {
    poly.Transform(m);
  }
}

void Object::TranslateTemp(const Vector3D &v) {
  for (auto &poly : temp) {
    poly.Translate(v);
  }
}

void Object::RotateTempToHeading() {
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
  TransformTemp(m);
}

void Object::add(const Poly &poly) {
  master.push_back(poly);
  updateList();
}

void Object::GenerateCube(float size) {
  const float x0 = -size, x1 = size;
  const float y0 = -size, y1 = size;
  const float z0 = -size, z1 = size;

  std::vector<Vertex> vertices;
  vertices.reserve(24);
  auto push_face = [&](float ax, float ay, float az, float bx, float by, float bz, float cx,
                       float cy, float cz, float dx, float dy, float dz) {
    Vertex v0(ax, ay, az, 1.0f, 0.0f, 1.0f);
    Vertex v1(bx, by, bz, 1.0f, 0.0f, 0.0f);
    Vertex v2(cx, cy, cz, 1.0f, 1.0f, 0.0f);
    Vertex v3(dx, dy, dz, 1.0f, 1.0f, 1.0f);
    vertices.push_back(v0);
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
  };
  // +Z face
  push_face(x0, y0, z1, x0, y1, z1, x1, y1, z1, x1, y0, z1);
  // -Z face
  push_face(x1, y0, z0, x1, y1, z0, x0, y1, z0, x0, y0, z0);
  // +X face
  push_face(x1, y0, z1, x1, y1, z1, x1, y1, z0, x1, y0, z0);
  // -X face
  push_face(x0, y0, z0, x0, y1, z0, x0, y1, z1, x0, y0, z1);
  // +Y face
  push_face(x0, y1, z1, x0, y1, z0, x1, y1, z0, x1, y1, z1);
  // -Y face
  push_face(x0, y0, z0, x0, y0, z1, x1, y0, z1, x1, y0, z0);

  std::vector<uint32_t> indices;
  indices.reserve(36);
  for (uint32_t f = 0; f < 6; ++f) {
    uint32_t b = f * 4;
    indices.push_back(b + 0);
    indices.push_back(b + 1);
    indices.push_back(b + 2);
    indices.push_back(b + 0);
    indices.push_back(b + 2);
    indices.push_back(b + 3);
  }

  RenderType rt = (tex && texWidth > 0 && texHeight > 0) ? TEXTURED : COLORED;
  float cr = 1.0f, cg = 1.0f, cb = 1.0f;
  AddMesh(vertices, indices, rt, tex, texWidth, texHeight, cr, cg, cb);

  rx = size;
  ry = size;
  rz = size;
  theta = 3.141592f;
  phi = 0.0f;
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
  updateList();
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
  std::vector<uint32_t> indices = {
      // +X face (x = x1)
      5,
      7,
      6,
      5,
      6,
      4,
      // +Y face (y = y1)
      3,
      7,
      6,
      3,
      6,
      2,
      // +Z face (z = z1)
      1,
      3,
      7,
      1,
      7,
      5,
      // -X face (x = x0)
      0,
      2,
      3,
      0,
      3,
      1,
      // -Y face (y = y0)
      0,
      1,
      5,
      0,
      5,
      4,
      // -Z face (z = z0)
      4,
      6,
      2,
      4,
      2,
      0,
  };
  AddMesh(vertices, indices, COLORED, nullptr, 0, 0, color.x, color.y, color.z);
}

void Object::GenerateShot(const Vector3D &pos, float theta_, float phi_) {
  master.push_back(Poly(Vertex(0, 0, 4, 1), Vertex(0, 2, -2, 1), Vertex(0, -2, -2, 1), Vertex(), 3,
                        Vector3D(1, 0, 0), COLORED));
  master.begin()->SetDoubleSided(true);
  master.begin()->SetVertexColors(
      rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX,
      rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX,
      rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX);

  rx = 0;
  ry = 0;
  rz = 0;
  theta = 0;
  phi = 0;

  updateList();
  theta = theta_;
  phi = phi_;
  position = pos;
  // SetRenderType(FLAT);
  RotateToHeading();
}

void Object::GeneratePlayer(const Vector3D &pos, float theta_, float phi_,
                            const unsigned short *texture, const int texWid, const int texHgt) {
  tex = texture;
  texWidth = texWid;
  texHeight = texHgt;
  GenerateCube();
  theta = theta_;
  phi = phi_;
  position = pos;
  SetRenderType(TEXTURED);
  RotateToHeading();
  updateList();
}

bool Object::UpdateTime(int time) { return true; }

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
  for (auto &poly : master) {
    poly.SetRenderType(rt);
  }
  for (auto &m : meshes) {
    m.rType = rt;
  }
  return true;
}

void Object::TransformToCamera(Matrix &m) {
  for (auto &poly : temp) {
    poly.TransformToCamera(m);
  }
}

void Object::TransformToPerspective(Matrix &m) {
  for (auto &poly : temp) {
    poly.TransformToPerspective(m);
  }
}

void Object::TransformToPixel(Matrix &m) {
  for (auto &poly : temp) {
    poly.TransformToPixel(m);
  }
}

// returns final render list
std::vector<Poly> Object::GetRenderList() const {
  std::vector<Poly> get;
  std::vector<Poly> local = temp;
  for (auto &poly : local) {
    if ((poly.visible || poly.doublesided) &&
        (poly.v[0].z > 0 || poly.v[1].z > 0 || poly.v[2].z > 0 || poly.v[3].z > 0)) {
      get.push_back(poly);
    }
  }

  return get;
}

std::vector<Poly> Object::GetTemp() const { return temp; }

void Object::AppendRenderPointers(std::vector<Poly *> &out) {
  for (auto &poly : temp) {
    if ((poly.visible || poly.doublesided) &&
        (poly.v[0].z > 0 || poly.v[1].z > 0 || poly.v[2].z > 0 || poly.v[3].z > 0)) {
      out.push_back(&poly);
    }
  }
}

void Object::AppendDrawItems(const Matrix &view, const Matrix &proj, const Matrix &viewport,
                             std::vector<Vertex> &outVertices, std::vector<uint32_t> &outIndices,
                             std::vector<DrawView> &outDraws) const {
  // Transform-and-append unique vertices for each mesh; rebase indices
  for (const auto &mesh : meshes) {
    const size_t baseVertex = outVertices.size();
    const size_t baseIndex = outIndices.size();
    outVertices.reserve(baseVertex + mesh.vertices.size());
    outIndices.reserve(baseIndex + mesh.indices.size());
    // Transform vertices: view -> projection -> homogeneous divide -> viewport
    for (const auto &vin : mesh.vertices) {
      Vertex v = vin;
      // apply object local->world translation
      v.Translate(Vector3D(position.x, position.y, position.z));
      v.TransformToCamera(view);
      v.TransformToPerspective(proj);
      v.HomogeneousDivide();
      v.Transform(viewport);
      outVertices.push_back(v);
    }
    // Copy local indices (no rebase here; we use baseVertex in the draw view)
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
  // Consider legacy polys
  for (const auto &poly : master) {
    int n = poly.numVertices;
    for (int i = 0; i < n; ++i) {
      const Vertex &v = poly.v[i];
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
  Point3D mn, mx;
  if (!GetLocalBounds(mn, mx))
    return false;
  outMin = mn + position;
  outMax = mx + position;
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

////////////////////////////////////////
/////////////////Projectile functons////
///////////////////////////////////////
void Object::projectileInit(const Vector3D &head, const Vector3D &pos) {
  // may have to create overload this to take a vector based
  // on where the mouse clicks
  heading = head;
  position = pos;
  GenerateCube(1);
  counter = 1;
}

bool Object::CollidesWith(const Object &b) {
  float distance = magnitude(b.GetPosition() - position);
  // if ( distance >= (radius + b.getradius()) )
  //	return false;
  if (!updateList())
    return false;
  TranslateTemp(position);
  std::vector<Poly> blist = b.GetTemp();
  for (auto &bpoly : blist) {
    Vector3D p0 = Vector3D(bpoly.v[0].x, bpoly.v[0].y, bpoly.v[0].z),
             p1 = Vector3D(bpoly.v[1].x, bpoly.v[1].y, bpoly.v[1].z),
             p2 = Vector3D(bpoly.v[2].x, bpoly.v[2].y, bpoly.v[2].z);
    for (auto &it : temp) {
      Vector3D A = Vector3D(it.v[0].x, it.v[0].y, it.v[0].z),
               B = Vector3D(it.v[1].x, it.v[1].y, it.v[1].z),
               C = Vector3D(it.v[2].x, it.v[2].y, it.v[2].z);
      Vector3D p, n1, n2, n3;
      float t = (-it.normal * (p0 - A)) / (it.normal * (p1 - p0));
      if (t > 0 && t < 1) {
        p = p0 + (p1 - p0) * t;
        n1 = normalize(Cross(A - B, p - B));
        n2 = normalize(Cross(B - C, p - C));
        n3 = normalize(Cross(C - A, p - A));
        if (n1 * n2 > 0 && n2 * n3 > 0)
          return true;
      } else {
        t = (-it.normal * (p0 - A)) / (it.normal * (p2 - p0));
        if (t > 0 && t < 1) {
          p = p0 + (p2 - p0) * t;
          n1 = normalize(Cross(A - B, p - B));
          n2 = normalize(Cross(B - C, p - C));
          n3 = normalize(Cross(C - A, p - A));
          if (n1 * n2 > 0 && n2 * n3 > 0)
            return true;
        }
      }
    }
  }
  return false;
}
