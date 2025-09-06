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
  // stores in objects master list
  master.push_back(Poly(Vertex(size, size, size, 1, 0, 0), Vertex(-size, size, size, 1, 1, 0),
                        Vertex(size, -size, size, 1, 0, 1), Vertex(), 3, Vector3D(0, 0, 1),
                        TEXTURED));

  master.push_back(Poly(Vertex(size, -size, size, 1, 0, 1), Vertex(-size, size, size, 1, 1, 0),
                        Vertex(-size, -size, size, 1, 1, 1), Vertex(), 3, Vector3D(0, 0, 1),
                        TEXTURED));

  master.push_back(Poly(Vertex(size, size, -size, 1, 1, 0), Vertex(size, -size, -size, 1, 1, 1),
                        Vertex(-size, size, -size, 1, 0, 0), Vertex(), 3, Vector3D(0, 0, -1),
                        TEXTURED));

  master.push_back(Poly(Vertex(size, -size, -size, 1, 1, 1), Vertex(-size, -size, -size, 1, 0, 1),
                        Vertex(-size, size, -size, 1, 0, 0), Vertex(), 3, Vector3D(0, 0, -1),
                        TEXTURED));

  master.push_back(Poly(Vertex(size, size, -size, 1, 0, 0), Vertex(size, size, size, 1, 1, 0),
                        Vertex(size, -size, size, 1, 1, 1), Vertex(), 3, Vector3D(1, 0, 0),
                        TEXTURED));

  master.push_back(Poly(Vertex(size, size, -size, 1, 0, 0), Vertex(size, -size, size, 1, 1, 1),
                        Vertex(size, -size, -size, 1, 0, 1), Vertex(), 3, Vector3D(1, 0, 0),
                        TEXTURED));

  master.push_back(Poly(Vertex(-size, size, -size, 1, 1, 0), Vertex(-size, -size, -size, 1, 1, 1),
                        Vertex(-size, size, size, 1, 0, 0), Vertex(), 3, Vector3D(-1, 0, 0),
                        TEXTURED));

  master.push_back(Poly(Vertex(-size, size, size, 1, 0, 0), Vertex(-size, -size, -size, 1, 1, 1),
                        Vertex(-size, -size, size, 1, 0, 1), Vertex(), 3, Vector3D(-1, 0, 0),
                        TEXTURED));

  master.push_back(Poly(Vertex(size, size, size, 1, 1, 0), Vertex(size, size, -size, 1, 1, 1),
                        Vertex(-size, size, size, 1, 0, 0), Vertex(), 3, Vector3D(0, 1, 0),
                        TEXTURED));

  master.push_back(Poly(Vertex(-size, size, -size, 1, 0, 1), Vertex(-size, size, size, 1, 0, 0),
                        Vertex(size, size, -size, 1, 1, 1), Vertex(), 3, Vector3D(0, 1, 0),
                        TEXTURED));

  master.push_back(Poly(Vertex(size, -size, size, 1, 1, 1), Vertex(-size, -size, -size, 1, 0, 0),
                        Vertex(size, -size, -size, 1, 1, 0), Vertex(), 3, Vector3D(0, -1, 0),
                        TEXTURED));

  master.push_back(Poly(Vertex(size, -size, size, 1, 1, 1), Vertex(-size, -size, size, 1, 0, 1),
                        Vertex(-size, -size, -size, 1, 0, 0), Vertex(), 3, Vector3D(0, -1, 0),
                        TEXTURED));

  for (auto &it : master) {
    it.SetTexture(tex, texWidth, texHeight);
    it.SetColor(rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX);
    it.SetVertexColors(0.9, 0, 0, 0, 0.9, 0, 0, 0, 0.9);
  }

  rx = size;
  ry = size;
  rz = size;
  theta = 3.141592;
  phi = 0;

  updateList();
}

void Object::GenerateTetra(float size) {
  Vertex p1 = Vertex(size, 0, -size / sqrt(2.0)), p2 = Vertex(-size, 0, -size / sqrt(2.0)),
         p3 = Vertex(0, size, size / sqrt(2.0)), p4 = Vertex(0, -size, size / sqrt(2.0));

  Poly tri1 = Poly(p1, p2,
                   p3), //, Vector3D(0,0,1),Point2D(0,0),Point2D(0,texWidth),Point2D(texWidth,0)),

      tri2 = Poly(
          p2, p3,
          p4), //,
               // Vector3D(0,0,1),Point2D(0,texWidth),Point2D(texWidth,0),Point2D(texWidth,texWidth)),

      tri3 = Poly(
          p3, p4,
          p1), //, Vector3D(0,0,-1),Point2D(texWidth,0),Point2D(texWidth,texWidth),Point2D(0,0)),

      tri4 = Poly(
          p1, p2,
          p4); //, Vector3D(0,0,-1),Point2D(texWidth,texWidth),Point2D(0,0),Point2D(0,texWidth));

  // stores in objects master list
  master.push_back(tri1);
  master.push_back(tri2);
  master.push_back(tri3);
  master.push_back(tri4);

  for (auto &poly : master) {
    poly.SetTexture(tex, texWidth, texHeight);
  }

  updateList();
}

void Object::GenerateFloor(float length, float depth) {

  master.push_back(Poly(Vertex(-length, 0, -length, 1, 0, 1), Vertex(-length, 0, length, 1, 0, 0),
                        Vertex(length, 0, length, 1, 1, 0), Vertex(), 3, Vector3D(0, 1, 0),
                        TEXTURED));
  master.push_back(Poly(Vertex(-length, 0, -length, 1, 0, 1), Vertex(length, 0, length, 1, 1, 0),
                        Vertex(length, 0, -length, 1, 1, 1), Vertex(), 3, Vector3D(0, 1, 0),
                        TEXTURED));

  for (auto &poly : master) {
    poly.SetTexture(tex, texWidth, texHeight);
  }

  rx = length / 2.0;
  ry = 0;
  rz = length / 2.0;
  theta = 0;
  phi = 3.141592 / 2.0;
  position = Point3D(0, depth, 0);
  updateList();
}

// Incomplete...
void Object::GenerateCeiling(float length, float depth) {
  position = Point3D(0, depth, 0);

  theta = 0;
  phi = 0;

  master.push_back(Poly(Vertex(-length, 0, -length, 1, 0, 1), Vertex(-length, 0, length, 1, 0, 0),
                        Vertex(length, 0, length, 1, 1, 0), Vertex(), 3, Vector3D(0, 1, 0),
                        TEXTURED));
  master.push_back(Poly(Vertex(-length, 0, -length, 1, 0, 1), Vertex(length, 0, length, 1, 1, 0),
                        Vertex(length, 0, -length, 1, 1, 1), Vertex(), 3, Vector3D(0, 1, 0),
                        TEXTURED));

  for (auto &poly : master) {
    poly.SetTexture(tex, texWidth, texHeight);
  }

  rx = length / 2.0;
  ry = 0;
  rz = length / 2.0;

  RotateToHeading(Vector3D(0, -1, 0));
  updateList();
}

// Generates wall
// Wall types: 0 = front: faces the player init
///////////// 1 = left: the left of player init
///////////// 2 = right: to the right of player init
///////////// 3 = back: behind player init
void Object::GenerateWall(size_t type, float length, float depth) {

  switch (type) {
  case 0: // front
    theta = 0;
    phi = 3.14 / 2;
    position = Point3D(0, depth, length);
    break;
  case 1: // left
    theta = -3.14 / 2;
    phi = 3.14 / 2;
    position = Point3D(-length, depth, 0);
    break;
  case 2: // right
    theta = 3.14 / 2;
    phi = 3.14 / 2;
    position = Point3D(length, depth, 0);
    break;
  case 3: // behind
    theta = 3.14;
    phi = 3.14 / 2;
    position = Point3D(0, depth, -length);
    break;
  }

  master.push_back(Poly(Vertex(-length, 0, -length, 1, 0, 1), Vertex(-length, 0, length, 1, 0, 0),
                        Vertex(length, 0, length, 1, 1, 0), Vertex(), 3, Vector3D(0, 1, 0),
                        TEXTURED));
  master.push_back(Poly(Vertex(-length, 0, -length, 1, 0, 1), Vertex(length, 0, length, 1, 1, 0),
                        Vertex(length, 0, -length, 1, 1, 1), Vertex(), 3, Vector3D(0, 1, 0),
                        TEXTURED));

  for (auto &poly : master) {
    poly.SetTexture(tex, texWidth, texHeight);
  }

  rx = length / 2.0;
  ry = length / 2.0;
  rz = length / 2.0;

  RotateToHeading();
  updateList();
}

void Object::GenerateAxes(float length, float thickness) {
  Matrix I;
  I.SetIdentity();
  auto make_transform = [](float tx, float ty, float tz, float rx, float ry, float rz) {
    Matrix m;
    m.SetIdentity();
    Matrix rxm;
    rxm.SetIdentity();
    rxm[1][1] = cosf(rx);
    rxm[1][2] = -sinf(rx);
    rxm[2][1] = sinf(rx);
    rxm[2][2] = cosf(rx);
    Matrix rym;
    rym.SetIdentity();
    rym[0][0] = cosf(ry);
    rym[0][2] = sinf(ry);
    rym[2][0] = -sinf(ry);
    rym[2][2] = cosf(ry);
    Matrix rzm;
    rzm.SetIdentity();
    rzm[0][0] = cosf(rz);
    rzm[0][1] = -sinf(rz);
    rzm[1][0] = sinf(rz);
    rzm[1][1] = cosf(rz);
    Matrix r = rxm * (rym * rzm);
    r[3][0] = tx;
    r[3][1] = ty;
    r[3][2] = tz;
    return r;
  };

  // Use rectangular prisms aligned with axes, centered at origin
  GenerateRectangularPrism(Vector3D(length / 2, thickness, thickness),
                           Matrix::Translation(Point3D(length / 2, 0, 0)), Vector3D(1, 0, 0));
  GenerateRectangularPrism(Vector3D(thickness, length / 2, thickness),
                           Matrix::Translation(Point3D(0, length / 2, 0)), Vector3D(0, 1, 0));
  GenerateRectangularPrism(Vector3D(thickness, thickness, length / 2),
                           Matrix::Translation(Point3D(0, 0, length / 2)), Vector3D(0, 0, 1));
  // negative axis
  GenerateRectangularPrism(Vector3D(length / 2, thickness, thickness),
                           Matrix::Translation(Point3D(-length / 2, 0, 0)), Vector3D(0.25, 0, 0));
  GenerateRectangularPrism(Vector3D(thickness, length / 2, thickness),
                           Matrix::Translation(Point3D(0, -length / 2, 0)), Vector3D(0, 0.25, 0));
  GenerateRectangularPrism(Vector3D(thickness, thickness, length / 2),
                           Matrix::Translation(Point3D(0, 0, -length / 2)), Vector3D(0, 0, 0.25));
  updateList();
}

void Object::GenerateRectangularPrism(const Vector3D &halfSize, const Matrix &transform,
                                      const Vector3D &color) {
  const float x0 = -halfSize.x, x1 = halfSize.x;
  const float y0 = -halfSize.y, y1 = halfSize.y;
  const float z0 = -halfSize.z, z1 = halfSize.z;

  auto add_face = [&](Vertex a, Vertex b, Vertex c, const Vector3D &n) {
    // Apply transform to each vertex (position only)
    a.Transform(transform);
    b.Transform(transform);
    c.Transform(transform);
    a.SetColor(color.x, color.y, color.z);
    b.SetColor(color.x, color.y, color.z);
    c.SetColor(color.x, color.y, color.z);
    Poly p(a, b, c, Vertex(), 3, n, COLORED);
    p.SetDoubleSided(true);
    master.emplace_back(p);
  };

  // +X face
  add_face(Vertex(x1, y0, z0), Vertex(x1, y0, z1), Vertex(x1, y1, z0), Vector3D(1, 0, 0));
  add_face(Vertex(x1, y1, z0), Vertex(x1, y0, z1), Vertex(x1, y1, z1), Vector3D(1, 0, 0));
  // -X face
  add_face(Vertex(x0, y0, z0), Vertex(x0, y1, z0), Vertex(x0, y0, z1), Vector3D(-1, 0, 0));
  add_face(Vertex(x0, y1, z0), Vertex(x0, y1, z1), Vertex(x0, y0, z1), Vector3D(-1, 0, 0));
  // +Y face
  add_face(Vertex(x0, y1, z0), Vertex(x1, y1, z0), Vertex(x0, y1, z1), Vector3D(0, 1, 0));
  add_face(Vertex(x1, y1, z0), Vertex(x1, y1, z1), Vertex(x0, y1, z1), Vector3D(0, 1, 0));
  // -Y face
  add_face(Vertex(x0, y0, z0), Vertex(x0, y0, z1), Vertex(x1, y0, z0), Vector3D(0, -1, 0));
  add_face(Vertex(x1, y0, z0), Vertex(x0, y0, z1), Vertex(x1, y0, z1), Vector3D(0, -1, 0));
  // +Z face
  add_face(Vertex(x0, y0, z1), Vertex(x0, y1, z1), Vertex(x1, y0, z1), Vector3D(0, 0, 1));
  add_face(Vertex(x1, y0, z1), Vertex(x0, y1, z1), Vertex(x1, y1, z1), Vector3D(0, 0, 1));
  // -Z face
  add_face(Vertex(x0, y0, z0), Vertex(x1, y0, z0), Vertex(x0, y1, z0), Vector3D(0, 0, -1));
  add_face(Vertex(x0, y1, z0), Vertex(x1, y0, z0), Vertex(x1, y1, z0), Vector3D(0, 0, -1));
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

bool Object::GetLocalBounds(Point3D &outMin, Point3D &outMax) const {
  if (master.empty())
    return false;
  bool initialized = false;
  Point3D mn, mx;
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
