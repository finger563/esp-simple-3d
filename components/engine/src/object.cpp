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
               const Vector3D &vel, Point3D pos, double _rx, double _ry, double _rz) {
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
               const Vector3D &vel, Point3D pos, double _rx, double _ry, double _rz) {
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
  return true;
}

bool Object::updateList(const std::vector<Poly> &poly) {
  clearTemp();
  std::copy(poly.begin(), poly.end(), std::back_inserter(temp));
  return true;
}

void Object::Rotate(Matrix &m) {
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
  double r = cos(phi);
  double x = r * sin(theta), y = sin(phi), z = r * cos(theta);
  Vector3D forward = normalize(Vector3D(x, y, z));
  Vector3D up = normalize(Vector3D(0, 1, 0));
  Vector3D right = normalize(Cross(up, forward));
  up = normalize(Cross(forward, right));
  Matrix m = Matrix();
  m.data[0][0] = right.x;
  m.data[0][1] = right.y;
  m.data[0][2] = right.z;
  m.data[1][0] = up.x;
  m.data[1][1] = up.y;
  m.data[1][2] = up.z;
  m.data[2][0] = forward.x;
  m.data[2][1] = forward.y;
  m.data[2][2] = forward.z;
  Rotate(m);
}

void Object::RotateToHeading(const Vector3D &changeUp) {
  double r = cos(phi);
  double x = r * sin(theta), y = sin(phi), z = r * cos(theta);
  Vector3D forward = normalize(Vector3D(x, y, z));
  Vector3D up = normalize(changeUp);
  Vector3D right = normalize(Cross(up, forward));
  up = normalize(Cross(forward, right));
  Matrix m = Matrix();
  m.data[0][0] = right.x;
  m.data[0][1] = right.y;
  m.data[0][2] = right.z;
  m.data[1][0] = up.x;
  m.data[1][1] = up.y;
  m.data[1][2] = up.z;
  m.data[2][0] = forward.x;
  m.data[2][1] = forward.y;
  m.data[2][2] = forward.z;
  Rotate(m);
}

void Object::clearTemp() {
  // empties temp list
  temp.clear();
}

void Object::RotateTemp(const Matrix &m) {
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
  double r = cos(phi);
  double x = r * sin(theta), y = sin(phi), z = r * cos(theta);
  Vector3D forward = normalize(Vector3D(x, y, z));
  Vector3D up = normalize(Vector3D(0, 1, 0));
  Vector3D right = normalize(Cross(up, forward));
  up = normalize(Cross(forward, right));
  Matrix m = Matrix();
  m.data[0][0] = right.x;
  m.data[0][1] = right.y;
  m.data[0][2] = right.z;
  m.data[1][0] = up.x;
  m.data[1][1] = up.y;
  m.data[1][2] = up.z;
  m.data[2][0] = forward.x;
  m.data[2][1] = forward.y;
  m.data[2][2] = forward.z;
  RotateTemp(m);
}

void Object::add(const Poly &poly) {
  master.push_back(poly);
  updateList();
}

void Object::GenerateCube(double size) {
  master.clear();

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
    it.SetColor(rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, rand() / (double)RAND_MAX);
#if 0
		it.SetVertexColors(rand()/(double)RAND_MAX,rand()/(double)RAND_MAX,rand()/(double)RAND_MAX,
			rand()/(double)RAND_MAX,rand()/(double)RAND_MAX,rand()/(double)RAND_MAX,
			rand()/(double)RAND_MAX,rand()/(double)RAND_MAX,rand()/(double)RAND_MAX);
#else
    it.SetVertexColors(0.9, 0, 0, 0, 0.9, 0, 0, 0, 0.9);
#endif
  }

  rx = size;
  ry = size;
  rz = size;
  theta = 3.141592;
  phi = 0;

  updateList();
}

void Object::GenerateTetra(double size) {
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

  master.clear();

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

void Object::GenerateFloor(double length, double depth) {

  master.clear();
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
void Object::GenerateCeiling(double length, double depth) {
  position = Point3D(0, depth, 0);

  theta = 0;
  phi = 0;

  master.clear();
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
void Object::GenerateWall(size_t type, double length, double depth) {

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

  master.clear();
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

void Object::GenerateShot(const Vector3D &pos, double theta_, double phi_) {
  master.clear();

  master.push_back(Poly(Vertex(0, 0, 4, 1), Vertex(0, 2, -2, 1), Vertex(0, -2, -2, 1), Vertex(), 3,
                        Vector3D(1, 0, 0), COLORED));
  master.begin()->SetDoubleSided(true);
  master.begin()->SetVertexColors(
      rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, rand() / (double)RAND_MAX,
      rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, rand() / (double)RAND_MAX,
      rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, rand() / (double)RAND_MAX);

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

void Object::GeneratePlayer(const Vector3D &pos, double theta_, double phi_,
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

bool Object::SetBoudingEllipsoid(double x, double y, double z) {
  rx = x;
  ry = y;
  rz = z;
  return true;
}

double Object::GetRadiusX(void) const { return rx; }

double Object::GetRadiusY(void) const { return ry; }

double Object::GetRadiusZ(void) const { return rz; }

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
  double distance = magnitude(b.GetPosition() - position);
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
      double t = (-it.normal * (p0 - A)) / (it.normal * (p1 - p0));
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
