#include "polygon.hpp"

double z_buffer[SIZE_X * SIZE_Y];
short display_buffer[SIZE_X * SIZE_Y];

// General Transformation Methods, only operate on x,y,z,w of vertices
void Poly::Transform(const Matrix &_m) {
  v[0].Transform(_m);
  v[1].Transform(_m);
  v[2].Transform(_m);
  if (numVertices == 4)
    v[3].Transform(_m);
  normal = (_m * normal);
}

void Poly::Translate(const Vector3D &_v) {
  v[0].Translate(_v);
  v[1].Translate(_v);
  v[2].Translate(_v);
  if (numVertices == 4)
    v[3].Translate(_v);
}

void Poly::Translate(const double _x, const double _y, const double _z) {
  v[0].Translate(_x, _y, _z);
  v[1].Translate(_x, _y, _z);
  v[2].Translate(_x, _y, _z);
  if (numVertices == 4)
    v[3].Translate(_x, _y, _z);
}

// Pipeline Transformation Methods
void Poly::TransformToCamera(const Matrix &_m) {
  v[0].TransformToCamera(_m);
  v[1].TransformToCamera(_m);
  v[2].TransformToCamera(_m);
  if (numVertices == 4)
    v[3].TransformToCamera(_m);
  normal = (_m * normal);
}

void Poly::TransformToPerspective(const Matrix &_m) {
  v[0].TransformToPerspective(_m);
  v[1].TransformToPerspective(_m);
  v[2].TransformToPerspective(_m);
  if (numVertices == 4)
    v[3].TransformToPerspective(_m);
  // normal = _m*normal;
  visible = false;
  Vector3D eye = Vector3D(0, 0, -1);
  Vector3D cull = eye - Vector3D(v[0].ex, v[0].ey, v[0].ez);
  double test = cull * normal;
  if (test > 0) {
    visible = true; // Triangle is renderable
    return;
  }
  cull = eye - Vector3D(v[1].ex, v[1].ey, v[1].ez);
  test = cull * normal;
  if (test > 0) {
    visible = true; // Triangle is renderable
    return;
  }
  cull = eye - Vector3D(v[2].ex, v[2].ey, v[2].ez);
  test = cull * normal;
  if (test > 0) {
    visible = true; // Triangle is renderable
    return;
  }
  if (numVertices == 4) {
    cull = eye - Vector3D(v[3].ex, v[3].ey, v[3].ez);
    test = cull * normal;
    if (test > 0) {
      visible = true; // Triangle is renderable
    }
  }
}

void Poly::TransformToPixel(const Matrix &_m) {
  v[0].Transform(_m);
  v[1].Transform(_m);
  v[2].Transform(_m);
  if (numVertices == 4)
    v[3].Transform(_m); // only transform the pixel coords (x,y,z)
}

// Pipeline function methods
void Poly::Clip() {
  double BC[POLY_MAX_VERTICES][6] = {}; // boundary tests
  int line[POLY_MAX_VERTICES][6] = {};  // which lines cross
  int lines[6] = {};

  for (int i = 0; i < numVertices; i++) {
    BC[i][0] = v[i].x;
    BC[i][1] = v[i].w - v[i].x;
    BC[i][2] = v[i].y;
    BC[i][3] = v[i].w - v[i].y;
    BC[i][4] = v[i].z;
    BC[i][5] = v[i].w - v[i].z;
  }

  for (int i = 0; i < numVertices - 1; i++) {
    line[i][0] = ((BC[i][0] > 0 && BC[i + 1][0] < 0) || (BC[i][0] < 0 && BC[i + 1][0] > 0)) ? 1 : 0;
    line[i][1] = ((BC[i][1] > 0 && BC[i + 1][1] < 0) || (BC[i][1] < 0 && BC[i + 1][1] > 0)) ? 1 : 0;
    line[i][2] = ((BC[i][2] > 0 && BC[i + 1][2] < 0) || (BC[i][2] < 0 && BC[i + 1][2] > 0)) ? 1 : 0;
    line[i][3] = ((BC[i][3] > 0 && BC[i + 1][3] < 0) || (BC[i][3] < 0 && BC[i + 1][3] > 0)) ? 1 : 0;
    line[i][4] = ((BC[i][4] > 0 && BC[i + 1][4] < 0) || (BC[i][4] < 0 && BC[i + 1][4] > 0)) ? 1 : 0;
    line[i][5] = ((BC[i][5] > 0 && BC[i + 1][5] < 0) || (BC[i][5] < 0 && BC[i + 1][5] > 0)) ? 1 : 0;
  }
  line[numVertices - 1][0] =
      ((BC[numVertices - 1][0] > 0 && BC[0][0] < 0) || (BC[numVertices - 1][0] < 0 && BC[0][0] > 0))
          ? 1
          : 0;
  line[numVertices - 1][1] =
      ((BC[numVertices - 1][1] > 0 && BC[0][1] < 0) || (BC[numVertices - 1][1] < 0 && BC[0][1] > 0))
          ? 1
          : 0;
  line[numVertices - 1][2] =
      ((BC[numVertices - 1][2] > 0 && BC[0][2] < 0) || (BC[numVertices - 1][2] < 0 && BC[0][2] > 0))
          ? 1
          : 0;
  line[numVertices - 1][3] =
      ((BC[numVertices - 1][3] > 0 && BC[0][3] < 0) || (BC[numVertices - 1][3] < 0 && BC[0][3] > 0))
          ? 1
          : 0;
  line[numVertices - 1][4] =
      ((BC[numVertices - 1][4] > 0 && BC[0][4] < 0) || (BC[numVertices - 1][4] < 0 && BC[0][4] > 0))
          ? 1
          : 0;
  line[numVertices - 1][5] =
      ((BC[numVertices - 1][5] > 0 && BC[0][5] < 0) || (BC[numVertices - 1][5] < 0 && BC[0][5] > 0))
          ? 1
          : 0;

  int num = 1;
  for (int i = 0; i < numVertices; i++) {
    lines[0] += num * line[i][0];
    lines[1] += num * line[i][1];
    lines[2] += num * line[i][2];
    lines[3] += num * line[i][3];
    lines[4] += num * line[i][4];
    lines[5] += num * line[i][5];
    num *= 2;
  }

  // bool test = false;
  // int linetest = 0;
  // for (int i=0;i<4;i++) {		// test x and y screen planes
  //	linetest += lines[i];
  // }
  // if ( linetest == 0 &&		// no lines cross any boundaries
  //	 ( v[0].x < -1 || v[0].x > 1 ||
  //	   v[1].x < -1 || v[1].x > 1 ||
  //	   v[2].x < -1 || v[2].x > 1 ||
  //	   v[0].y < -1 || v[0].y > 1 ||
  //	   v[1].y < -1 || v[1].y > 1 ||
  //	   v[2].y < -1 || v[2].y > 1
  //	   )
  //	 ) {
  //	visible = false;
  //	return;
  // }

  Vertex sv = Vertex(), ev = Vertex();
  double a1, a2;

  if (numVertices == 3) { // Poly is a triangle
    switch (lines[4]) {
    default:
      return;
      break;
    case 3: // Lines 0 and 1 cross scanline
      a1 = BC[0][4] / (BC[0][4] - BC[1][4]);
      a2 = BC[1][4] / (BC[1][4] - BC[2][4]);
      sv = v[0] + (v[1] - v[0]) * a1;
      ev = v[1] + (v[2] - v[1]) * a2;
      if (BC[1][4] < 0) { // 1 is shared vertex
        v[3] = v[2];
        v[2] = ev;
        v[1] = sv;
        numVertices = 4;
      } else {
        v[0] = sv;
        v[2] = ev;
      }
      break;
    case 5: // Lines 0 and 2 cross scanline
      a1 = BC[0][4] / (BC[0][4] - BC[1][4]);
      a2 = BC[2][4] / (BC[2][4] - BC[0][4]);
      sv = v[0] + (v[1] - v[0]) * a1;
      ev = v[2] + (v[0] - v[2]) * a2;
      if (BC[0][4] < 0) { // 0 is shared vertex
        v[0] = sv;
        v[3] = ev;
        numVertices = 4;
      } else {
        v[1] = sv;
        v[2] = ev;
      }
      break;
    case 6: // Lines 1 and 2 cross scanline
      a1 = BC[1][4] / (BC[1][4] - BC[2][4]);
      a2 = BC[2][4] / (BC[2][4] - BC[0][4]);
      sv = v[1] + (v[2] - v[1]) * a1;
      ev = v[2] + (v[0] - v[2]) * a2;
      if (BC[2][4] < 0) { // 2 is shared vertex
        v[2] = sv;
        v[3] = ev;
        numVertices = 4;
      } else {
        v[1] = sv;
        v[0] = ev;
      }
      break;
    }
  } else {
    return; // don't support quad clipping right now
    switch (lines[4]) {
    default:
      break;
    case 3: // Lines 0 and 1 cross scanline
      a1 = BC[0][4] / (BC[0][4] - BC[1][4]);
      a2 = BC[1][4] / (BC[1][4] - BC[2][4]);
      sv = v[0] + (v[1] - v[0]) * a1;
      ev = v[1] + (v[2] - v[1]) * a2;
      break;
    case 5: // Lines 0 and 2 cross scanline
      a1 = BC[0][4] / (BC[0][4] - BC[1][4]);
      a2 = BC[2][4] / (BC[2][4] - BC[3][4]);
      sv = v[0] + (v[1] - v[0]) * a1;
      ev = v[2] + (v[3] - v[2]) * a2;
      break;
    case 6: // Lines 1 and 2 cross scanline
      a1 = BC[1][4] / (BC[1][4] - BC[2][4]);
      a2 = BC[2][4] / (BC[2][4] - BC[3][4]);
      sv = v[1] + (v[2] - v[1]) * a1;
      ev = v[2] + (v[3] - v[2]) * a2;
      break;
    case 9: // Lines 0 and 3 cross scanline
      a1 = BC[0][4] / (BC[0][4] - BC[1][4]);
      a2 = BC[3][4] / (BC[3][4] - BC[0][4]);
      sv = v[0] + (v[1] - v[0]) * a1;
      ev = v[3] + (v[0] - v[3]) * a2;
      break;
    case 10: // Lines 1 and 3 cross scanline
      a1 = BC[1][4] / (BC[1][4] - BC[2][4]);
      a2 = BC[3][4] / (BC[3][4] - BC[0][4]);
      sv = v[1] + (v[2] - v[1]) * a1;
      ev = v[3] + (v[0] - v[3]) * a2;
      break;
    case 12: // Lines 2 and 3 cross scanline
      a1 = BC[2][4] / (BC[2][4] - BC[3][4]);
      a2 = BC[3][4] / (BC[3][4] - BC[0][4]);
      sv = v[2] + (v[3] - v[2]) * a1;
      ev = v[3] + (v[0] - v[3]) * a2;
      break;
    }
  }
}

void Poly::HomogeneousDivide() {
  v[0].HomogeneousDivide();
  v[1].HomogeneousDivide();
  v[2].HomogeneousDivide();
  if (numVertices == 4)
    v[3].HomogeneousDivide();
}

void Poly::SetupRasterization() {
  double al, ar;

  if (doublesided) {
    Vector3D eye = Vector3D(0, 0, -1);
    Vector3D cull = eye - Vector3D(v[0].ex, v[0].ey, v[0].ez);
    double test = cull * normal;
    if (test > 0) {
      Vertex temp[4];
      for (int i = 0; i < numVertices; i++)
        temp[i] = v[numVertices - i - 1];
      for (int i = 0; i < numVertices; i++)
        v[i] = temp[i];
    }
  }

  YSort(ySorted);
  Vertex vl, vr;

  switch (rType) {
  default:
  case FLAT:
    numInterps = 4; // just x,y,ez,hw
    break;
  case COLORED:
    numInterps = 7; // just x,y,ez,hw,r,g,b
    break;
  case SMOOTH:
    numInterps = 7; // just x,y,ez,hw,nx,ny,nz
    break;
  case TEXTURED:
    numInterps = 6; // just x,y,ez,hw,u,v
    break;
  case TEXTURED_SMOOTH:
    numInterps = 9; // just x,y,ez,hw,nx,ny,nz,u,v
    break;
  }

  int ind = edges[0], indl = (ind > 0) ? ind - 1 : numVertices - 1, indr = (ind + 1) % numVertices,
      tmp;

  for (int v = 0; v < POLY_MAX_VERTICES; v++) {
    for (int i = 0; i < NUM_VERTEX_DATA; i++) {
      increments[v][0][i] = vl[i];
      increments[v][1][i] = vr[i];
    }
  }

  al = 1 / (v[ind].y - v[indl].y); //- 1);
  ar = 1 / (v[ind].y - v[indr].y); //- 1);
  vl = (v[indl] - v[ind]) * al;
  vr = (v[indr] - v[ind]) * ar;

  for (int i = 0; i < NUM_VERTEX_DATA; i++) {
    increments[0][0][i] = vl[i];
    increments[0][1][i] = vr[i];
  }
  sides[0][0] = ind;
  sides[0][1] = ind;

  for (int j = 1; j < 4; j++) {
    if (v[indr].y > v[indl].y) { // go down right of poly
      ind = indr;
      indr = (indr + 1) % numVertices;
      ar = 1 / (v[ind].y - v[indr].y); //- 1);
      vr = (v[indr] - v[ind]) * ar;
      for (int i = 0; i < NUM_VERTEX_DATA; i++) {
        increments[j][0][i] = increments[j - 1][0][i];
        increments[j][1][i] = vr[i];
      }
      sides[j][0] = sides[j - 1][0];
      sides[j][1] = ind;
    } else { // go down left of poly
      ind = indl;
      indl = (indl > 0) ? indl - 1 : numVertices - 1;
      al = 1 / (v[ind].y - v[indl].y); //- 1);
      vl = (v[indl] - v[ind]) * al;
      for (int i = 0; i < NUM_VERTEX_DATA; i++) {
        increments[j][0][i] = vl[i];
        increments[j][1][i] = increments[j - 1][1][i];
      }
      sides[j][0] = ind;
      sides[j][1] = sides[j - 1][1];
    }
  }
}

// Rasterization Methods
void Poly::Rasterize() {
  for (int y = MaxY(); y >= MinY(); y--) {
    double BC[POLY_MAX_VERTICES] = {}; // boundary tests against y scanline
    int line[POLY_MAX_VERTICES] = {};  // which lines cross y scanline
    int lines = 0;
    double a1, a2,
        ai; // alphas for each crossing line (there can only be 2), and for inside scanline
    Vertex sv, ev, vi; // Start and end scanline vertices, and rendering vertex

    for (int i = 0; i < numVertices; i++) {
      BC[i] = v[i].y - y; // + is above, - is below, 0 is on
    }

    if (numVertices == 3) {
      line[0] = ((BC[0] > 0 && BC[1] < 0) || (BC[0] < 0 && BC[1] > 0)) ? 1 : 0;
      line[1] = ((BC[1] > 0 && BC[2] < 0) || (BC[1] < 0 && BC[2] > 0)) ? 1 : 0;
      line[2] = ((BC[2] > 0 && BC[0] < 0) || (BC[2] < 0 && BC[0] > 0)) ? 1 : 0;
      lines = line[0] + line[1] * 2 + line[2] * 4;
      switch (lines) {
      default:  // These cases represent a degenerate triangle
        return; // therefore do not do anything else
      case 3:   // Lines 0 and 1 cross scanline
        a1 = BC[0] / (BC[0] - BC[1]);
        a2 = BC[1] / (BC[1] - BC[2]);
        sv = v[0] + (v[1] - v[0]) * a1;
        ev = v[1] + (v[2] - v[1]) * a2;
        break;
      case 5: // Lines 0 and 2 cross scanline
        a1 = BC[0] / (BC[0] - BC[1]);
        a2 = BC[2] / (BC[2] - BC[0]);
        sv = v[0] + (v[1] - v[0]) * a1;
        ev = v[2] + (v[0] - v[2]) * a2;
        break;
      case 6: // Lines 1 and 2 cross scanline
        a1 = BC[1] / (BC[1] - BC[2]);
        a2 = BC[2] / (BC[2] - BC[0]);
        sv = v[1] + (v[2] - v[1]) * a1;
        ev = v[2] + (v[0] - v[2]) * a2;
        break;
      }
    } else { // Poly is a QUAD
      line[0] = ((BC[0] > 0 && BC[1] < 0) || (BC[0] < 0 && BC[1] > 0)) ? 1 : 0;
      line[1] = ((BC[1] > 0 && BC[2] < 0) || (BC[1] < 0 && BC[2] > 0)) ? 1 : 0;
      line[2] = ((BC[2] > 0 && BC[3] < 0) || (BC[2] < 0 && BC[3] > 0)) ? 1 : 0;
      line[3] = ((BC[3] > 0 && BC[0] < 0) || (BC[3] < 0 && BC[0] > 0)) ? 1 : 0;
      lines = line[0] + line[1] * 2 + line[2] * 4 + line[3] * 8;
      switch (lines) {
      default:  // These cases represent a degenerate triangle
        return; // therefore do not do anything else
      case 3:   // Lines 0 and 1 cross scanline
        a1 = BC[0] / (BC[0] - BC[1]);
        a2 = BC[1] / (BC[1] - BC[2]);
        sv = v[0] + (v[1] - v[0]) * a1;
        ev = v[1] + (v[2] - v[1]) * a2;
        break;
      case 5: // Lines 0 and 2 cross scanline
        a1 = BC[0] / (BC[0] - BC[1]);
        a2 = BC[2] / (BC[2] - BC[3]);
        sv = v[0] + (v[1] - v[0]) * a1;
        ev = v[2] + (v[3] - v[2]) * a2;
        break;
      case 6: // Lines 1 and 2 cross scanline
        a1 = BC[1] / (BC[1] - BC[2]);
        a2 = BC[2] / (BC[2] - BC[3]);
        sv = v[1] + (v[2] - v[1]) * a1;
        ev = v[2] + (v[3] - v[2]) * a2;
        break;
      case 9: // Lines 0 and 3 cross scanline
        a1 = BC[0] / (BC[0] - BC[1]);
        a2 = BC[3] / (BC[3] - BC[0]);
        sv = v[0] + (v[1] - v[0]) * a1;
        ev = v[3] + (v[0] - v[3]) * a2;
        break;
      case 10: // Lines 1 and 3 cross scanline
        a1 = BC[1] / (BC[1] - BC[2]);
        a2 = BC[3] / (BC[3] - BC[0]);
        sv = v[1] + (v[2] - v[1]) * a1;
        ev = v[3] + (v[0] - v[3]) * a2;
        break;
      case 12: // Lines 2 and 3 cross scanline
        a1 = BC[2] / (BC[2] - BC[3]);
        a2 = BC[3] / (BC[3] - BC[0]);
        sv = v[2] + (v[3] - v[2]) * a1;
        ev = v[3] + (v[0] - v[3]) * a2;
        break;
      }
    }
    if (sv.x > ev.x) { // need to flip start and end vertices
      Vertex temp = ev;
      ev = sv;
      sv = temp;
    }
    vi = sv;
    if (vi.x < 0) {
      vi.x = 0;
    }
    if (ev.x >= SIZE_X) {
      ai = (sv.x - (SIZE_X - 1)) / ((sv.x - (SIZE_X - 1)) - (ev.x - (SIZE_X - 1)));
      ev = sv + (ev - sv) * ai;
    }
    if (floor(vi.x) == ceil(ev.x))
      return;
    for (int x = ceil(vi.x); x <= floor(ev.x); x++) {
      ai = (sv.x - x) / ((sv.x - x) - (ev.x - x));
      vi = sv + (ev - sv) * ai;
      if (vi.ez / vi.hw < z_buffer[x + y * SIZE_X]) {
        z_buffer[x + y * SIZE_X] = vi.ez / vi.hw;
        switch (rType) { // What are we interpolating/rendering?
        case FLAT:
          display_buffer[x + y * SIZE_X] = RGB_MAKE((int)(r * 255), (int)(g * 255), (int)(b * 255));
          break;
        case COLORED:
          vi.r = vi.r / vi.hw; // divide all interpolated values by hw
          vi.g = vi.g / vi.hw; // divide all interpolated values by hw
          vi.b = vi.b / vi.hw; // divide all interpolated values by hw
          display_buffer[x + y * SIZE_X] =
              RGB_MAKE((int)(vi.r * 255), (int)(vi.g * 255), (int)(vi.g * 255));
          break;
        case SMOOTH:
          break;
        case TEXTURED:
          vi.u = vi.u / vi.hw; // divide all interpolated values by hw
          vi.v = vi.v / vi.hw; // divide all interpolated values by hw
          if (vi.u < 0 || vi.u > 1)
            vi.u = 0;
          if (vi.v < 0 || vi.v > 1)
            vi.v = 0;
          display_buffer[x + y * SIZE_X] =
              texture[(int)(vi.u * (texwidth - 1)) + ((int)(vi.v * (texheight - 1))) * texwidth];
          break;
        case TEXTURED_SMOOTH:
          break;
        default:
          break;
        }
      }
    }
  }
}

void Poly::Rasterize(const int y) {
  if (y > MaxY() || y < MinY())
    return;
  double BC[POLY_MAX_VERTICES] = {}; // boundary tests against y scanline
  int line[POLY_MAX_VERTICES] = {};  // which lines cross y scanline
  int lines = 0;
  double a1, a2, ai; // alphas for each crossing line (there can only be 2), and for inside scanline
  Vertex sv, ev, vi; // Start and end scanline vertices, and rendering vertex

  for (int i = 0; i < numVertices; i++) {
    BC[i] = v[i].y - y; // + is above, - is below, 0 is on
  }

  if (numVertices == 3) {
    line[0] = ((BC[0] >= 0 && BC[1] <= 0) || (BC[0] <= 0 && BC[1] >= 0)) ? 1 : 0;
    line[1] = ((BC[1] >= 0 && BC[2] <= 0) || (BC[1] <= 0 && BC[2] >= 0)) ? 1 : 0;
    line[2] = ((BC[2] >= 0 && BC[0] <= 0) || (BC[2] <= 0 && BC[0] >= 0)) ? 1 : 0;
    lines = line[0] + line[1] * 2 + line[2] * 4;
    switch (lines) {
    default:  // These cases represent a degenerate triangle
      return; // therefore do not do anything else
    case 3:   // Lines 0 and 1 cross scanline
      a1 = BC[0] / (BC[0] - BC[1]);
      a2 = BC[1] / (BC[1] - BC[2]);
      sv = v[0] + (v[1] - v[0]) * a1;
      ev = v[1] + (v[2] - v[1]) * a2;
      break;
    case 5: // Lines 0 and 2 cross scanline
      a1 = BC[0] / (BC[0] - BC[1]);
      a2 = BC[2] / (BC[2] - BC[0]);
      sv = v[0] + (v[1] - v[0]) * a1;
      ev = v[2] + (v[0] - v[2]) * a2;
      break;
    case 6: // Lines 1 and 2 cross scanline
      a1 = BC[1] / (BC[1] - BC[2]);
      a2 = BC[2] / (BC[2] - BC[0]);
      sv = v[1] + (v[2] - v[1]) * a1;
      ev = v[2] + (v[0] - v[2]) * a2;
      break;
    }
  } else { // Poly is a QUAD
    line[0] = ((BC[0] > 0 && BC[1] < 0) || (BC[0] < 0 && BC[1] > 0)) ? 1 : 0;
    line[1] = ((BC[1] > 0 && BC[2] < 0) || (BC[1] < 0 && BC[2] > 0)) ? 1 : 0;
    line[2] = ((BC[2] > 0 && BC[3] < 0) || (BC[2] < 0 && BC[3] > 0)) ? 1 : 0;
    line[3] = ((BC[3] > 0 && BC[0] < 0) || (BC[3] < 0 && BC[0] > 0)) ? 1 : 0;
    lines = line[0] + line[1] * 2 + line[2] * 4 + line[3] * 8;
    switch (lines) {
    default:  // These cases represent a degenerate triangle
      return; // therefore do not do anything else
    case 3:   // Lines 0 and 1 cross scanline
      a1 = BC[0] / (BC[0] - BC[1]);
      a2 = BC[1] / (BC[1] - BC[2]);
      sv = v[0] + (v[1] - v[0]) * a1;
      ev = v[1] + (v[2] - v[1]) * a2;
      break;
    case 5: // Lines 0 and 2 cross scanline
      a1 = BC[0] / (BC[0] - BC[1]);
      a2 = BC[2] / (BC[2] - BC[3]);
      sv = v[0] + (v[1] - v[0]) * a1;
      ev = v[2] + (v[3] - v[2]) * a2;
      break;
    case 6: // Lines 1 and 2 cross scanline
      a1 = BC[1] / (BC[1] - BC[2]);
      a2 = BC[2] / (BC[2] - BC[3]);
      sv = v[1] + (v[2] - v[1]) * a1;
      ev = v[2] + (v[3] - v[2]) * a2;
      break;
    case 9: // Lines 0 and 3 cross scanline
      a1 = BC[0] / (BC[0] - BC[1]);
      a2 = BC[3] / (BC[3] - BC[0]);
      sv = v[0] + (v[1] - v[0]) * a1;
      ev = v[3] + (v[0] - v[3]) * a2;
      break;
    case 10: // Lines 1 and 3 cross scanline
      a1 = BC[1] / (BC[1] - BC[2]);
      a2 = BC[3] / (BC[3] - BC[0]);
      sv = v[1] + (v[2] - v[1]) * a1;
      ev = v[3] + (v[0] - v[3]) * a2;
      break;
    case 12: // Lines 2 and 3 cross scanline
      a1 = BC[2] / (BC[2] - BC[3]);
      a2 = BC[3] / (BC[3] - BC[0]);
      sv = v[2] + (v[3] - v[2]) * a1;
      ev = v[3] + (v[0] - v[3]) * a2;
      break;
    }
  }
  if (sv.x > ev.x) { // need to flip start and end vertices
    Vertex temp = ev;
    ev = sv;
    sv = temp;
  }
  vi = sv;
  if (vi.x < 0) {
    vi.x = 0;
  }
  if (ev.x < 0)
    return;
  if (ev.x >= SIZE_X) {
    ai = (sv.x - (SIZE_X - 1)) / ((sv.x - (SIZE_X - 1)) - (ev.x - (SIZE_X - 1)));
    ev = sv + (ev - sv) * ai;
  }
  if (floor(vi.x) == ceil(ev.x))
    return;
  for (int x = vi.x; x <= ev.x; x++) {
    ai = (sv.x - x) / ((sv.x - x) - (ev.x - x));
    vi = sv + (ev - sv) * ai;
    if (vi.ez / vi.hw < z_buffer[x + y * SIZE_X]) {
      z_buffer[x + y * SIZE_X] = vi.ez / vi.hw;
      switch (rType) { // What are we interpolating/rendering?
      case FLAT:
        display_buffer[x + y * SIZE_X] =
            RGB_MAKE((char)(r * 255.0), (char)(g * 255.0), (char)(b * 255.0));
        break;
      case COLORED:
        vi.r = vi.r / vi.hw; // divide all interpolated values by hw
        vi.g = vi.g / vi.hw; // divide all interpolated values by hw
        vi.b = vi.b / vi.hw; // divide all interpolated values by hw
        display_buffer[x + y * SIZE_X] =
            RGB_MAKE((char)(vi.r * 255.0), (char)(vi.g * 255.0), (char)(vi.b * 255.0));
        break;
      case SMOOTH:
        break;
      case TEXTURED:
        vi.u = vi.u / vi.hw; // divide all interpolated values by hw
        vi.v = vi.v / vi.hw; // divide all interpolated values by hw
        if (vi.u < 0 || vi.u > 1)
          vi.u = 0;
        if (vi.v < 0 || vi.v > 1)
          vi.v = 0;
        display_buffer[x + y * SIZE_X] =
            texture[(int)(vi.u * (texwidth - 1)) + ((int)(vi.v * (texheight - 1))) * texwidth];
        break;
      case TEXTURED_SMOOTH:
        break;
      default:
        break;
      }
    }
  }
}

void Poly::RasterizeFast() {}

void Poly::RasterizeFast(const int y) {
  if (y > ySorted[0].y || (y < ySorted[2].y && numVertices == 3) || y <= ySorted[3].y)
    return;

  double interp[2][NUM_VERTEX_DATA]; // on the stack for speed, but only use numInterps
  double px[NUM_VERTEX_DATA];        // For the pixel in the scanline

  double dyl, dyr, ai;
  double dx;
  int depthindex = -1, leftindex = 0, rightindex = 0;

  if (y < ySorted[0].y && y > ySorted[1].y &&
      (ySorted[0].y) != (ySorted[1].y)) { // We are between 1st and 2nd vertex
    depthindex = 0;
  } else if (y <= ySorted[1].y && // We are between 2nd and 3rd vertex
             y > ySorted[2].y) {
    depthindex = 1;
  } else if (numVertices == 4 && // QUAD: we are between 3rd and 4th vertex
             y <= ySorted[2].y && y > ySorted[3].y) {
    depthindex = 2;
  }
  if (depthindex == -1)
    return;
  leftindex = sides[depthindex][0], rightindex = sides[depthindex][1];
  dyl = v[leftindex].y - y;
  dyr = v[rightindex].y - y;

  for (int i = 0; i < NUM_VERTEX_DATA; i++) {
    interp[0][i] = increments[depthindex][0][i] * dyl + v[leftindex][i];
    interp[1][i] = increments[depthindex][1][i] * dyr + v[rightindex][i];
  }

  double sx = (interp[0][0]), ex = (interp[1][0]);
  ai = 1 / (interp[0][0] - interp[1][0] - 1);
  if (ex < 0 || sx >= SIZE_X)
    return;
  if (sx < 0)
    sx = 0;
  if (ex >= SIZE_X)
    ex = SIZE_X - 1;

  for (int x = sx; x <= ex; x++) {
    dx = (interp[0][0] - x);
    for (int i = 0; i < NUM_VERTEX_DATA; i++)
      px[i] = interp[0][i] + (interp[1][i] - interp[0][i]) * ai * dx;
    if (px[6] / px[NUM_VERTEX_DATA - 1] < z_buffer[x + y * SIZE_X]) {
      z_buffer[x + y * SIZE_X] = px[6] / px[NUM_VERTEX_DATA - 1];
      switch (rType) {
      default:
      case FLAT:
        display_buffer[x + y * SIZE_X] =
            RGB_MAKE((char)(r * 255.0), (char)(g * 255.0), (char)(b * 255.0));
        break;
      case COLORED:
        px[9] = px[9] / px[NUM_VERTEX_DATA - 1];   // divide all interpolated values by hw
        px[10] = px[10] / px[NUM_VERTEX_DATA - 1]; // divide all interpolated values by hw
        px[11] = px[11] / px[NUM_VERTEX_DATA - 1]; // divide all interpolated values by hw
        display_buffer[x + y * SIZE_X] =
            RGB_MAKE((char)(px[9] * 255.0), (char)(px[10] * 255.0), (char)(px[11] * 255.0));
        break;
      case SMOOTH:
        break;
      case TEXTURED:
        px[7] = px[7] / px[NUM_VERTEX_DATA - 1];
        px[8] = px[8] / px[NUM_VERTEX_DATA - 1];
        if (px[7] < 0 || px[7] > 1)
          px[7] = 0;
        if (px[8] < 0 || px[8] > 1)
          px[8] = 0;
        display_buffer[x + y * SIZE_X] =
            texture[(int)(px[7] * (texwidth - 1)) + ((int)(px[8] * (texheight - 1))) * texwidth];
        break;
      case TEXTURED_SMOOTH:
        break;
      }
    }
  }
}

// Helper Functions
double Poly::MinX() {
  double min = v[0].x;
  for (int i = 0; i < numVertices; i++) {
    if (min > v[i].x)
      min = v[i].x;
  }
  return min;
}

double Poly::MinY() {
  double min = v[0].y;
  for (int i = 0; i < numVertices; i++) {
    if (min > v[i].y)
      min = v[i].y;
  }
  return min;
}

double Poly::MinZ() {
  double min = v[0].z;
  for (int i = 0; i < numVertices; i++) {
    if (min > v[i].z)
      min = v[i].z;
  }
  return min;
}

double Poly::MaxX() {
  double max = v[0].x;
  for (int i = 0; i < numVertices; i++) {
    if (max < v[i].x)
      max = v[i].x;
  }
  return max;
}

double Poly::MaxY() {
  double max = v[0].y;
  for (int i = 0; i < numVertices; i++) {
    if (max < v[i].y)
      max = v[i].y;
  }
  return max;
}

double Poly::MaxZ() {
  double max = v[0].z;
  for (int i = 0; i < numVertices; i++) {
    if (max < v[i].z)
      max = v[i].z;
  }
  return max;
}

void Poly::YSort(Vertex *temp) {
  temp[0] = v[0];
  temp[1] = v[1];
  temp[2] = v[2];
  temp[3] = v[3];
  edges[0] = 0;
  edges[1] = 1;
  edges[2] = 2;
  edges[3] = 3;

  if (v[0].y < v[1].y) {
    temp[0] = v[1];
    temp[1] = v[0];
    edges[0] = 1;
    edges[1] = 0;
  }
  if (temp[0].y < v[2].y) { // v[2] has largest y
    temp[2] = temp[1];
    temp[1] = temp[0];
    temp[0] = v[2];
    edges[2] = edges[1];
    edges[1] = edges[0];
    edges[0] = 2;
  } else if (temp[1].y < v[2].y) { // v[2] has 2nd largest y
    temp[2] = temp[1];
    temp[1] = v[2];
    edges[2] = edges[1];
    edges[1] = 2;
  }

  if (numVertices == 4) {
    if (temp[0].y < v[3].y) {
      temp[3] = temp[2];
      temp[2] = temp[1];
      temp[1] = temp[0];
      temp[0] = v[3];
      edges[3] = edges[2];
      edges[2] = edges[1];
      edges[1] = edges[0];
      edges[0] = 3;
    } else {
      if (temp[1].y < v[3].y) {
        temp[3] = temp[2];
        temp[2] = temp[1];
        temp[1] = v[3];
        edges[3] = edges[2];
        edges[2] = edges[1];
        edges[1] = 3;
      } else if (temp[2].y < v[3].y) {
        temp[3] = temp[2];
        temp[2] = v[3];
        edges[3] = edges[2];
        edges[2] = 3;
      }
    }
  }
}

void Poly::XSort(Vertex *temp) {
  temp[0] = v[0];
  temp[1] = v[1];
  temp[2] = v[2];
  temp[3] = v[3];

  if (v[0].x < v[1].x) {
    temp[0] = v[1];
    temp[1] = v[0];
  }
  if (temp[0].x < v[2].x) { // v[2] has largest x
    temp[2] = temp[1];
    temp[1] = temp[0];
    temp[0] = v[2];
  } else if (temp[1].x < v[2].x) { // v[2] has 2nd largest x
    temp[2] = temp[1];
    temp[1] = v[2];
  }

  if (numVertices == 4) {
    if (temp[0].x < v[3].x) {
      temp[3] = temp[2];
      temp[2] = temp[1];
      temp[1] = temp[0];
      temp[0] = v[3];
    } else {
      if (temp[1].x < v[3].x) {
        temp[3] = temp[2];
        temp[2] = temp[1];
        temp[1] = v[3];
      } else if (temp[2].x < v[3].x) {
        temp[3] = temp[2];
        temp[2] = v[3];
      }
    }
  }
}

void Poly::ZSort(Vertex *temp) {
  temp[0] = v[0];
  temp[1] = v[1];
  temp[2] = v[2];
  temp[3] = v[3];
  if (v[0].z < v[1].z) {
    temp[0] = v[1];
    temp[1] = v[0];
  }
  if (temp[0].z < v[2].z) { // v[2] has largest z
    temp[2] = temp[1];
    temp[1] = temp[0];
    temp[0] = v[2];
  } else if (temp[1].z < v[2].z) { // v[2] has 2nd largest z
    temp[2] = temp[1];
    temp[1] = v[2];
  }
  if (numVertices == 4) {
    if (temp[0].z < v[3].z) {
      temp[3] = temp[2];
      temp[2] = temp[1];
      temp[1] = temp[0];
      temp[0] = v[3];
    } else {
      if (temp[1].z < v[3].z) {
        temp[3] = temp[2];
        temp[2] = temp[1];
        temp[1] = v[3];
      } else if (temp[2].z < v[3].z) {
        temp[3] = temp[2];
        temp[2] = v[3];
      }
    }
  }
}

// Operator Overloads
Poly &Poly::operator=(const Poly &rhs) {
  for (int i = 0; i < POLY_MAX_VERTICES; i++)
    v[i] = rhs.v[i];

  numVertices = rhs.numVertices;

  texture = rhs.texture;
  texwidth = rhs.texwidth;

  r = rhs.r;
  g = rhs.g;
  b = rhs.b;

  rType = rhs.rType;

  normal = rhs.normal;
  visible = rhs.visible;
  return (*this);
}

// INCOMPLETE IMPLEMENTATION
bool Poly::operator==(const Poly &rhs) const {
  if (numVertices != rhs.numVertices)
    return false;
  for (int i = 0; i < POLY_MAX_VERTICES; i++) {
    if (v[i] != rhs.v[i])
      return false;
  }
  if (texture != rhs.texture)
    return false;
  if (texwidth != rhs.texwidth)
    return false;
  if (r != rhs.r || g != rhs.g || b != rhs.b)
    return false;
  if (rType != rhs.rType)
    return false;
  if (normal != rhs.normal)
    return false;
  if (visible != rhs.visible)
    return false;
  return true;
}
