#include "polygon.hpp"
#include "render_layout.hpp"
#include <cmath>
// Helper to rasterize a single triangle (no Poly construction)
void RasterizeTriangle(const Vertex &a, const Vertex &b, const Vertex &c, RenderType rt,
                       const unsigned short *texture, int texwidth, int texheight, float cr,
                       float cg, float cb) {
  // Copy and y-sort vertices ascending (v0 at top, v2 at bottom)
  Vertex v0 = a, v1 = b, v2 = c;
  if (v1.y < v0.y)
    std::swap(v0, v1);
  if (v2.y < v1.y)
    std::swap(v1, v2);
  if (v1.y < v0.y)
    std::swap(v0, v1);

  // Quick reject if triangle fully outside screen vertically
  if (v2.y < 0 || v0.y >= SIZE_Y)
    return;
  // Degenerate check
  if (std::fabs(v2.y - v0.y) < 1e-6f)
    return;

  auto lerp_vertex = [](const Vertex &a, const Vertex &b, float t) {
    Vertex r;
    r.x = a.x + (b.x - a.x) * t;
    r.y = a.y + (b.y - a.y) * t;
    r.ez = a.ez + (b.ez - a.ez) * t;
    r.hw = a.hw + (b.hw - a.hw) * t;
    r.u = a.u + (b.u - a.u) * t;
    r.v = a.v + (b.v - a.v) * t;
    return r;
  };

  auto span_and_fill = [&](int yStart, int yEnd, const Vertex &leftA, const Vertex &leftB,
                           const Vertex &rightA, const Vertex &rightB) {
    if (yEnd < 0 || yStart >= SIZE_Y)
      return;
    if (yStart < 0)
      yStart = 0;
    if (yEnd >= SIZE_Y)
      yEnd = SIZE_Y - 1;
    const float dyL = (leftB.y - leftA.y);
    const float dyR = (rightB.y - rightA.y);
    if (std::fabs(dyL) < 1e-6f || std::fabs(dyR) < 1e-6f)
      return;
    for (int y = yStart; y <= yEnd; ++y) {
      float tL = ((float)y + 0.0f - leftA.y) / dyL;
      float tR = ((float)y + 0.0f - rightA.y) / dyR;
      Vertex L = lerp_vertex(leftA, leftB, tL);
      Vertex R = lerp_vertex(rightA, rightB, tR);
      if (L.x > R.x)
        std::swap(L, R);

      int xStart = (int)std::ceil(L.x);
      int xEnd = (int)std::floor(R.x);
      if (xEnd < 0 || xStart >= SIZE_X)
        continue;
      if (xStart < 0)
        xStart = 0;
      if (xEnd >= SIZE_X)
        xEnd = SIZE_X - 1;
      const float invDen = 1.0f / (R.x - L.x + 1e-12f);

      float t0 = ((float)xStart - L.x) * invDen;
      float ez = L.ez + (R.ez - L.ez) * t0;
      float hw = L.hw + (R.hw - L.hw) * t0;
      float u = L.u + (R.u - L.u) * t0;
      float v = L.v + (R.v - L.v) * t0;
      const float dez = (R.ez - L.ez) * invDen;
      const float dhw = (R.hw - L.hw) * invDen;
      const float du = (R.u - L.u) * invDen;
      const float dv = (R.v - L.v) * invDen;

      uint16_t flatColor =
          RGB_MAKE((uint8_t)(cr * 255.0f), (uint8_t)(cg * 255.0f), (uint8_t)(cb * 255.0f));
      float *zrow = z_buffer + y * SIZE_X;
      uint16_t *drow = display_buffer + y * SIZE_X;

      if (rt == TEXTURED && texture) {
        const float uScale = (float)(texwidth - 1) * 65536.0f;
        const float vScale = (float)(texheight - 1) * 65536.0f;
        for (int x = xStart; x <= xEnd; ++x) {
          float zval = ez / hw;
          if (zval < zrow[x]) {
            zrow[x] = zval;
            float pu = u / hw;
            float pv = v / hw;
            int u_lt0 = pu<0.0f, u_gt1 = pu> 1.0f;
            int u_in = !(u_lt0 | u_gt1);
            int v_lt0 = pv<0.0f, v_gt1 = pv> 1.0f;
            int v_in = !(v_lt0 | v_gt1);
            pu = u_in * pu + u_gt1 * 1.0f + u_lt0 * 0.0f;
            pv = v_in * pv + v_gt1 * 1.0f + v_lt0 * 0.0f;
            int32_t ufx = (int32_t)(pu * uScale);
            int32_t vfx = (int32_t)(pv * vScale);
            int tx = ufx >> 16;
            int ty = vfx >> 16;
            drow[x] = texture[tx + ty * texwidth];
          }
          ez += dez;
          hw += dhw;
          u += du;
          v += dv;
        }
      } else {
        for (int x = xStart; x <= xEnd; ++x) {
          float zval = ez / hw;
          if (zval < zrow[x]) {
            zrow[x] = zval;
            drow[x] = flatColor;
          }
          ez += dez;
          hw += dhw;
          u += du;
          v += dv;
        }
      }
    }
  };

  const float dy_long = (v2.y - v0.y);
  // Upper half v0->v1
  if (v1.y > v0.y) {
    span_and_fill((int)std::ceil(v0.y), (int)std::floor(v1.y), v0, v1, // short edge v0->v1
                  v0, v2                                               // long edge v0->v2
    );
  }
  // Lower half v1->v2
  if (v2.y > v1.y) {
    span_and_fill((int)std::ceil(v1.y), (int)std::floor(v2.y), v1, v2, // short edge v1->v2
                  v0, v2                                               // long edge v0->v2
    );
  }
}

// General Transformation Methods, only operate on x,y,z,w of vertices
void Poly::Transform(const Matrix &_m) {
  v[0].Transform(_m);
  v[1].Transform(_m);
  v[2].Transform(_m);
  if (numVertices == 4)
    v[3].Transform(_m);
  {
    // Transform normal as a direction (w=0) to avoid translation
    Vector3D n = Vector3D(normal.x, normal.y, normal.z, 0);
    n = _m * n;
    normal.x = n.x;
    normal.y = n.y;
    normal.z = n.z;
  }
}

void Poly::Translate(const Vector3D &_v) {
  v[0].Translate(_v);
  v[1].Translate(_v);
  v[2].Translate(_v);
  if (numVertices == 4)
    v[3].Translate(_v);
}

void Poly::Translate(const float _x, const float _y, const float _z) {
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
  {
    // Transform normal as a direction (w=0) to avoid translation
    Vector3D n = Vector3D(normal.x, normal.y, normal.z, 0);
    n = _m * n;
    normal.x = n.x;
    normal.y = n.y;
    normal.z = n.z;
  }
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
  float test = cull * normal;
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
  float BC[POLY_MAX_VERTICES][6] = {}; // boundary tests
  int line[POLY_MAX_VERTICES][6] = {}; // which lines cross
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
  float a1, a2;

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
  float al, ar;

  if (doublesided) {
    Vector3D eye = Vector3D(0, 0, -1);
    Vector3D cull = eye - Vector3D(v[0].ex, v[0].ey, v[0].ez);
    float test = cull * normal;
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
    float BC[POLY_MAX_VERTICES] = {}; // boundary tests against y scanline
    int line[POLY_MAX_VERTICES] = {}; // which lines cross y scanline
    int lines = 0;
    float a1, a2,
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
      continue;
    uint16_t poly_color = RGB_MAKE((int)(r * 255.0f), (int)(g * 255.0f), (int)(b * 255.0f));
    for (int x = ceil(vi.x); x <= floor(ev.x); x++) {
      ai = (sv.x - x) / ((sv.x - x) - (ev.x - x));
      vi = sv + (ev - sv) * ai;
      if (vi.ez / vi.hw < z_buffer[x + y * SIZE_X]) {
        z_buffer[x + y * SIZE_X] = vi.ez / vi.hw;
        switch (rType) { // What are we interpolating/rendering?
        case FLAT:
          display_buffer[x + y * SIZE_X] = poly_color;
          break;
        case COLORED: {
          float red = vi.r / vi.hw;   // divide all interpolated values by hw
          float green = vi.g / vi.hw; // divide all interpolated values by hw
          float blue = vi.b / vi.hw;  // divide all interpolated values by hw
          display_buffer[x + y * SIZE_X] =
              RGB_MAKE((int)(red * 255.0f), (int)(green * 255.0f), (int)(blue * 255.0f));
        } break;
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
  float BC[POLY_MAX_VERTICES] = {}; // boundary tests against y scanline
  int line[POLY_MAX_VERTICES] = {}; // which lines cross y scanline
  int lines = 0;
  float a1, a2;  // alphas for each crossing line (there can only be 2)
  Vertex sv, ev; // Start and end scanline vertices

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

  // Build RenderPack endpoints from sv and ev
  renderlayout::RenderPack<2> scan; // 0 = left, 1 = right
  scan.x(0) = sv.x;
  scan.y(0) = sv.y;
  scan.ez(0) = sv.ez;
  scan.hw(0) = sv.hw;
  scan.u(0) = sv.u;
  scan.v(0) = sv.v;
  scan.r(0) = sv.r;
  scan.g(0) = sv.g;
  scan.b(0) = sv.b;
  scan.x(1) = ev.x;
  scan.y(1) = ev.y;
  scan.ez(1) = ev.ez;
  scan.hw(1) = ev.hw;
  scan.u(1) = ev.u;
  scan.v(1) = ev.v;
  scan.r(1) = ev.r;
  scan.g(1) = ev.g;
  scan.b(1) = ev.b;

  float sx = scan.x(0);
  float ex = scan.x(1);
  if (sx < 0)
    sx = 0;
  if (ex < 0)
    return;
  if (ex >= SIZE_X)
    ex = SIZE_X - 1;
  if (floorf(sx) == ceilf(ex))
    return;

  const float invDen = 1.0f / (scan.x(0) - scan.x(1));
  uint16_t poly_color =
      RGB_MAKE((uint8_t)(r * 255.0f), (uint8_t)(g * 255.0f), (uint8_t)(b * 255.0f));
  for (int x = static_cast<int>(ceilf(sx)); x <= static_cast<int>(floorf(ex)); x++) {
    const float t = (scan.x(0) - static_cast<float>(x)) * invDen;
    const float pix_ez = scan.ez(0) + (scan.ez(1) - scan.ez(0)) * t;
    const float pix_hw = scan.hw(0) + (scan.hw(1) - scan.hw(0)) * t;
    const float zval = pix_ez / pix_hw;
    if (zval < z_buffer[x + y * SIZE_X]) {
      z_buffer[x + y * SIZE_X] = zval;
      switch (rType) { // What are we interpolating/rendering?
      case FLAT:
        display_buffer[x + y * SIZE_X] = poly_color;
        break;
      case COLORED: {
        const float pix_r = (scan.r(0) + (scan.r(1) - scan.r(0)) * t) / pix_hw;
        const float pix_g = (scan.g(0) + (scan.g(1) - scan.g(0)) * t) / pix_hw;
        const float pix_b = (scan.b(0) + (scan.b(1) - scan.b(0)) * t) / pix_hw;
        display_buffer[x + y * SIZE_X] = RGB_MAKE(
            (uint8_t)(pix_r * 255.0f), (uint8_t)(pix_g * 255.0f), (uint8_t)(pix_b * 255.0f));
        break;
      }
      case SMOOTH:
        break;
      case TEXTURED: {
        float pix_u = (scan.u(0) + (scan.u(1) - scan.u(0)) * t) / pix_hw;
        float pix_v = (scan.v(0) + (scan.v(1) - scan.v(0)) * t) / pix_hw;
        int u_lt0 = pix_u<0.0f, u_gt1 = pix_u> 1.0f;
        int u_in = !(u_lt0 | u_gt1);
        int v_lt0 = pix_v<0.0f, v_gt1 = pix_v> 1.0f;
        int v_in = !(v_lt0 | v_gt1);
        pix_u = u_in * pix_u + u_gt1 * 1.0f + u_lt0 * 0.0f;
        pix_v = v_in * pix_v + v_gt1 * 1.0f + v_lt0 * 0.0f;
        int32_t ufx = (int32_t)(pix_u * (float)(texwidth - 1) * 65536.0f);
        int32_t vfx = (int32_t)(pix_v * (float)(texheight - 1) * 65536.0f);
        int tx = ufx >> 16;
        int ty = vfx >> 16;
        display_buffer[x + y * SIZE_X] = texture[tx + ty * texwidth];
        break;
      }
      case TEXTURED_SMOOTH:
        break;
      default:
        break;
      }
    }
  }
}

void Poly::RasterizeFast(const int y) {
  if (y > ySorted[0].y || (y < ySorted[2].y && numVertices == 3) || y <= ySorted[3].y)
    return;

  float dyl, dyr;
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

  // Bridge to zero-cost layout pack for left/right scanline endpoints
  renderlayout::RenderPack<2> scan; // 0 = left, 1 = right
  // Required lanes for all modes
  scan.x(0) = increments[depthindex][0][0] * dyl + v[leftindex][0];
  scan.x(1) = increments[depthindex][1][0] * dyr + v[rightindex][0];
  scan.y(0) = increments[depthindex][0][1] * dyl + v[leftindex][1];
  scan.y(1) = increments[depthindex][1][1] * dyr + v[rightindex][1];
  scan.ez(0) = increments[depthindex][0][6] * dyl + v[leftindex][6];
  scan.ez(1) = increments[depthindex][1][6] * dyr + v[rightindex][6];
  scan.hw(0) = increments[depthindex][0][15] * dyl + v[leftindex][15];
  scan.hw(1) = increments[depthindex][1][15] * dyr + v[rightindex][15];
  // Texture and color lanes (filled unconditionally; compiler can DCE when unused)
  scan.u(0) = increments[depthindex][0][7] * dyl + v[leftindex][7];
  scan.u(1) = increments[depthindex][1][7] * dyr + v[rightindex][7];
  scan.v(0) = increments[depthindex][0][8] * dyl + v[leftindex][8];
  scan.v(1) = increments[depthindex][1][8] * dyr + v[rightindex][8];
  scan.r(0) = increments[depthindex][0][9] * dyl + v[leftindex][9];
  scan.r(1) = increments[depthindex][1][9] * dyr + v[rightindex][9];
  scan.g(0) = increments[depthindex][0][10] * dyl + v[leftindex][10];
  scan.g(1) = increments[depthindex][1][10] * dyr + v[rightindex][10];
  scan.b(0) = increments[depthindex][0][11] * dyl + v[leftindex][11];
  scan.b(1) = increments[depthindex][1][11] * dyr + v[rightindex][11];

  float sx = scan.x(0), ex = scan.x(1);
  if (ex < 0 || sx >= SIZE_X)
    return;
  if (sx < 0)
    sx = 0;
  if (ex >= SIZE_X)
    ex = SIZE_X - 1;
  int xStart = (int)std::ceil(sx);
  int xEnd = (int)std::floor(ex);
  if (xEnd < xStart)
    return;

  const float invDen = 1.0f / (scan.x(0) - scan.x(1));
  float t = (scan.x(0) - (float)xStart) * invDen;
  const float dt = -invDen;

  // Row pointers
  float *zrow = z_buffer + y * SIZE_X;
  uint16_t *drow = display_buffer + y * SIZE_X;

  // Interpolants at xStart and their per-pixel steps
  float ez = scan.ez(0) + (scan.ez(1) - scan.ez(0)) * t;
  float hw = scan.hw(0) + (scan.hw(1) - scan.hw(0)) * t;
  const float dez = (scan.ez(1) - scan.ez(0)) * dt;
  const float dhw = (scan.hw(1) - scan.hw(0)) * dt;

  switch (rType) {
  default:
  case FLAT: {
    const uint16_t flatColor =
        RGB_MAKE((uint8_t)(r * 255.0f), (uint8_t)(g * 255.0f), (uint8_t)(b * 255.0f));
    for (int x = xStart; x <= xEnd; ++x) {
      const float zval = ez / hw;
      if (zval < zrow[x]) {
        zrow[x] = zval;
        drow[x] = flatColor;
      }
      ez += dez;
      hw += dhw;
    }
    break;
  }
  case COLORED: {
    float cr = scan.r(0) + (scan.r(1) - scan.r(0)) * t;
    float cg = scan.g(0) + (scan.g(1) - scan.g(0)) * t;
    float cb = scan.b(0) + (scan.b(1) - scan.b(0)) * t;
    const float dcr = (scan.r(1) - scan.r(0)) * dt;
    const float dcg = (scan.g(1) - scan.g(0)) * dt;
    const float dcb = (scan.b(1) - scan.b(0)) * dt;
    for (int x = xStart; x <= xEnd; ++x) {
      const float inv_hw = 1.0f / hw;
      const float zval = ez * inv_hw;
      if (zval < zrow[x]) {
        zrow[x] = zval;
        drow[x] = RGB_MAKE((uint8_t)((cr * inv_hw) * 255.0f), (uint8_t)((cg * inv_hw) * 255.0f),
                           (uint8_t)((cb * inv_hw) * 255.0f));
      }
      ez += dez;
      hw += dhw;
      cr += dcr;
      cg += dcg;
      cb += dcb;
    }
    break;
  }
  case TEXTURED: {
    float uu = scan.u(0) + (scan.u(1) - scan.u(0)) * t;
    float vv = scan.v(0) + (scan.v(1) - scan.v(0)) * t;
    const float du = (scan.u(1) - scan.u(0)) * dt;
    const float dv = (scan.v(1) - scan.v(0)) * dt;
    const float uScale = (float)(texwidth - 1) * 65536.0f;
    const float vScale = (float)(texheight - 1) * 65536.0f;
    for (int x = xStart; x <= xEnd; ++x) {
      const float inv_hw = 1.0f / hw;
      const float zval = ez * inv_hw;
      if (zval < zrow[x]) {
        zrow[x] = zval;
        float pu = uu * inv_hw;
        float pv = vv * inv_hw;
        int u_lt0 = pu<0.0f, u_gt1 = pu> 1.0f;
        int u_in = !(u_lt0 | u_gt1);
        int v_lt0 = pv<0.0f, v_gt1 = pv> 1.0f;
        int v_in = !(v_lt0 | v_gt1);
        pu = u_in * pu + u_gt1 * 1.0f + u_lt0 * 0.0f;
        pv = v_in * pv + v_gt1 * 1.0f + v_lt0 * 0.0f;
        int32_t ufx = (int32_t)(pu * uScale);
        int32_t vfx = (int32_t)(pv * vScale);
        int tx = ufx >> 16;
        int ty = vfx >> 16;
        drow[x] = texture[tx + ty * texwidth];
      }
      ez += dez;
      hw += dhw;
      uu += du;
      vv += dv;
    }
    break;
  }
  case SMOOTH:
  case TEXTURED_SMOOTH:
    // Unchanged
    break;
  }
}

void Poly::RasterizeFull() {
  // Determine y-extent from sorted vertices
  int yTop = static_cast<int>(std::floor(ySorted[0].y));
  int yBot = 0;
  if (numVertices == 3) {
    yBot = static_cast<int>(std::ceil(ySorted[2].y));
  } else {
    yBot = static_cast<int>(std::ceil(ySorted[3].y));
  }
  if (yTop >= SIZE_Y)
    yTop = SIZE_Y - 1;
  if (yBot < 0)
    yBot = 0;
  for (int y = yTop; y >= yBot; --y) {
    RasterizeFast(y);
  }
}

// Helper Functions
float Poly::MinX() {
  float min = v[0].x;
  for (int i = 0; i < numVertices; i++) {
    if (min > v[i].x)
      min = v[i].x;
  }
  return min;
}

float Poly::MinY() {
  float min = v[0].y;
  for (int i = 0; i < numVertices; i++) {
    if (min > v[i].y)
      min = v[i].y;
  }
  return min;
}

float Poly::MinZ() {
  float min = v[0].z;
  for (int i = 0; i < numVertices; i++) {
    if (min > v[i].z)
      min = v[i].z;
  }
  return min;
}

float Poly::MaxX() {
  float max = v[0].x;
  for (int i = 0; i < numVertices; i++) {
    if (max < v[i].x)
      max = v[i].x;
  }
  return max;
}

float Poly::MaxY() {
  float max = v[0].y;
  for (int i = 0; i < numVertices; i++) {
    if (max < v[i].y)
      max = v[i].y;
  }
  return max;
}

float Poly::MaxZ() {
  float max = v[0].z;
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
