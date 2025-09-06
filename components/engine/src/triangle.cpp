#include "triangle.hpp"

#include "main.hpp"
#include "render_layout.hpp"
#include <cmath>
#include <cstdint>

extern float *z_buffer;
extern uint16_t *display_buffer;

// Used for rotating the polygons in an object
void Triangle::Rotate(const Matrix &m) {
  a = m * a;
  b = m * b;
  c = m * c;
  {
    Vector3D n = Vector3D(normal.x, normal.y, normal.z, 0);
    normal = m * n;
  }
}

void Triangle::Translate(const Vector3D &v) {
  a = a + v;
  b = b + v;
  c = c + v;
}

void Triangle::Translate(const float x, const float y, const float z) {
  Vector3D v = Vector3D(x, y, z);
  a = a + v;
  b = b + v;
  c = c + v;
}

// Pass in the View to screen space transform
void Triangle::DrawWireframe(const Matrix &m) {
  if (normal.z > 0) {
    return;
  }
  // DRAW_Line ( s1.x, s1.y, s2.x, s2.y, color );
  // DRAW_Line ( s2.x, s2.y, s3.x, s3.y, color );
  // DRAW_Line ( s3.x, s3.y, s1.x, s1.y, color );
}

// Pass in the view to screen space transform
void Triangle::TransformToScreen(const Matrix &m) {
  visible = false;
  Vector3D eye = Vector3D(0, 0, -1);
  Vector3D cull = eye - a;
  float test = cull * normal;
  if (test < 0) {
    return; // Triangle isn't renderable, don't need to calculate anything else
  }
  visible = true;

  s1 = a;
  s2 = b;
  s3 = c;

  if (s1.z > 0) {
    s1.x = (s1.x) / (s1.z + 1); // d should be contained in m
    s1.y = (s1.y) / (s1.z + 1);
  } else if (s1.z == 0) { // screen-space coords are the same as world if z=0
  } else {
    // s1.x = (s1.x)/s1.z;
    // s1.y = (s1.y)/s1.z;
  }

  if (s2.z > 0) {
    s2.x = (s2.x) / (s2.z + 1);
    s2.y = (s2.y) / (s2.z + 1);
  } else if (s2.z == 0) {
  } else {
    // s2.x = (s2.x)/s2.z;
    // s2.y = (s2.y)/s2.z;
  }

  if (s3.z > 0) {
    s3.x = (s3.x) / (s3.z + 1);
    s3.y = (s3.y) / (s3.z + 1);
  } else if (s3.z == 0) {
  } else {
    // s3.x = (s3.x)/s3.z;
    // s3.y = (s3.y)/s3.z;
  }

  // sort vertices and associated texture coords by  Screen space y
  Point3D t1 = s1, t2 = s2, t3 = s3;
  Point2D tex1 = u, tex2 = v, tex3 = w;
  Point3D tmp;
  Point2D tex;
  if (t1.y < s2.y) {
    t1 = s2;
    t2 = s1;
    tex1 = v;
    tex2 = u;
  }
  if (t1.y < s3.y) {
    t3 = t1;
    t1 = s3;
    tex3 = tex1;
    tex1 = w;
  }
  if (t2.y < t3.y) {
    tmp = t2;
    t2 = t3;
    t3 = tmp;
    tex = tex2;
    tex2 = tex3;
    tex3 = tex;
  }
  s1 = t1;
  s2 = t2;
  s3 = t3;
  u1 = tex1;
  u2 = tex2;
  u3 = tex3; // Sort the texture points along with polygon points

  s1 = m * s1; // PERFORM THE TRANSFORMATION
  s2 = m * s2;
  s3 = m * s3;

  // Screen-space Slopes
  m12 = (s2.x - s1.x) / (s2.y - s1.y); // inverse slope between s1 & s2
  m13 = (s3.x - s1.x) / (s3.y - s1.y); // inverse slope between s1 & s3
  m23 = (s3.x - s2.x) / (s3.y - s2.y); // inverse slope between s2 & s3

  float sy1 = s1.y, sx1 = s1.x, sy2 = s2.y, sx2 = s2.x, sy3 = s3.y, sx3 = s3.x;

  if (s1.z > 0) {
    sy1 = s1.y * s1.z;
    sx1 = s1.x * s1.z;
  }
  if (s2.z > 0) {
    sy2 = s2.y * s2.z;
    sx2 = s2.x * s2.z;
  }
  if (s3.z > 0) {
    sy3 = s3.y * s3.z;
    sx3 = s3.x * s3.z;
  }

  s4.y = s2.y;
  s4.x = s1.x + (s2.y - s1.y) * m13;
  s4.z = 1 / (-(sy3 - sy1) + (s3.z - s1.z) * (s1.y + (s2.y - s1.y))) *
         ((s3.z - s1.z) * sy1 - (sy3 - sy1) * s1.z);

  float sy4 = s4.y;
  if (s4.z > 0) {
    sy4 = s4.y * s4.z;
  }

  dz12 = (s2.z - s1.z) / ((s2.z - s1.z) * sy1 - (sy2 - sy1) * s1.z); // slopes for z-buffering
  dz13 = (s3.z - s1.z) / ((s3.z - s1.z) * sy1 - (sy3 - sy1) * s1.z);
  dz14 = (s4.z - s1.z) / ((s4.z - s1.z) * sy1 - (sy4 - sy1) * s1.z);
  dz23 = (s3.z - s2.z) / ((s3.z - s2.z) * sy2 - (sy3 - sy2) * s2.z);
  dz43 = (s3.z - s4.z) / ((s3.z - s4.z) * sy4 - (sy3 - sy4) * s4.z);

  u4.x = u1.x + (u3.x - u1.x) * (sy4 - sy1) / (sy3 - sy1); // Intermediate texture coords
  u4.y = u1.y + (u3.y - u1.y) * (sy4 - sy1) / (sy3 - sy1);

  dtx13 = (u3.x - u1.x) / (sy3 - sy1); // slopes for texture coordinates
  dty13 = (u3.y - u1.y) / (sy3 - sy1);

  dtx12 = (u2.x - u1.x) / (sy2 - sy1);
  dty12 = (u2.y - u1.y) / (sy2 - sy1);

  dtx23 = (u3.x - u2.x) / (sy3 - sy2);
  dty23 = (u3.y - u2.y) / (sy3 - sy2);

  dtx14 = (u4.x - u1.x) / (sy4 - sy1);
  dty14 = (u4.y - u1.y) / (sy4 - sy1);

  dtx43 = (u3.x - u4.x) / (sy3 - sy4);
  dty43 = (u3.y - u4.y) / (sy3 - sy4);

  if (std::fabs(s1.y - s2.y) < EPSILON) { // Flat top
    if (s1.x < s2.x) {
      type = FLAT_TOP_RIGHT;
      sx = s1.x;
      ex = s2.x;
      sz = 1 / s1.z;
      ez = 1 / s2.z;
      stu = u1.x;
      stv = u1.y;
      etu = u2.x;
      etv = u2.y;
    } else {
      sx = s2.x;
      ex = s1.x;
      sz = 1 / s2.z;
      ez = 1 / s1.z;
      stu = u2.x;
      stv = u2.y;
      etu = u1.x;
      etv = u1.y;
      type = FLAT_TOP_LEFT;
    }
  } else {
    sx = s1.x;
    ex = s1.x;
    sz = 1 / s1.z;
    ez = 1 / s1.z;
    stu = u1.x;
    stv = u1.y;
    etu = u1.x;
    etv = u1.y;
    if (std::fabs(s2.y - s3.y) < EPSILON) { // Flat Bottom
      if (s3.x < s2.x) {
        type = FLAT_BOTTOM_RIGHT;
      } else {
        type = FLAT_BOTTOM_LEFT;
      }
    } else { // Normal triangle
      if (s2.x > s4.x) {
        type = NORMAL_RIGHT;
      } else {
        type = NORMAL_LEFT;
      }
    }
  }
}

void Triangle::DrawFilled(void) {
  if (normal.z > 0) {
    return;
  }
  float zi;
  switch (type) {
  case FLAT_TOP_RIGHT:
    for (int y = s1.y; y >= s3.y; y--) {
      dzx = (1 / ez - 1 / sz) / ((1 / ez - 1 / sz) * ex / ez - (ex / ez - sx / sz) / ez);
      zi = sz;
      float *zrow = z_buffer + y * SIZE_X;
      uint16_t *drow = display_buffer + y * SIZE_X;
      for (int x = sx; x <= ex; x++) {
        float z = 1.0f / zi;
        // if (zi > zrow[x]) {
        //   zrow[x] = zi;
        if (z < zrow[x]) {
          zrow[x] = z;
          drow[x] = color;
        }
        zi += dzx;
      }
      sz -= dz13;
      ez -= dz23;
      sx -= m13;
      ex -= m23;
    }
    break;
  case FLAT_TOP_LEFT:
    for (int y = s1.y; y >= s3.y; y--) {
      dzx = (1 / ez - 1 / sz) / ((1 / ez - 1 / sz) * ex / ez - (ex / ez - sx / sz) / ez);
      zi = sz;
      float *zrow = z_buffer + y * SIZE_X;
      uint16_t *drow = display_buffer + y * SIZE_X;
      for (int x = sx; x <= ex; x++) {
        float z = 1.0f / zi;
        // if (zi > zrow[x]) {
        //   zrow[x] = zi;
        if (z < zrow[x]) {
          zrow[x] = z;
          drow[x] = color;
        }
        zi += dzx;
      }
      sz -= dz23;
      ez -= dz13;
      sx -= m23;
      ex -= m13;
    }
    break;
  case FLAT_BOTTOM_RIGHT:
    for (int y = s1.y; y >= s3.y; y--) {
      dzx = (1 / ez - 1 / sz) / ((1 / ez - 1 / sz) * ex / ez - (ex / ez - sx / sz) / ez);
      zi = sz;
      float *zrow = z_buffer + y * SIZE_X;
      uint16_t *drow = display_buffer + y * SIZE_X;
      for (int x = sx; x <= ex; x++) {
        float z = 1.0f / zi;
        // if (zi > zrow[x]) {
        //   zrow[x] = zi;
        if (z < zrow[x]) {
          zrow[x] = z;
          drow[x] = color;
        }
        zi += dzx;
      }
      sz -= dz13;
      ez -= dz12;
      sx -= m13;
      ex -= m12;
    }
    break;
  case FLAT_BOTTOM_LEFT:
    for (int y = s1.y; y >= s3.y; y--) {
      dzx = (1 / ez - 1 / sz) / ((1 / ez - 1 / sz) * ex / ez - (ex / ez - sx / sz) / ez);
      zi = sz;
      float *zrow = z_buffer + y * SIZE_X;
      uint16_t *drow = display_buffer + y * SIZE_X;
      for (int x = sx; x <= ex; x++) {
        // if (zi > zrow[x]) {
        //   zrow[x] = zi;
        float z = 1.0f / zi;
        if (z < zrow[x]) {
          zrow[x] = z;
          drow[x] = color;
        }
        zi += dzx;
      }
      sz -= dz12;
      ez -= dz13;
      sx -= m12;
      ex -= m13;
    }
    break;
  case NORMAL_RIGHT:
    for (int y = s1.y; y >= s2.y; y--) {
      dzx = (1 / ez - 1 / sz) / ((1 / ez - 1 / sz) * ex / ez - (ex / ez - sx / sz) / ez);
      zi = sz;
      float *zrow = z_buffer + y * SIZE_X;
      uint16_t *drow = display_buffer + y * SIZE_X;
      for (int x = sx; x <= ex; x++) {
        // if (zi > zrow[x]) {
        //   zrow[x] = zi;
        float z = 1.0f / zi;
        if (z < zrow[x]) {
          zrow[x] = z;
          drow[x] = color;
        }
        zi += dzx;
      }
      sz -= dz13;
      ez -= dz12;
      sx -= m13;
      ex -= m12;
    }
    sx = s4.x;
    ex = s2.x;
    for (int y = s4.y; y >= s3.y; y--) {
      dzx = (1 / ez - 1 / sz) / ((1 / ez - 1 / sz) * ex / ez - (ex / ez - sx / sz) / ez);
      zi = sz;
      float *zrow = z_buffer + y * SIZE_X;
      uint16_t *drow = display_buffer + y * SIZE_X;
      for (int x = sx; x <= ex; x++) {
        // if (zi > zrow[x]) {
        //   zrow[x] = zi;
        float z = 1.0f / zi;
        if (z < zrow[x]) {
          zrow[x] = z;
          drow[x] = color;
        }
        zi += dzx;
      }
      sz -= dz13;
      ez -= dz23;
      sx -= m13;
      ex -= m23;
    }
    break;
  case NORMAL_LEFT:
    for (int y = s1.y; y >= s2.y; y--) {
      dzx = (1 / ez - 1 / sz) / ((1 / ez - 1 / sz) * ex / ez - (ex / ez - sx / sz) / ez);
      zi = sz;
      float *zrow = z_buffer + y * SIZE_X;
      uint16_t *drow = display_buffer + y * SIZE_X;
      for (int x = sx; x <= ex; x++) {
        // if (zi > zrow[x]) {
        //   zrow[x] = zi;
        float z = 1.0f / zi;
        if (z < zrow[x]) {
          zrow[x] = z;
          drow[x] = color;
        }
        zi += dzx;
      }
      sz -= dz12;
      ez -= dz13;
      sx -= m12;
      ex -= m13;
    }
    sx = s2.x;
    ex = s4.x;
    for (int y = s4.y; y >= s3.y; y--) {
      dzx = (1 / ez - 1 / sz) / ((1 / ez - 1 / sz) * ex / ez - (ex / ez - sx / sz) / ez);
      zi = sz;
      float *zrow = z_buffer + y * SIZE_X;
      uint16_t *drow = display_buffer + y * SIZE_X;
      for (int x = sx; x <= ex; x++) {
        // if (zi > zrow[x]) {
        //   zrow[x] = zi;
        float z = 1.0f / zi;
        if (z < zrow[x]) {
          zrow[x] = z;
          drow[x] = color;
        }
        zi += dzx;
      }
      sz -= dz23;
      ez -= dz13;
      sx -= m23;
      ex -= m13;
    }
    break;
  default:
    break;
  }
}

// Pass in the pixel-space y value
void Triangle::DrawFilledZbuffer(const int y) {
  if (normal.z > 0 || s1.y < y || s3.y > y)
    return;

  float dy = y - s1.y; // dy is negative!
  float zi;

  switch (type) {
  case FLAT_TOP_RIGHT:
    sz = 1 / s1.z + (dy)*dz13;
    ez = 1 / s2.z + (dy)*dz23;

    sx = s1.x + (dy)*m13;
    ex = s2.x + (dy)*m23;
    break;
  case FLAT_TOP_LEFT:
    sz = 1 / s2.z + (dy)*dz23;
    ez = 1 / s1.z + (dy)*dz13;

    sx = s2.x + (dy)*m23;
    ex = s1.x + (dy)*m13;
    break;
  case FLAT_BOTTOM_RIGHT:
    sz = 1 / s1.z + (dy)*dz13;
    ez = 1 / s1.z + (dy)*dz12;

    sx = s1.x + (dy)*m13;
    ex = s1.x + (dy)*m12;
    break;
  case FLAT_BOTTOM_LEFT:
    sz = 1 / s1.z + (dy)*dz12;
    ez = 1 / s1.z + (dy)*dz13;

    sx = s1.x + (dy)*m12;
    ex = s1.x + (dy)*m13;
    break;
  case NORMAL_RIGHT:
    if (s4.y <= y) {
      sz = 1 / s1.z + (dy)*dz14;
      ez = 1 / s1.z + (dy)*dz12;

      sx = s1.x + (dy)*m13;
      ex = s1.x + (dy)*m12;
    } else {
      dy = y - s4.y;
      sz = 1 / s4.z + (dy)*dz43;
      ez = 1 / s2.z + (dy)*dz23;

      sx = s4.x + (dy)*m13;
      ex = s2.x + (dy)*m23;
    }
    break;
  case NORMAL_LEFT:
    if (s4.y <= y) {
      sz = 1 / s1.z + (dy)*dz12;
      ez = 1 / s1.z + (dy)*dz14;

      sx = s1.x + (dy)*m12;
      ex = s1.x + (dy)*m13;
    } else {
      dy = y - s2.y;
      sz = 1 / s2.z + (dy)*dz23;
      ez = 1 / s4.z + (dy)*dz43;

      sx = s2.x + (dy)*m23;
      ex = s4.x + (dy)*m13;
    }
    break;
  default:
    break;
  }

  // Use RenderPack for per-pixel interpolation across the scanline
  renderlayout::RenderPack<2> scan;
  scan.x(0) = sx;
  scan.x(1) = ex;
  scan.ez(0) = sz;
  scan.ez(1) = ez;

  int startX = static_cast<int>(std::ceil(std::fmin(sx, ex)));
  int endX = static_cast<int>(std::floor(std::fmax(sx, ex)));
  if (endX < 0 || startX >= SIZE_X)
    return;
  if (startX < 0)
    startX = 0;
  if (endX >= SIZE_X)
    endX = SIZE_X - 1;
  const float invDen = 1.0f / (scan.x(0) - scan.x(1));

  for (int x = startX; x <= endX; ++x) {
    const float t = (scan.x(0) - static_cast<float>(x)) * invDen;
    zi = scan.ez(0) + (scan.ez(1) - scan.ez(0)) * t;
    // if (zi > z_buffer[x + y * SIZE_X]) {
    //   z_buffer[x + y * SIZE_X] = zi;
    float z = 1.0f / sz;
    if (z < z_buffer[x + y * SIZE_X]) {
      z_buffer[x + y * SIZE_X] = z;
      display_buffer[x + y * SIZE_X] = color;
    }
  }
}

// Pass in the pixel-space y value
void Triangle::DrawTexturedZbuffer(const int y) {
  if (s1.y < y || s3.y > y)
    return;

  float dy = y - s1.y; // dy is negative!

  switch (type) {
  case FLAT_TOP_RIGHT:
    sz = 1 / s1.z + (dy)*dz13;
    ez = 1 / s2.z + (dy)*dz23;

    sx = s1.x + (dy)*m13;
    ex = s2.x + (dy)*m23;

    stu = (y / sz - s1.y * s1.z) * dtx13 + u1.x;
    stv = (y / sz - s1.y * s1.z) * dty13 + u1.y;
    etu = (y / ez - s2.y * s2.z) * dtx23 + u2.x;
    etv = (y / ez - s2.y * s2.z) * dty23 + u2.y;
    break;
  case FLAT_TOP_LEFT:
    sz = 1 / s2.z + (dy)*dz23;
    ez = 1 / s1.z + (dy)*dz13;

    sx = s2.x + (dy)*m23;
    ex = s1.x + (dy)*m13;

    stu = (y / sz - s2.y * s2.z) * dtx23 + u2.x;
    stv = (y / sz - s2.y * s2.z) * dty23 + u2.y;
    etu = (y / ez - s1.y * s1.z) * dtx13 + u1.x;
    etv = (y / ez - s1.y * s1.z) * dty13 + u1.y;
    break;
  case FLAT_BOTTOM_RIGHT:
    sz = 1 / s1.z + (dy)*dz13;
    ez = 1 / s1.z + (dy)*dz12;

    sx = s1.x + (dy)*m13;
    ex = s1.x + (dy)*m12;

    stu = (y / sz - s1.y * s1.z) * dtx13 + u1.x;
    stv = (y / sz - s1.y * s1.z) * dty13 + u1.y;
    etu = (y / ez - s1.y * s1.z) * dtx12 + u1.x;
    etv = (y / ez - s1.y * s1.z) * dty12 + u1.y;
    break;
  case FLAT_BOTTOM_LEFT:
    sz = 1 / s1.z + (dy)*dz12;
    ez = 1 / s1.z + (dy)*dz13;

    sx = s1.x + (dy)*m12;
    ex = s1.x + (dy)*m13;

    stu = (y / sz - s1.y * s1.z) * dtx12 + u1.x;
    stv = (y / sz - s1.y * s1.z) * dty12 + u1.y;
    etu = (y / ez - s1.y * s1.z) * dtx13 + u1.x;
    etv = (y / ez - s1.y * s1.z) * dty13 + u1.y;
    break;
  case NORMAL_RIGHT:
    if (s2.y <= y) {
      sz = 1 / s1.z + (dy)*dz14;
      ez = 1 / s1.z + (dy)*dz12;

      sx = s1.x + (dy)*m13;
      ex = s1.x + (dy)*m12;

      stu = (y / sz - s1.y * s1.z) * dtx14 + u1.x;
      stv = (y / sz - s1.y * s1.z) * dty14 + u1.y;
      etu = (y / ez - s1.y * s1.z) * dtx12 + u1.x;
      etv = (y / ez - s1.y * s1.z) * dty12 + u1.y;
    } else {
      dy = y - s2.y;
      sz = 1 / s4.z + (dy)*dz43;
      ez = 1 / s2.z + (dy)*dz23;

      sx = s4.x + (dy)*m13;
      ex = s2.x + (dy)*m23;

      stu = (y / sz - s4.y * s4.z) * dtx43 + u4.x;
      stv = (y / sz - s4.y * s4.z) * dty43 + u4.y;
      etu = (y / ez - s2.y * s2.z) * dtx23 + u2.x;
      etv = (y / ez - s2.y * s2.z) * dty23 + u2.y;
    }
    break;
  case NORMAL_LEFT:
    if (s2.y <= y) {
      sz = 1 / s1.z + (dy)*dz12;
      ez = 1 / s1.z + (dy)*dz14;

      sx = s1.x + (dy)*m12;
      ex = s1.x + (dy)*m13;

      stu = (y / sz - s1.y * s1.z) * dtx12 + u1.x;
      stv = (y / sz - s1.y * s1.z) * dty12 + u1.y;
      etu = (y / ez - s1.y * s1.z) * dtx14 + u1.x;
      etv = (y / ez - s1.y * s1.z) * dty14 + u1.y;
    } else {
      dy = y - s2.y;
      sz = 1 / s2.z + (dy)*dz23;
      ez = 1 / s4.z + (dy)*dz43;

      sx = s2.x + (dy)*m23;
      ex = s4.x + (dy)*m13;

      stu = (y / sz - s2.y * s2.z) * dtx23 + u2.x;
      stv = (y / sz - s2.y * s2.z) * dty23 + u2.y;
      etu = (y / ez - s4.y * s4.z) * dtx43 + u4.x;
      etv = (y / ez - s4.y * s4.z) * dty43 + u4.y;
    }
    break;
  default:
    break;
  }

  // Use RenderPack for per-pixel interpolation across the scanline
  renderlayout::RenderPack<2> scan;
  scan.x(0) = sx;
  scan.x(1) = ex;
  scan.ez(0) = sz;
  scan.ez(1) = ez;
  scan.u(0) = stu;
  scan.u(1) = etu;
  scan.v(0) = stv;
  scan.v(1) = etv;

  if (ez < 0 && sz < 0) // if this line starts and ends behind viewport
    return;

  int startX = static_cast<int>(std::ceil(std::fmin(sx, ex)));
  int endX = static_cast<int>(std::floor(std::fmax(sx, ex)));
  if (endX < 0 || startX >= SIZE_X)
    return;
  if (startX < 0)
    startX = 0;
  if (endX >= SIZE_X)
    endX = SIZE_X - 1;
  if (!std::isfinite((double)sx) || !std::isfinite((double)ex))
    return;

  const float invDen = 1.0f / (scan.x(0) - scan.x(1));
  for (int x = startX; x <= endX; ++x) {
    const float t = (scan.x(0) - static_cast<float>(x)) * invDen;
    const float zinv = scan.ez(0) + (scan.ez(1) - scan.ez(0)) * t;
    const float z = 1.0f / zinv;
    // if (zinv > z_buffer[x + y * SIZE_X]) {
    //   z_buffer[x + y * SIZE_X] = zinv;
    if (z < z_buffer[x + y * SIZE_X]) {
      z_buffer[x + y * SIZE_X] = z;
      float tu = (scan.u(0) + (scan.u(1) - scan.u(0)) * t);
      float tv = (scan.v(0) + (scan.v(1) - scan.v(0)) * t);
      // Branchless clamp to [0, texwidth-1] and [0, texheight-1] using normalized u,v
      int u_lt0 = tu<0.0f, u_gt1 = tu> 1.0f;
      int u_in = !(u_lt0 | u_gt1);
      int v_lt0 = tv<0.0f, v_gt1 = tv> 1.0f;
      int v_in = !(v_lt0 | v_gt1);
      tu = u_in * tu + u_gt1 * 1.0f + u_lt0 * 0.0f;
      tv = v_in * tv + v_gt1 * 1.0f + v_lt0 * 0.0f;
      // Fixed-point 16.16 addressing
      int32_t ufx = (int32_t)(tu * (float)(texwidth - 1) * 65536.0f);
      int32_t vfx = (int32_t)(tv * (float)(texwidth - 1) * 65536.0f);
      int tx = ufx >> 16;
      int ty = vfx >> 16;
      display_buffer[x + y * SIZE_X] = texture[tx + ty * texwidth];
    }
  }
}

// Pass in the view to screen space transform
void Triangle::TransformToScreenHomogeneous(const Matrix &m) {
  visible = false;
  Vector3D eye = Vector3D(0, 0, -1);
  Vector3D cull = eye - a;
  float test = cull * normal;
  if (test < 0) {
    return; // Triangle isn't renderable, don't need to calculate anything else
  }
  visible = true;

  s1 = a;
  s2 = b;
  s3 = c;

  if (s1.z > 0) {
    s1.x = (s1.x) / (s1.z + 1); // d should be contained in m
    s1.y = (s1.y) / (s1.z + 1);
  }

  if (s2.z > 0) {
    s2.x = (s2.x) / (s2.z + 1);
    s2.y = (s2.y) / (s2.z + 1);
  }

  if (s3.z > 0) {
    s3.x = (s3.x) / (s3.z + 1);
    s3.y = (s3.y) / (s3.z + 1);
  }

  // sort vertices and associated texture coords by  Screen space y
  Point3D t1 = s1, t2 = s2, t3 = s3;
  Point2D tex1 = u, tex2 = v, tex3 = w;
  Point3D tmp;
  Point2D tex;
  if (t1.y < s2.y) {
    t1 = s2;
    t2 = s1;
    tex1 = v;
    tex2 = u;
  }
  if (t1.y < s3.y) {
    t3 = t1;
    t1 = s3;
    tex3 = tex1;
    tex1 = w;
  }
  if (t2.y < t3.y) {
    tmp = t2;
    t2 = t3;
    t3 = tmp;
    tex = tex2;
    tex2 = tex3;
    tex3 = tex;
  }
  s1 = t1;
  s2 = t2;
  s3 = t3;
  u1 = tex1;
  u2 = tex2;
  u3 = tex3; // Sort the texture points along with polygon points

  s1 = m * s1; // PERFORM THE TRANSFORMATION
  s2 = m * s2;
  s3 = m * s3;

  // Screen-space Slopes
  m12 = (s2.x - s1.x) / (s2.y - s1.y); // inverse slope between s1 & s2
  m13 = (s3.x - s1.x) / (s3.y - s1.y); // inverse slope between s1 & s3
  m23 = (s3.x - s2.x) / (s3.y - s2.y); // inverse slope between s2 & s3

  float sy1 = s1.y, sx1 = s1.x, sy2 = s2.y, sx2 = s2.x, sy3 = s3.y, sx3 = s3.x;

  s4.y = s2.y;
  s4.x = s1.x + (s2.y - s1.y) * m13;
  s4.z = 1 / (-(sy3 - sy1) + (s3.z - s1.z) * (s1.y + (s2.y - s1.y))) *
         ((s3.z - s1.z) * sy1 - (sy3 - sy1) * s1.z);

  if (std::fabs(s1.y - s2.y) < EPSILON) { // Flat top
    if (s1.x < s2.x) {
      type = FLAT_TOP_RIGHT;
      sx = s1.x;
      ex = s2.x;
      sz = 1 / s1.z;
      ez = 1 / s2.z;
      stu = u1.x;
      stv = u1.y;
      etu = u2.x;
      etv = u2.y;
    } else {
      sx = s2.x;
      ex = s1.x;
      sz = 1 / s2.z;
      ez = 1 / s1.z;
      stu = u2.x;
      stv = u2.y;
      etu = u1.x;
      etv = u1.y;
      type = FLAT_TOP_LEFT;
    }
  } else {
    sx = s1.x;
    ex = s1.x;
    sz = 1 / s1.z;
    ez = 1 / s1.z;
    stu = u1.x;
    stv = u1.y;
    etu = u1.x;
    etv = u1.y;
    if (std::fabs(s2.y - s3.y) < EPSILON) { // Flat Bottom
      if (s3.x < s2.x) {
        type = FLAT_BOTTOM_RIGHT;
      } else {
        type = FLAT_BOTTOM_LEFT;
      }
    } else { // Normal triangle
      if (s2.x > s4.x) {
        type = NORMAL_RIGHT;
      } else {
        type = NORMAL_LEFT;
      }
    }
  }
}

// Pass in the pixel-space y value
void Triangle::DrawTexturedZbufferHomogeneous(const int y) {
  if (s1.y < y || s3.y > y)
    return;

  float dy = y - s1.y; // dy is negative!

  switch (type) {
  case FLAT_TOP_RIGHT:
    break;
  case FLAT_TOP_LEFT:
    break;
  case FLAT_BOTTOM_RIGHT:
    break;
  case FLAT_BOTTOM_LEFT:
    break;
  case NORMAL_RIGHT:
    if (s2.y <= y) {
    } else {
      dy = y - s2.y;
    }
    break;
  case NORMAL_LEFT:
    if (s2.y <= y) {
    } else {
      dy = y - s2.y;
    }
    break;
  default:
    break;
  }

  float zi = sz;
  float z = 1.0f / zi;
  int x;
  for (x = sx; x <= ex; x++) {
    if (z < z_buffer[x + y * SIZE_X]) {
      z_buffer[x + y * SIZE_X] = z;
    }
    // if (zi > z_buffer[x + y * SIZE_X]) {
    //   z_buffer[x + y * SIZE_X] = zi;
    // }
    // zi += dzx;		// because dx > 0, we increment
  }
}
