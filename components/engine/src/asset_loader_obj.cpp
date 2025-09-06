#include "asset_loader.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

using std::string;
namespace fs = std::filesystem;

namespace {
struct Vec3 {
  float x{}, y{}, z{};
};
struct Vec2 {
  float u{}, v{};
};

static inline void trim(string &s) {
  auto notspace = [](int ch) { return !std::isspace(ch); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
  s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
}

struct FaceIdx {
  int v{-1}, vt{-1}, vn{-1};
};

static bool parseFaceTriplet(const string &tok, FaceIdx &out) {
  // formats: v, v/vt, v//vn, v/vt/vn (1-based indices)
  int v = 0, vt = 0, vn = 0;
  char c1 = '\0', c2 = '\0';
  std::stringstream ss(tok);
  ss >> v;
  if (ss.peek() == '/') {
    ss.get(c1);
    if (ss.peek() != '/')
      ss >> vt;
  }
  if (ss.peek() == '/') {
    ss.get(c2);
    ss >> vn;
  }
  if (ss.fail())
    return false;
  out.v = v - 1;
  out.vt = (vt ? vt - 1 : -1);
  out.vn = (vn ? vn - 1 : -1);
  return true;
}

struct MtlInfo {
  string map_kd;
  float kd[3]{1.0f, 1.0f, 1.0f};
};

static void loadMTL(const fs::path &mtlPath, std::unordered_map<string, MtlInfo> &out) {
  std::ifstream f(mtlPath);
  if (!f)
    return;
  string line, cur;
  MtlInfo m;
  while (std::getline(f, line)) {
    trim(line);
    if (line.empty() || line[0] == '#')
      continue;
    std::stringstream ss(line);
    string key;
    ss >> key;
    if (key == "newmtl") {
      if (!cur.empty())
        out[cur] = m;
      m = MtlInfo{};
      ss >> cur;
    } else if (key == "Kd") {
      ss >> m.kd[0] >> m.kd[1] >> m.kd[2];
    } else if (key == "map_Kd") {
      ss >> m.map_kd;
    }
  }
  if (!cur.empty())
    out[cur] = m;
}

} // namespace

namespace asset {

bool LoadOBJ(const string &objPath, Object &outObject, MaterialInfo *outMat,
             TextureDecodeFn decoder) {
  std::ifstream f(objPath);
  if (!f)
    return false;

  fs::path base = fs::path(objPath).parent_path();
  std::unordered_map<string, MtlInfo> mtls;
  string activeMtl;

  std::vector<Vec3> pos;
  pos.reserve(1024);
  std::vector<Vec3> nor;
  nor.reserve(1024);
  std::vector<Vec2> tex;
  tex.reserve(1024);

  // We build polys per face; for quads/ngons we triangulate fan-wise
  std::vector<FaceIdx> face;
  face.reserve(8);

  string line;
  while (std::getline(f, line)) {
    trim(line);
    if (line.empty() || line[0] == '#')
      continue;
    std::stringstream ss(line);
    string key;
    ss >> key;

    if (key == "mtllib") {
      string mtl;
      ss >> mtl;
      loadMTL(base / mtl, mtls);
    } else if (key == "usemtl") {
      ss >> activeMtl;
    } else if (key == "v") {
      Vec3 v{};
      ss >> v.x >> v.y >> v.z;
      pos.push_back(v);
    } else if (key == "vn") {
      Vec3 n{};
      ss >> n.x >> n.y >> n.z;
      nor.push_back(n);
    } else if (key == "vt") {
      Vec2 t{};
      ss >> t.u >> t.v;
      tex.push_back(t);
    } else if (key == "f") {
      face.clear();
      string tok;
      while (ss >> tok) {
        FaceIdx idx{};
        if (parseFaceTriplet(tok, idx))
          face.push_back(idx);
      }
      if (face.size() < 3)
        continue;
      // Triangulate face[0], face[i-1], face[i]
      for (size_t i = 2; i < face.size(); ++i) {
        FaceIdx idx0 = face[0], idx1 = face[i - 1], idx2 = face[i];
        auto mkVertex = [&](const FaceIdx &idx) -> Vertex {
          const Vec3 &p = pos[idx.v];
          Vertex v(p.x, p.y, p.z, 1.0f);
          if (idx.vt >= 0 && (size_t)idx.vt < tex.size()) {
            v.u = tex[idx.vt].u;
            v.v = tex[idx.vt].v;
          }
          if (idx.vn >= 0 && (size_t)idx.vn < nor.size()) {
            v.nx = nor[idx.vn].x;
            v.ny = nor[idx.vn].y;
            v.nz = nor[idx.vn].z;
          }
          return v;
        };
        Vertex v0 = mkVertex(idx0);
        Vertex v1 = mkVertex(idx1);
        Vertex v2 = mkVertex(idx2);
        Poly poly(v0, v1, v2, Vertex(), 3, Vector3D(0, 0, 1), TEXTURED);
        // Assign material data and optionally decode texture
        if (!activeMtl.empty()) {
          auto it = mtls.find(activeMtl);
          if (it != mtls.end()) {
            const auto &m = it->second;
            if (!m.map_kd.empty()) {
              fs::path tpath = base / m.map_kd;
              if (outMat) {
                outMat->hasTexture = true;
                outMat->texturePath = tpath.string();
              }
              if (decoder) {
                uint16_t *texPtr = nullptr;
                int tw = 0, th = 0;
                if (decoder(tpath.string(), texPtr, tw, th) && texPtr && tw > 0 && th > 0) {
                  poly.SetTexture(texPtr, tw, th);
                }
              }
            } else {
              poly.SetRenderType(COLORED);
              poly.SetColor(m.kd[0], m.kd[1], m.kd[2]);
              if (outMat) {
                outMat->kd[0] = m.kd[0];
                outMat->kd[1] = m.kd[1];
                outMat->kd[2] = m.kd[2];
              }
            }
          }
        }
        outObject.add(poly);
      }
    }
  }
  return true;
}

// GLB loader is implemented in asset_loader_glb.cpp

} // namespace asset
