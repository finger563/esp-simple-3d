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
  std::stringstream ss(tok);
  ss >> v;
  if (ss.peek() == '/') {
    char c1 = '\0';
    ss.get(c1);
    if (ss.peek() != '/')
      ss >> vt;
  }
  if (ss.peek() == '/') {
    char c2 = '\0';
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
  pos.reserve(4096);
  std::vector<Vec3> nor;
  nor.reserve(4096);
  std::vector<Vec2> tex;
  tex.reserve(4096);

  // Accumulate one mesh per material group
  std::vector<Vertex> vertices;
  std::vector<uint32_t> indices;
  vertices.reserve(8192);
  indices.reserve(16384);

  // Vertex welding: (v, vt, vn) -> index
  auto key_for = [](int v, int vt, int vn) -> uint64_t {
    // Pack into 64 bits (21 bits each) with -1 mapped to 0, others plus 1
    uint64_t kv = (uint64_t)(v + 1) & 0x1FFFFF;
    uint64_t kt = (uint64_t)(vt + 1) & 0x1FFFFF;
    uint64_t kn = (uint64_t)(vn + 1) & 0x1FFFFF;
    return (kv << 42) | (kt << 21) | kn;
  };
  std::unordered_map<uint64_t, uint32_t> weld;
  weld.reserve(8192);

  RenderType curRT = COLORED;
  const unsigned short *curTex = nullptr;
  int curTW = 0, curTH = 0;
  float curR = 1.0f, curG = 1.0f, curB = 1.0f;

  auto flush_mesh = [&]() {
    if (!vertices.empty() && !indices.empty()) {
      outObject.AddMesh(vertices, indices, curRT, curTex, curTW, curTH, curR, curG, curB);
      vertices.clear();
      indices.clear();
      weld.clear();
    }
  };

  string line;
  std::vector<FaceIdx> face;
  face.reserve(16);

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
      flush_mesh();
      ss >> activeMtl;
      // Reset material defaults
      curRT = COLORED;
      curTex = nullptr;
      curTW = curTH = 0;
      curR = curG = curB = 1.0f;
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
                curRT = TEXTURED;
                curTex = texPtr;
                curTW = tw;
                curTH = th;
              }
            }
          } else {
            curRT = COLORED;
            curR = m.kd[0];
            curG = m.kd[1];
            curB = m.kd[2];
            if (outMat) {
              outMat->kd[0] = curR;
              outMat->kd[1] = curG;
              outMat->kd[2] = curB;
            }
          }
        }
      }
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

      auto get_or_add = [&](const FaceIdx &idx) -> uint32_t {
        uint64_t key = key_for(idx.v, idx.vt, idx.vn);
        auto it = weld.find(key);
        if (it != weld.end())
          return it->second;

        // Build vertex
        Vertex vv;
        // position
        const Vec3 &p = pos[(size_t)idx.v];
        vv = Vertex(p.x, p.y, p.z, 1.0f);
        // uv (flip V: OBJ vt usually uses bottom-left origin; our textures are top-left)
        if (idx.vt >= 0 && (size_t)idx.vt < tex.size()) {
          float u = tex[(size_t)idx.vt].u;
          float v = tex[(size_t)idx.vt].v;
          vv.u = u;
          vv.v = 1.0f - v; // flip V
        }
        // normal
        if (idx.vn >= 0 && (size_t)idx.vn < nor.size()) {
          vv.nx = nor[(size_t)idx.vn].x;
          vv.ny = nor[(size_t)idx.vn].y;
          vv.nz = nor[(size_t)idx.vn].z;
        }
        uint32_t newIndex = static_cast<uint32_t>(vertices.size());
        vertices.push_back(vv);
        weld.emplace(key, newIndex);
        return newIndex;
      };

      // Triangulate fan-wise: (0, i-1, i)
      for (size_t i = 2; i < face.size(); ++i) {
        uint32_t i0 = get_or_add(face[0]);
        uint32_t i1 = get_or_add(face[i - 1]);
        uint32_t i2 = get_or_add(face[i]);
        indices.push_back(i0);
        indices.push_back(i1);
        indices.push_back(i2);
      }
    }
  }

  flush_mesh();
  return true;
}

// GLB loader is implemented in asset_loader_glb.cpp

} // namespace asset
