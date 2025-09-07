// NOTE: You can switch to the cgltf-based implementation by defining ENGINE_USE_CGLTF
// and adding cgltf.h to your include path. The cJSON-based fallback remains as default.

#include "asset_loader.hpp"

#include "format.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

using std::string;
namespace fs = std::filesystem;

namespace asset {

#ifdef ENGINE_USE_CGLTF

// cgltf single-header implementation
#ifndef CGLTF_IMPLEMENTATION
#define CGLTF_IMPLEMENTATION
#endif
#include "cgltf.h"

static inline string dir_of(const string &path) {
  fs::path p(path);
  return p.parent_path().string();
}

bool LoadGLB(const string &glbPath, Object &outObject, MaterialInfo *outMat,
             TextureDecodeFn fileDecoder, TextureDecodeBytesFn bytesDecoder) {
  cgltf_options options{};
  cgltf_data *data = nullptr;
  cgltf_result res = cgltf_parse_file(&options, glbPath.c_str(), &data);
  if (res != cgltf_result_success || !data)
    return false;
  // Load buffers (BIN chunk for GLB; external buffers if any)
  res = cgltf_load_buffers(&options, data, glbPath.c_str());
  if (res != cgltf_result_success) {
    cgltf_free(data);
    return false;
  }

  if (data->meshes_count == 0) {
    cgltf_free(data);
    return false;
  }

  // First pass: compute global bounds of all positions
  float minx = std::numeric_limits<float>::infinity();
  float miny = std::numeric_limits<float>::infinity();
  float minz = std::numeric_limits<float>::infinity();
  float maxx = -std::numeric_limits<float>::infinity();
  float maxy = -std::numeric_limits<float>::infinity();
  float maxz = -std::numeric_limits<float>::infinity();
  for (cgltf_size meshIndex = 0; meshIndex < data->meshes_count; ++meshIndex) {
    const cgltf_mesh &mesh = data->meshes[meshIndex];
    for (cgltf_size primIndex = 0; primIndex < mesh.primitives_count; ++primIndex) {
      const cgltf_primitive &prim = mesh.primitives[primIndex];
      const cgltf_accessor *posAcc = nullptr;
      for (cgltf_size i = 0; i < prim.attributes_count; ++i) {
        const cgltf_attribute &attr = prim.attributes[i];
        if (attr.type == cgltf_attribute_type_position)
          posAcc = attr.data;
      }
      if (!posAcc)
        continue;
      for (cgltf_size i = 0; i < posAcc->count; ++i) {
        float tmp[3] = {0, 0, 0};
        cgltf_accessor_read_float(posAcc, i, tmp, 3);
        minx = std::min(minx, tmp[0]);
        miny = std::min(miny, tmp[1]);
        minz = std::min(minz, tmp[2]);
        maxx = std::max(maxx, tmp[0]);
        maxy = std::max(maxy, tmp[1]);
        maxz = std::max(maxz, tmp[2]);
      }
    }
  }
  const float cx = (minx + maxx) * 0.5f;
  const float cy = (miny + maxy) * 0.5f;
  const float cz = (minz + maxz) * 0.5f;

  size_t total_vertices = 0;
  size_t total_tris = 0;
  for (cgltf_size meshIndex = 0; meshIndex < data->meshes_count; ++meshIndex) {
    const cgltf_mesh &mesh = data->meshes[meshIndex];
    fmt::print("[GLB] Mesh {} has {} primitives\n", (int)meshIndex, (int)mesh.primitives_count);
    for (cgltf_size primIndex = 0; primIndex < mesh.primitives_count; ++primIndex) {
      const cgltf_primitive &prim = mesh.primitives[primIndex];
      fmt::print("[GLB]  Primitive {} type {}\n", (int)primIndex, (int)prim.type);

      const cgltf_accessor *posAcc = nullptr;
      const cgltf_accessor *norAcc = nullptr;
      const cgltf_accessor *uvAcc = nullptr;
      for (cgltf_size i = 0; i < prim.attributes_count; ++i) {
        const cgltf_attribute &attr = prim.attributes[i];
        switch (attr.type) {
        case cgltf_attribute_type_position:
          posAcc = attr.data;
          break;
        case cgltf_attribute_type_normal:
          norAcc = attr.data;
          break;
        case cgltf_attribute_type_texcoord:
          if (attr.index == 0)
            uvAcc = attr.data;
          break;
        default:
          break;
        }
      }
      if (!posAcc)
        continue;

      std::vector<Vertex> vertices;
      vertices.resize((size_t)posAcc->count);
      for (cgltf_size i = 0; i < posAcc->count; ++i) {
        float tmp[3] = {0, 0, 0};
        cgltf_accessor_read_float(posAcc, i, tmp, 3);
        vertices[i] = Vertex(tmp[0] - cx, tmp[1] - cy, tmp[2] - cz, 1.0f);
      }
      if (norAcc) {
        for (cgltf_size i = 0; i < std::min(norAcc->count, posAcc->count); ++i) {
          float n[3] = {0, 0, 0};
          cgltf_accessor_read_float(norAcc, i, n, 3);
          vertices[i].nx = n[0];
          vertices[i].ny = n[1];
          vertices[i].nz = n[2];
        }
      }
      if (uvAcc) {
        for (cgltf_size i = 0; i < std::min(uvAcc->count, posAcc->count); ++i) {
          float t[2] = {0, 0};
          cgltf_accessor_read_float(uvAcc, i, t, 2);
          vertices[i].u = t[0];
          vertices[i].v = t[1];
        }
      }

      std::vector<uint32_t> indices;
      if (prim.indices) {
        const cgltf_accessor *idxAcc = prim.indices;
        indices.resize((size_t)idxAcc->count);
        for (cgltf_size i = 0; i < idxAcc->count; ++i)
          indices[i] = (uint32_t)cgltf_accessor_read_index(idxAcc, i);
      } else {
        indices.resize((size_t)posAcc->count);
        for (cgltf_size i = 0; i < posAcc->count; ++i)
          indices[i] = (uint32_t)i;
      }

      std::vector<uint32_t> triIndices;
      if (prim.type == cgltf_primitive_type_triangles) {
        triIndices = indices;
      } else if (prim.type == cgltf_primitive_type_triangle_strip) {
        if (indices.size() >= 3) {
          triIndices.reserve((indices.size() - 2) * 3);
          for (size_t i = 0; i + 2 < indices.size(); ++i) {
            if ((i & 1) == 0) {
              triIndices.push_back(indices[i + 0]);
              triIndices.push_back(indices[i + 1]);
              triIndices.push_back(indices[i + 2]);
            } else {
              triIndices.push_back(indices[i + 1]);
              triIndices.push_back(indices[i + 0]);
              triIndices.push_back(indices[i + 2]);
            }
          }
        }
      } else if (prim.type == cgltf_primitive_type_triangle_fan) {
        if (indices.size() >= 3) {
          triIndices.reserve((indices.size() - 2) * 3);
          uint32_t c = indices[0];
          for (size_t i = 1; i + 1 < indices.size(); ++i) {
            triIndices.push_back(c);
            triIndices.push_back(indices[i]);
            triIndices.push_back(indices[i + 1]);
          }
        }
      } else {
        fmt::print("[GLB]  skipping unsupported primitive type {}\n", (int)prim.type);
      }

      uint16_t *decodedTex = nullptr;
      int texW = 0, texH = 0;
      float baseColor[3] = {1, 1, 1};
      if (prim.material) {
        const cgltf_material *mat = prim.material;
        baseColor[0] = (float)mat->pbr_metallic_roughness.base_color_factor[0];
        baseColor[1] = (float)mat->pbr_metallic_roughness.base_color_factor[1];
        baseColor[2] = (float)mat->pbr_metallic_roughness.base_color_factor[2];
        if (outMat) {
          outMat->kd[0] = baseColor[0];
          outMat->kd[1] = baseColor[1];
          outMat->kd[2] = baseColor[2];
        }
        const cgltf_texture_view &bct = mat->pbr_metallic_roughness.base_color_texture;
        if (bct.texture && bct.texture->image) {
          const cgltf_image *img = bct.texture->image;
          bool ok = false;
          if (img->buffer_view && img->buffer_view->buffer && img->buffer_view->buffer->data &&
              bytesDecoder) {
            const uint8_t *ptr = (const uint8_t *)img->buffer_view->buffer->data;
            ptr += img->buffer_view->offset;
            size_t len = img->buffer_view->size;
            const char *mime = img->mime_type ? img->mime_type : "image/png";
            ok = bytesDecoder(ptr, len, mime, decodedTex, texW, texH);
          } else if (img->uri) {
            string uri = img->uri;
            if (uri.rfind("data:", 0) == 0 && bytesDecoder) {
              size_t comma = uri.find(',');
              if (comma != string::npos) {
                string header = uri.substr(5, comma - 5);
                string payload = uri.substr(comma + 1);
                size_t sc = header.find(';');
                string mimeType = (sc != string::npos) ? header.substr(0, sc) : header;
                auto b64dec = [](const string &in, std::vector<uint8_t> &out) {
                  static int8_t LUT[256];
                  static bool init = false;
                  if (!init) {
                    for (int i = 0; i < 256; i++)
                      LUT[i] = -1;
                    const string A =
                        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
                    for (int i = 0; i < (int)A.size(); ++i)
                      LUT[(uint8_t)A[i]] = (int8_t)i;
                    LUT[(uint8_t)'='] = 0;
                    init = true;
                  }
                  int val = 0, valb = -8;
                  out.clear();
                  out.reserve(in.size() * 3 / 4);
                  for (uint8_t c : in) {
                    int8_t d = LUT[c];
                    if (d == -1)
                      continue;
                    val = (val << 6) + d;
                    valb += 6;
                    if (valb >= 0) {
                      out.push_back((uint8_t)((val >> valb) & 0xFF));
                      valb -= 8;
                    }
                  }
                  return true;
                };
                std::vector<uint8_t> bytes;
                b64dec(payload, bytes);
                ok = bytesDecoder(bytes.data(), bytes.size(), mimeType, decodedTex, texW, texH);
              }
            } else if (fileDecoder) {
              fs::path full = fs::path(dir_of(glbPath)) / uri;
              if (outMat)
                outMat->texturePath = full.string();
              ok = fileDecoder(full.string(), decodedTex, texW, texH);
            }
          }
          if (ok && outMat)
            outMat->hasTexture = true;
        }
      }

      const bool haveTexture = decodedTex && texW > 0 && texH > 0;
      outObject.AddMesh(vertices, triIndices, haveTexture ? TEXTURED : COLORED, decodedTex, texW,
                        texH, baseColor[0], baseColor[1], baseColor[2]);
      total_vertices += vertices.size();
      total_tris += triIndices.size() / 3;
      fmt::print("[GLB]  emitted mesh prim={} verts={} tris={} textured={}\n", (int)primIndex,
                 (int)vertices.size(), (int)(triIndices.size() / 3), haveTexture);
    }
  }

  fmt::print("[GLB] total vertices {} total tris {}\n", (int)total_vertices, (int)total_tris);
  fmt::print("[GLB] model bounds: min ({:.3f}, {:.3f}, {:.3f}) max ({:.3f}, {:.3f}, {:.3f})\n",
             minx, miny, minz, maxx, maxy, maxz);

  cgltf_free(data);
  return true;
}

#else // ENGINE_USE_CGLTF

#include "cJSON.h"

// cJSON-based minimal GLB reader (fallback)
bool LoadGLB(const string &glbPath, Object &outObject, MaterialInfo *outMat,
             TextureDecodeFn fileDecoder, TextureDecodeBytesFn bytesDecoder) {
  std::ifstream f(glbPath, std::ios::binary);
  if (!f)
    return false;

  auto readU32 = [&](uint32_t &v) { f.read(reinterpret_cast<char *>(&v), 4); };

  // GLB header
  uint32_t magic = 0, version = 0, length = 0;
  readU32(magic);
  readU32(version);
  readU32(length);
  if (magic != 0x46546C67 /*glTF*/)
    return false;

  // JSON chunk
  uint32_t chunkLen = 0, chunkType = 0;
  readU32(chunkLen);
  readU32(chunkType);
  if (chunkType != 0x4E4F534A /*JSON*/)
    return false;
  std::vector<char> json(chunkLen);
  f.read(json.data(), chunkLen);

  // BIN chunk (optional but expected for buffers)
  uint32_t binLen = 0, binType = 0;
  if (f.peek() != EOF) {
    readU32(binLen);
    readU32(binType);
  }
  if (binLen == 0 || binType != 0x004E4942 /*BIN*/)
    return false;
  std::vector<uint8_t> bin(binLen);
  f.read(reinterpret_cast<char *>(bin.data()), binLen);

  // Parse JSON with cJSON
  cJSON *root = cJSON_ParseWithLength(json.data(), json.size());
  if (!root)
    return false;
  cJSON *meshes = cJSON_GetObjectItem(root, "meshes");
  if (!cJSON_IsArray(meshes)) {
    cJSON_Delete(root);
    return false;
  }
  cJSON *mesh0 = cJSON_GetArrayItem(meshes, 0);
  if (!mesh0) {
    cJSON_Delete(root);
    return false;
  }
  cJSON *prims = cJSON_GetObjectItem(mesh0, "primitives");
  if (!cJSON_IsArray(prims)) {
    cJSON_Delete(root);
    return false;
  }
  cJSON *prim0 = cJSON_GetArrayItem(prims, 0);
  if (!prim0) {
    cJSON_Delete(root);
    return false;
  }
  cJSON *attrs = cJSON_GetObjectItem(prim0, "attributes");
  if (!cJSON_IsObject(attrs)) {
    cJSON_Delete(root);
    return false;
  }
  cJSON *posItem = cJSON_GetObjectItem(attrs, "POSITION");
  if (!cJSON_IsNumber(posItem)) {
    cJSON_Delete(root);
    return false;
  }
  int posAcc = posItem->valueint;
  int norAcc = cJSON_IsNumber(cJSON_GetObjectItem(attrs, "NORMAL"))
                   ? cJSON_GetObjectItem(attrs, "NORMAL")->valueint
                   : -1;
  int uvAcc = cJSON_IsNumber(cJSON_GetObjectItem(attrs, "TEXCOORD_0"))
                  ? cJSON_GetObjectItem(attrs, "TEXCOORD_0")->valueint
                  : -1;
  cJSON *indItem = cJSON_GetObjectItem(prim0, "indices");
  if (!cJSON_IsNumber(indItem)) {
    cJSON_Delete(root);
    return false;
  }
  int idxAcc = indItem->valueint;
  int materialIndex = cJSON_IsNumber(cJSON_GetObjectItem(prim0, "material"))
                          ? cJSON_GetObjectItem(prim0, "material")->valueint
                          : -1;

  cJSON *bufferViews = cJSON_GetObjectItem(root, "bufferViews");
  if (!cJSON_IsArray(bufferViews)) {
    cJSON_Delete(root);
    return false;
  }
  cJSON *accessors = cJSON_GetObjectItem(root, "accessors");
  if (!cJSON_IsArray(accessors)) {
    cJSON_Delete(root);
    return false;
  }

  auto readView = [&](int viewIndex, size_t &byteOffset, size_t &byteLength, size_t &stride) {
    cJSON *view = cJSON_GetArrayItem(bufferViews, viewIndex);
    if (!view)
      return false;
    cJSON *bo = cJSON_GetObjectItem(view, "byteOffset");
    byteOffset = cJSON_IsNumber(bo) ? (size_t)bo->valuedouble : 0;
    cJSON *bl = cJSON_GetObjectItem(view, "byteLength");
    if (!cJSON_IsNumber(bl))
      return false;
    byteLength = (size_t)bl->valuedouble;
    cJSON *bs = cJSON_GetObjectItem(view, "byteStride");
    stride = cJSON_IsNumber(bs) ? (size_t)bs->valuedouble : 0;
    return true;
  };

  auto readAccessor = [&](int accIndex, int &count, int &componentType, int &bvIndex,
                          size_t &byteOffset, int &components) {
    cJSON *acc = cJSON_GetArrayItem(accessors, accIndex);
    if (!acc)
      return false;
    cJSON *cnt = cJSON_GetObjectItem(acc, "count");
    cJSON *ct = cJSON_GetObjectItem(acc, "componentType");
    cJSON *bv = cJSON_GetObjectItem(acc, "bufferView");
    if (!cJSON_IsNumber(cnt) || !cJSON_IsNumber(ct) || !cJSON_IsNumber(bv))
      return false;
    count = cnt->valueint;
    componentType = ct->valueint;
    bvIndex = bv->valueint;
    cJSON *bo = cJSON_GetObjectItem(acc, "byteOffset");
    byteOffset = cJSON_IsNumber(bo) ? (size_t)bo->valuedouble : 0;
    const char *type = cJSON_GetObjectItem(acc, "type")->valuestring;
    if (std::strcmp(type, "SCALAR") == 0)
      components = 1;
    else if (std::strcmp(type, "VEC2") == 0)
      components = 2;
    else if (std::strcmp(type, "VEC3") == 0)
      components = 3;
    else if (std::strcmp(type, "VEC4") == 0)
      components = 4;
    else
      components = 0;
    return true;
  };

  auto readFloatVec = [&](int accIndex, std::vector<float> &out) {
    int count = 0, compType = 0, viewIdx = 0, comps = 0;
    size_t accOff = 0;
    if (!readAccessor(accIndex, count, compType, viewIdx, accOff, comps))
      return false;
    if (compType != 5126 /*FLOAT*/)
      return false;
    size_t bvOff = 0, bvLen = 0, stride = 0;
    if (!readView(viewIdx, bvOff, bvLen, stride))
      return false;
    if (stride == 0)
      stride = comps * sizeof(float);
    const uint8_t *ptr = bin.data() + bvOff + accOff;
    out.resize((size_t)count * comps);
    for (int i = 0; i < count; i++)
      memcpy(&out[i * comps], ptr + i * stride, comps * sizeof(float));
    return true;
  };

  auto readIndices = [&](int accIndex, std::vector<uint32_t> &out) {
    int count = 0, compType = 0, viewIdx = 0, comps = 0;
    size_t accOff = 0;
    if (!readAccessor(accIndex, count, compType, viewIdx, accOff, comps))
      return false;
    size_t bvOff = 0, bvLen = 0, stride = 0;
    if (!readView(viewIdx, bvOff, bvLen, stride))
      return false;
    const uint8_t *ptr = bin.data() + bvOff + accOff;
    out.resize(count);
    switch (compType) {
    case 5123: // UNSIGNED_SHORT
      for (int i = 0; i < count; i++)
        out[i] = ((const uint16_t *)ptr)[i];
      break;
    case 5125: // UNSIGNED_INT
      for (int i = 0; i < count; i++)
        out[i] = ((const uint32_t *)ptr)[i];
      break;
    default:
      return false;
    }
    return true;
  };

  std::vector<float> positions, normals, texcoords;
  std::vector<uint32_t> indices;
  if (!readFloatVec(posAcc, positions)) {
    cJSON_Delete(root);
    return false;
  }
  if (norAcc >= 0)
    readFloatVec(norAcc, normals);
  if (uvAcc >= 0)
    readFloatVec(uvAcc, texcoords);
  if (!readIndices(idxAcc, indices)) {
    cJSON_Delete(root);
    return false;
  }

  // Build polys from indexed triangles
  for (size_t i = 0; i + 2 < indices.size(); i += 3) {
    auto mkV = [&](uint32_t idx) -> Vertex {
      Vertex v(positions[idx * 3 + 0], positions[idx * 3 + 1], positions[idx * 3 + 2], 1.0f);
      if (!normals.empty()) {
        v.nx = normals[idx * 3 + 0];
        v.ny = normals[idx * 3 + 1];
        v.nz = normals[idx * 3 + 2];
      }
      if (!texcoords.empty()) {
        v.u = texcoords[idx * 2 + 0];
        v.v = texcoords[idx * 2 + 1];
      }
      return v;
    };
    Vertex v0 = mkV(indices[i + 0]);
    Vertex v1 = mkV(indices[i + 1]);
    Vertex v2 = mkV(indices[i + 2]);
    Poly poly(v0, v1, v2, Vertex(), 3, Vector3D(0, 0, 1), TEXTURED);
    outObject.add(poly);
  }

  // Images block: support bufferView (embedded), data: URI, and file URI
  struct ImageData {
    const uint8_t *ptr{nullptr};
    size_t len{0};
    string mime;
    string file;
    std::vector<uint8_t> storage;
  };
  std::vector<ImageData> images;
  cJSON *imagesArr = cJSON_GetObjectItem(root, "images");
  if (cJSON_IsArray(imagesArr)) {
    int imgCount = cJSON_GetArraySize(imagesArr);
    images.reserve(imgCount);
    for (int i = 0; i < imgCount; i++) {
      cJSON *img = cJSON_GetArrayItem(imagesArr, i);
      ImageData im;
      cJSON *bv = cJSON_GetObjectItem(img, "bufferView");
      cJSON *uri = cJSON_GetObjectItem(img, "uri");
      cJSON *mt = cJSON_GetObjectItem(img, "mimeType");
      if (cJSON_IsNumber(bv)) {
        int viewIdx = bv->valueint;
        size_t off = 0, len = 0, stride = 0;
        if (readView(viewIdx, off, len, stride)) {
          im.ptr = bin.data() + off;
          im.len = len;
          im.mime = mt && cJSON_IsString(mt) ? mt->valuestring : "image/jpeg";
        }
      } else if (cJSON_IsString(uri)) {
        string u = uri->valuestring;
        if (u.rfind("data:", 0) == 0) {
          size_t comma = u.find(",");
          if (comma != string::npos) {
            string header = u.substr(5, comma - 5);
            string payload = u.substr(comma + 1);
            size_t sc = header.find(';');
            string mimeType = (sc != string::npos) ? header.substr(0, sc) : header;
            auto b64dec = [](const string &in, std::vector<uint8_t> &out) {
              static int8_t LUT[256];
              static bool init = false;
              if (!init) {
                for (int i = 0; i < 256; i++)
                  LUT[i] = -1;
                const string A = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
                for (int i = 0; i < (int)A.size(); ++i)
                  LUT[(uint8_t)A[i]] = (int8_t)i;
                LUT[(uint8_t)'='] = 0;
                init = true;
              }
              int val = 0, valb = -8;
              out.clear();
              out.reserve(in.size() * 3 / 4);
              for (uint8_t c : in) {
                int8_t d = LUT[c];
                if (d == -1)
                  continue;
                val = (val << 6) + d;
                valb += 6;
                if (valb >= 0) {
                  out.push_back((uint8_t)((val >> valb) & 0xFF));
                  valb -= 8;
                }
              }
              return true;
            };
            b64dec(payload, im.storage);
            im.ptr = im.storage.data();
            im.len = im.storage.size();
            im.mime = mimeType;
          }
        } else {
          im.file = (fs::path(glbPath).parent_path() / u).string();
        }
      }
      images.push_back(im);
    }
  }

  // Decode material texture if present for this primitive and apply to its polys
  if (materialIndex >= 0) {
    cJSON *mats = cJSON_GetObjectItem(root, "materials");
    if (cJSON_IsArray(mats)) {
      cJSON *mat = cJSON_GetArrayItem(mats, materialIndex);
      if (mat) {
        cJSON *pbr = cJSON_GetObjectItem(mat, "pbrMetallicRoughness");
        if (pbr) {
          cJSON *bct = cJSON_GetObjectItem(pbr, "baseColorTexture");
          if (bct) {
            int texIndex = cJSON_GetObjectItem(bct, "index")->valueint;
            cJSON *textures = cJSON_GetObjectItem(root, "textures");
            int imgIndex = -1;
            if (cJSON_IsArray(textures)) {
              cJSON *texObj = cJSON_GetArrayItem(textures, texIndex);
              if (texObj) {
                cJSON *src = cJSON_GetObjectItem(texObj, "source");
                if (cJSON_IsNumber(src))
                  imgIndex = src->valueint;
              }
            }
            if (imgIndex >= 0 && (size_t)imgIndex < images.size()) {
              const auto &im = images[(size_t)imgIndex];
              uint16_t *texPtr = nullptr;
              int tw = 0, th = 0;
              bool ok = false;
              if (!im.file.empty() && fileDecoder)
                ok = fileDecoder(im.file, texPtr, tw, th);
              else if (im.ptr && im.len > 0 && bytesDecoder)
                ok = bytesDecoder(im.ptr, im.len, im.mime, texPtr, tw, th);
              if (ok && texPtr && tw > 0 && th > 0) {
                if (outMat) {
                  outMat->hasTexture = true;
                  outMat->texturePath = im.file;
                }
                auto list = outObject.GetRenderList();
                for (auto &p : list)
                  p.SetTexture(texPtr, tw, th);
                outObject.updateList(list);
              }
            }
          } else {
            cJSON *cf = cJSON_GetObjectItem(pbr, "baseColorFactor");
            if (cJSON_IsArray(cf) && cJSON_GetArraySize(cf) >= 3 && outMat) {
              outMat->kd[0] = (float)cJSON_GetArrayItem(cf, 0)->valuedouble;
              outMat->kd[1] = (float)cJSON_GetArrayItem(cf, 1)->valuedouble;
              outMat->kd[2] = (float)cJSON_GetArrayItem(cf, 2)->valuedouble;
            }
          }
        }
      }
    }
  }

  cJSON_Delete(root);
  return true;
}

#endif // ENGINE_USE_CGLTF

} // namespace asset
