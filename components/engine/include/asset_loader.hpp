#pragma once

#include <functional>
#include <string>

#include "object.hpp"

struct MaterialInfo {
  bool hasTexture{false};
  std::string texturePath;       // path to map_Kd if present (relative to model file)
  float kd[3]{1.0f, 1.0f, 1.0f}; // diffuse color from MTL (Kd)
};

namespace asset {

using TextureDecodeFn =
    std::function<bool(const std::string &path, uint16_t *&outPtr, int &outWidth, int &outHeight)>;

using TextureDecodeBytesFn =
    std::function<bool(const uint8_t *bytes, size_t length, const std::string &mimeType,
                       uint16_t *&outPtr, int &outWidth, int &outHeight)>;

// Load a Wavefront OBJ (and optional MTL) from filesystem into an Object.
// - Populates Object with triangles as Poly (TEXTURED if vt present and map_Kd found, else COLORED)
// - On success, fills outMat with texture path (if any) and diffuse color
// - Texture pixels are NOT loaded here; caller can decode and apply via Poly::SetTexture later
bool LoadOBJ(const std::string &objPath, Object &outObject, MaterialInfo *outMat = nullptr,
             TextureDecodeFn decoder = nullptr);

// Load a glTF 2.0 binary (.glb) model into an Object (minimal subset).
// - Current implementation is a stub that returns false (not yet supported on this target)
bool LoadGLB(const std::string &glbPath, Object &outObject, MaterialInfo *outMat = nullptr,
             TextureDecodeFn fileDecoder = nullptr, TextureDecodeBytesFn bytesDecoder = nullptr);

} // namespace asset
