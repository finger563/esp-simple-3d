idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES math json format
  )

# Enable cgltf-based GLB loader by default (header is vendored in include)
target_compile_definitions(${COMPONENT_LIB} PRIVATE ENGINE_USE_CGLTF)
