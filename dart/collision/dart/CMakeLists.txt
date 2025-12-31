# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB engine_hdrs "engine/*.hpp")
file(GLOB engine_srcs "engine/*.cpp")
dart_add_core_headers(${hdrs} ${engine_hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${engine_srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_dart headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME dart
  TARGET_DIR "dart/collision/dart/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/dart.hpp ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION include/dart/collision/dart
  COMPONENT headers
)
install(
  FILES ${engine_hdrs}
  DESTINATION include/dart/collision/dart/engine
  COMPONENT headers
)
