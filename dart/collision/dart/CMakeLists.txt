# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_dart headers" ${hdrs})
dart_generate_include_header_list(
  collision_dart_headers
  "dart/collision/dart/"
  "collision_dart headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/dart.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/dart.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/dart.hpp
  DESTINATION include/dart/collision/dart
  COMPONENT headers
)
