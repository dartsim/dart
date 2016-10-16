# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "math headers" ${hdrs})
dart_generate_include_header_list(
  math_headers
  "dart/math/"
  "math headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/math.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/math.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/math.hpp
  DESTINATION include/dart/math
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/math/detail
  COMPONENT headers
)
