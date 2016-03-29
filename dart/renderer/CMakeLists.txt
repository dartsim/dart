# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(dart_renderer_hdrs ${hdrs} PARENT_SCOPE)
set(dart_renderer_srcs ${srcs} PARENT_SCOPE)

# Generate header for this namespace
dart_get_filename_components(header_names "renderer headers" ${hdrs})
dart_generate_include_header_list(
  renderer_headers
  "dart/renderer/"
  "renderer headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/renderer.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/renderer.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/renderer.hpp
  DESTINATION include/dart/renderer
  COMPONENT headers
)
