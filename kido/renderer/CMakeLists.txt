# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(kido_renderer_hdrs ${hdrs} PARENT_SCOPE)
set(kido_renderer_srcs ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_renderer ${srcs} ${hdrs})
#target_link_libraries(kido_renderer ${KIDO_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "renderer headers" ${hdrs})
kido_generate_include_header_list(
  renderer_headers
  "kido/renderer/"
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
  DESTINATION include/kido/renderer
  COMPONENT headers
)
#install(TARGETS kido_renderer EXPORT KIDOTargets DESTINATION lib)

