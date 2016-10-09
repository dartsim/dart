# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "dynamics headers" ${hdrs})
dart_generate_include_header_list(
  dynamics_headers
  "dart/dynamics/"
  "dynamics headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/dynamics.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/dynamics.hpp
  DESTINATION include/dart/dynamics
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/dynamics/detail
  COMPONENT headers
)
