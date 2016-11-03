# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "lcpsolver headers" ${hdrs})
dart_generate_include_header_list(
  lcpsolver_headers
  "dart/lcpsolver/"
  "lcpsolver headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/lcpsolver.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/lcpsolver.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/lcpsolver.hpp
  DESTINATION include/dart/lcpsolver
  COMPONENT headers
)
