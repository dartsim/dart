# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs})
dart_add_core_sources(${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "lcpsolver headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/lcpsolver.hpp"
  "dart/lcpsolver/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/lcpsolver.hpp
  DESTINATION include/dart/lcpsolver
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
