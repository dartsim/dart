# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

dart_add_io_headers(${hdrs})
dart_add_io_headers(${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "io_sdf headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/sdf.hpp"
  "dart/io/sdf/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/sdf.hpp
  DESTINATION include/dart/io/sdf
  COMPONENT headers
)
