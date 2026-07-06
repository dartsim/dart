# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

dart_add_io_headers(${hdrs})
dart_add_io_sources(${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "io_urdf headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME urdf
  TARGET_DIR "dart/io/urdf/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES
    ${hdrs}
    ${CMAKE_CURRENT_BINARY_DIR}/All.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/urdf.hpp
  DESTINATION include/dart/io/urdf
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
