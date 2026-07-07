# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

dart_add_io_headers(${hdrs})
dart_add_io_sources(${srcs})
dart_add_io_headers(${detail_hdrs})
dart_add_io_sources(${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "io_sdf headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME sdf
  TARGET_DIR "dart/io/sdf/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES
    ${hdrs}
    ${CMAKE_CURRENT_BINARY_DIR}/All.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/sdf.hpp
  DESTINATION include/dart/io/sdf
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/io/sdf/detail
  COMPONENT headers
)
