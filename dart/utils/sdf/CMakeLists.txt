dart_check_optional_package(SDFORMAT "SDF loading" "sdformat" "14.8")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

dart_add_utils_headers(${hdrs} ${detail_hdrs})
dart_add_utils_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "utils_sdf headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/sdf.hpp"
  "dart/utils/sdf/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/sdf.hpp
  DESTINATION include/dart/utils/sdf
  COMPONENT headers
)

# Install the finder only on SDF-on builds for examples using sdformat directly.
install(
  FILES "${DART_SOURCE_DIR}/cmake/DARTFindsdformat.cmake"
  DESTINATION "${CONFIG_INSTALL_DIR}"
)
