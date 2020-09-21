# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

dart_add_utils_headers(${hdrs} ${detail_hdrs})
dart_add_utils_headers(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "utils_mjcf headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mjcf.hpp"
  "dart/utils/mjcf/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/mjcf.hpp
  DESTINATION include/dart/utils/mjcf
  COMPONENT headers
)
