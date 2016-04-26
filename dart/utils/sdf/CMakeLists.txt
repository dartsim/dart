# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_utils_hdrs ${dart_utils_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_utils_srcs ${dart_utils_srcs} ${srcs} PARENT_SCOPE)

# Generate header for this namespace
dart_get_filename_components(header_names "utils_sdf headers" ${hdrs})
dart_generate_include_header_list(
  utils_sdf_headers
  "dart/utils/sdf/"
  "utils_sdf headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/sdf.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/sdf.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/sdf.hpp
  DESTINATION include/dart/utils/sdf
  COMPONENT headers
)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
