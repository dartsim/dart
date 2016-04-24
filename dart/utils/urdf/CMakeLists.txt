# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS})

# Library
dart_add_library(dart-utils-urdf ${hdrs} ${srcs})
target_link_libraries(dart-utils-urdf dart-utils ${urdfdom_LIBRARIES})

# Generate header for this namespace
dart_get_filename_components(header_names "utils_urdf headers" ${hdrs})
dart_generate_include_header_list(
  utils_urdf_headers
  "dart/utils/urdf/"
  "utils_urdf headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/urdf.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/urdf.hpp
  DESTINATION include/dart/utils/urdf
  COMPONENT headers
)
install(TARGETS dart-utils-urdf EXPORT DARTTargets DESTINATION lib)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
