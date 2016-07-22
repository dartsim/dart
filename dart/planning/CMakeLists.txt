# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

include_directories(SYSTEM ${FLANN_INCLUDE_DIRS})

# Library
dart_add_library(dart-planning ${srcs} ${hdrs})
target_link_libraries(dart-planning dart ${FLANN_LIBRARIES})

# Generate header for this namespace
dart_get_filename_components(header_names "planning headers" ${hdrs})
dart_generate_include_header_list(
  planning_headers
  "dart/planning/"
  "planning headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/planning.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/planning.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/planning.hpp
  DESTINATION include/dart/planning
  COMPONENT headers
)
install(TARGETS dart-planning EXPORT DART-PLANNINGTargets DESTINATION lib)
install(EXPORT DART-PLANNINGTargets DESTINATION share/dart)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
