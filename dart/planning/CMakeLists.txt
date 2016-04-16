# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

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
  ${CMAKE_CURRENT_SOURCE_DIR}/planning.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/planning.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/planning.h
  DESTINATION include/dart/planning
  COMPONENT headers
)
install(TARGETS dart-planning EXPORT DARTTargets DESTINATION lib)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
