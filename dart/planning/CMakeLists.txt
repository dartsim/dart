# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-planning)
set(component_name planning)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC ${FLANN_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  dart
  ${FLANN_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)

# Coverage test
dart_coveralls_add(${hdrs} ${srcs})

## Generate header for this namespace
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

## Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/planning.hpp
  DESTINATION include/dart/planning
  COMPONENT headers
)
