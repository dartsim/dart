# Dependency checks
dart_find_package(flann)
dart_check_optional_package(FLANN "dart-planning" "flann" "1.8.4")

# Add lz4 dependency explicitly. lz4 is a dependency of flann, but it's not
# added to flann as the dependency.
dart_find_package(lz4)
dart_check_optional_package(lz4 "dart-planning" "lz4")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-planning)
set(component_name planning)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC dart flann lz4)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} flann lz4)

## Generate header for this namespace
dart_get_filename_components(header_names "planning headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/planning.hpp"
  "dart/planning/"
  ${header_names}
)

## Install
if(NOT DART_BUILD_DARTPY)
  install(
    FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/planning.hpp
    DESTINATION include/dart/planning
    COMPONENT headers
  )
endif()

dart_format_add(${hdrs} ${srcs})
