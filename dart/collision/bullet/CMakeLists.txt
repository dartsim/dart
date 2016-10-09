# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-collision-bullet)
set(component_name collision-bullet)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC ${BULLET_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  dart
  ${BULLET_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)

# Coverage test
dart_coveralls_add(${hdrs} ${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_bullet headers" ${hdrs})
dart_generate_include_header_list(
  collision_bullet_headers
  "dart/collision/bullet/"
  "collision_bullet headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bullet.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/bullet.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/bullet.hpp
  DESTINATION include/dart/collision/bullet
  COMPONENT headers
)
