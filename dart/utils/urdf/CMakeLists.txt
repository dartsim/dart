# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-utils-urdf)
set(component_name utils-urdf)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC ${urdfdom_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  dart-utils
  ${urdfdom_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} utils)

# Coverage test
dart_coveralls_add(${hdrs} ${srcs})

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
