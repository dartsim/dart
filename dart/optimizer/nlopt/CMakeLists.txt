# Dependency checks
dart_find_package(NLOPT)
dart_check_optional_package(NLOPT "dart-optimizer-nlopt" "nlopt" "2.4.1")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-optimizer-nlopt)
set(component_name optimizer-nlopt)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC dart NLOPT::nlopt)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} NLOPT)

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer_nlopt headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/nlopt.hpp"
  "dart/optimizer/nlopt/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/nlopt.hpp
  DESTINATION include/dart/optimizer/nlopt
  COMPONENT headers
)
