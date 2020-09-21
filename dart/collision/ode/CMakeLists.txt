# Dependency checks
dart_find_package(ODE)
dart_check_optional_package(ODE "dart-collision-ode" "ode" "0.13")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-collision-ode)
set(component_name collision-ode)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
target_link_libraries(${target_name} PUBLIC dart ODE::ODE)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} ODE)

# Generate header for this namespace
dart_get_filename_components(header_names "collision_ode headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/ode.hpp"
  "dart/collision/ode/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/ode.hpp
  DESTINATION include/dart/collision/ode
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
