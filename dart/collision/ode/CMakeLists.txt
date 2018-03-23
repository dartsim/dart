# Dependency checks
find_package(ODE 0.13 QUIET)
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
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC ${ODE_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  dart
  ${ODE_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)

# Generate export header
dart_add_export_file(${target_name} COMPONENT_PATH collision/ode)

# Generate header for this namespace
dart_get_filename_components(header_names "collision_ode headers" ${hdrs})
dart_generate_include_header_list(
  collision_ode_headers
  "dart/collision/ode/"
  "collision_ode headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ode.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/ode.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/ode.hpp
  DESTINATION include/dart/collision/ode
  COMPONENT headers
)
