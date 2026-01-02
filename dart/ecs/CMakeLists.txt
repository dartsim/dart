# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target/component names
set(target_name ${PROJECT_NAME}-ecs)
set(component_name ecs)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})

target_include_directories(
  ${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${target_name} PUBLIC cxx_std_20)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})

# Generate header for this namespace
dart_get_filename_components(header_names "ecs headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME ecs
  TARGET_DIR "dart/ecs/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install (includes All.hpp, ecs.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/ecs.hpp
  DESTINATION include/dart/ecs
  COMPONENT headers
)

# Install generated compatibility headers preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/ecs
)

dart_format_add(${hdrs} ${srcs})
