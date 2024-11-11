# Dependency checks
dart_find_package(NLOPT)
dart_check_optional_package(NLOPT "dart-optimizer-nlopt" "nlopt" "2.4.1")

if(NOT NLOPT_VERSION)
  # If version is not found, we just assume 2.9.0
  set(NLOPT_VERSION "2.9.0")
endif()

# Attempt to parse version components
string(REGEX REPLACE "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$" "\\1" NLOPT_MAJOR_VERSION "${NLOPT_VERSION}")
string(REGEX REPLACE "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$" "\\2" NLOPT_MINOR_VERSION "${NLOPT_VERSION}")
string(REGEX REPLACE "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$" "\\3" NLOPT_PATCH_VERSION "${NLOPT_VERSION}")

# Check if parsing succeeded
if(NOT (NLOPT_MAJOR_VERSION MATCHES "^[0-9]+$" AND 
        NLOPT_MINOR_VERSION MATCHES "^[0-9]+$" AND 
        NLOPT_PATCH_VERSION MATCHES "^[0-9]+$"))
  message(WARNING "Failed to parse NLOPT_VERSION '${NLOPT_VERSION}'. Using default values (2, 9, 0) for version components.")
  set(NLOPT_MAJOR_VERSION "2")
  set(NLOPT_MINOR_VERSION "9")
  set(NLOPT_PATCH_VERSION "0")
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-optimizer-nlopt)
set(component_name optimizer-nlopt)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC dart NLOPT::nlopt)
target_compile_definitions(${target_name}
  PUBLIC
    -DNLOPT_MAJOR_VERSION=${NLOPT_MAJOR_VERSION}
    -DNLOPT_MINOR_VERSION=${NLOPT_MINOR_VERSION}
    -DNLOPT_PATCH_VERSION=${NLOPT_PATCH_VERSION}
)

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

dart_format_add(${hdrs} ${srcs})
