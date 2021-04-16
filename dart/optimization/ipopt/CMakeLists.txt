# Dependency checks
dart_find_package(IPOPT)
dart_check_optional_package(IPOPT "dart-optimization-ipopt" "ipopt" "3.11.9")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-optimization-ipopt)
set(component_name optimization-ipopt)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC dart IPOPT::ipopt)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} IPOPT)

# Generate header for this namespace
dart_get_filename_components(header_names "optimization_ipopt headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/ipopt.hpp"
  "dart/optimization/ipopt/"
  ${header_names}
)

# Add definitions for the IpOpt version
string(REPLACE "." ";" ipopt_version_list ${IPOPT_VERSION})
list(GET ipopt_version_list 0 IPOPT_VERSION_MAJOR)
list(GET ipopt_version_list 1 IPOPT_VERSION_MINOR)
list(GET ipopt_version_list 2 IPOPT_VERSION_PATCH)
target_compile_definitions(${target_name}
  PUBLIC
    IPOPT_VERSION_MAJOR=${IPOPT_VERSION_MAJOR}
    IPOPT_VERSION_MINOR=${IPOPT_VERSION_MINOR}
    IPOPT_VERSION_PATCH=${IPOPT_VERSION_PATCH}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/ipopt.hpp
  DESTINATION include/dart/optimization/ipopt
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
