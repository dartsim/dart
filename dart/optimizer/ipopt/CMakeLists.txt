# Dependency checks
set(_dart_build_mode "${DART_BUILD_OPTIMIZER_IPOPT}")
if(_dart_build_mode STREQUAL "OFF")
  if(DART_VERBOSE)
    message(STATUS "Skipping dart-optimizer-ipopt (DART_BUILD_OPTIMIZER_IPOPT=OFF)")
  endif()
  set(DART_HAVE_IPOPT FALSE CACHE BOOL "Whether IPOPT support is enabled" FORCE)
  return()
endif()

dart_find_package(IPOPT)

if(NOT IPOPT_FOUND)
  set(DART_HAVE_IPOPT FALSE CACHE BOOL "Whether IPOPT support is enabled" FORCE)
  if(_dart_build_mode STREQUAL "ON")
    message(FATAL_ERROR "DART_BUILD_OPTIMIZER_IPOPT=ON but IPOPT was not found.")
  elseif(DART_VERBOSE)
    message(STATUS "Skipping dart-optimizer-ipopt (IPOPT not found)")
  endif()
  return()
endif()

set(DART_HAVE_IPOPT TRUE CACHE BOOL "Whether IPOPT support is enabled" FORCE)

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-optimizer-ipopt)
set(component_name optimizer-ipopt)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC dart IPOPT::ipopt)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} IPOPT)

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer_ipopt headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME ipopt
  TARGET_DIR "dart/optimizer/ipopt/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
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
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/ipopt.hpp ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION include/dart/optimizer/ipopt
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
