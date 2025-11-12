# Dependency checks
set(_dart_build_mode "${DART_BUILD_COLLISION_ODE}")
if(_dart_build_mode STREQUAL "OFF")
  if(DART_VERBOSE)
    message(STATUS "Skipping dart-collision-ode (DART_BUILD_COLLISION_ODE=OFF)")
  endif()
  set(DART_HAVE_ODE FALSE CACHE BOOL "Whether ODE support is enabled" FORCE)
  return()
endif()

dart_find_package(ODE)

if(NOT ODE_FOUND)
  message(WARNING "Looking for ODE - NOT found, to use dart-collision-ode, please install libode-dev")
  set(DART_HAVE_ODE FALSE CACHE BOOL "Whether ODE support is enabled" FORCE)
  if(_dart_build_mode STREQUAL "ON")
    message(FATAL_ERROR "DART_BUILD_COLLISION_ODE=ON but ODE was not found.")
  endif()
  return()
endif()

set(DART_HAVE_ODE TRUE CACHE BOOL "Whether ODE support is enabled" FORCE)

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
dart_generate_component_headers(
  COMPONENT_NAME ode
  TARGET_DIR "dart/collision/ode/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/ode.hpp
  DESTINATION include/dart/collision/ode
  COMPONENT headers
)

install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/collision/ode/detail
  COMPONENT headers
)

dart_install_compat_headers(COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS} DESTINATION_PREFIX include/dart/collision/ode)

dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
