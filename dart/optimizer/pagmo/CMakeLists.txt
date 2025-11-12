# Dependency checks
set(_dart_build_mode "${DART_BUILD_OPTIMIZER_PAGMO}")
if(_dart_build_mode STREQUAL "OFF")
  if(DART_VERBOSE)
    message(STATUS "Skipping dart-optimizer-pagmo (DART_BUILD_OPTIMIZER_PAGMO=OFF)")
  endif()
  set(DART_HAVE_PAGMO FALSE CACHE BOOL "Whether PaGMO support is enabled" FORCE)
  return()
endif()

dart_find_package(pagmo)

if(NOT pagmo_FOUND)
  set(DART_HAVE_PAGMO FALSE CACHE BOOL "Whether PaGMO support is enabled" FORCE)
  if(_dart_build_mode STREQUAL "ON")
    message(FATAL_ERROR "DART_BUILD_OPTIMIZER_PAGMO=ON but pagmo was not found.")
  elseif(DART_VERBOSE)
    message(STATUS "Skipping dart-optimizer-pagmo (pagmo not found)")
  endif()
  return()
endif()

set(DART_HAVE_PAGMO TRUE CACHE BOOL "Whether PaGMO support is enabled" FORCE)

find_package(Threads REQUIRED)

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-optimizer-pagmo)
set(component_name optimizer-pagmo)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC dart Pagmo::pagmo)
target_compile_definitions(${target_name}
  PUBLIC BOOST_ALL_NO_EMBEDDED_GDB_SCRIPTS)

# Thread
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(${target_name} PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(${target_name} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} pagmo NLOPT)

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer_pagmo headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME pagmo
  TARGET_DIR "dart/optimizer/pagmo/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/pagmo.hpp ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION include/dart/optimizer/pagmo
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
