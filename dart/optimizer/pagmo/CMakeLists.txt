# Dependency checks
find_package(PAGMO 2.8 QUIET)
dart_check_optional_package(PAGMO "dart-optimizer-pagmo" "pagmo" "2.8")

find_package(Threads REQUIRED)
dart_check_optional_package(Threads "dart-optimizer-pagmo" "Threads" "")

find_package(NLOPT 2.4.1 QUIET)
dart_check_optional_package(NLOPT "dart-optimizer-pagmo" "nlopt" "2.4.1")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-optimizer-pagmo)
set(component_name optimizer-pagmo)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC dart ${NLOPT_LIBRARIES})

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

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer_pagmo headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/pagmo.hpp"
  "dart/optimizer/pagmo/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/pagmo.hpp
  DESTINATION include/dart/optimizer/pagmo
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
