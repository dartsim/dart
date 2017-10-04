# Dependency checks
find_package(TINYXML 2.6.2 QUIET)
dart_check_optional_package(TINYXML "dart-io" "tinyxml" "2.6.2")

find_package(TINYXML2 QUIET)
dart_check_optional_package(TINYXML2 "dart-io" "tinyxml2" "1.0.1")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

function(dart_add_io_headers)
  dart_property_add(DART_IO_HEADERS ${ARGN})
endfunction(dart_add_io_headers)

function(dart_add_io_sources)
  dart_property_add(DART_IO_SOURCES ${ARGN})
endfunction(dart_add_io_sources)

# Add subdirectories
add_subdirectory(sdf)

get_property(dart_io_headers GLOBAL PROPERTY DART_IO_HEADERS)
get_property(dart_io_sources GLOBAL PROPERTY DART_IO_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-io)
set(component_name io)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${dart_io_headers} ${dart_io_sources})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC
    ${TINYXML_INCLUDE_DIRS}
    ${TINYXML2_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  dart
  ${TINYXML_LIBRARIES}
  ${TINYXML2_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)

# Generate header for this namespace
dart_get_filename_components(header_names "io headers" ${hdrs})
if(urdfdom_FOUND)
  list(APPEND header_names "urdf/urdf.hpp")
endif()
set(
  header_names
  ${header_names}
  sdf/sdf.hpp
)
dart_generate_include_header_list(
  io_headers
  "dart/io/"
  "io headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/io.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/io.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/io.hpp
  DESTINATION include/dart/io
  COMPONENT headers
)

# Add subdirectories (components)
add_subdirectory(urdf)
