# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Dependency checks
if (NOT TARGET dart-gui)
  return()
endif()

dart_find_package(GLUT)
if(WIN32 AND NOT CYGWIN)
  dart_check_optional_package(GLUT "dart-gui" "freeglut")
else()
  dart_check_optional_package(GLUT "dart-gui" "freeglut3")
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-gui-glut)
set(component_name gui-glut)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC dart-gui GLUT::GLUT)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} gui)
if(NOT WIN32 OR CYGWIN)
  add_component_dependency_packages(${PROJECT_NAME} ${component_name} GLUT)
endif()

# Generate header for this namespace
dart_get_filename_components(header_names "gui glut headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/glut.hpp"
  "dart/gui/glut/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/glut.hpp
  DESTINATION include/dart/gui/glut
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
