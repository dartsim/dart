# Dependency checks
if (NOT TARGET dart-gui)
  return()
endif()

# OpenSceneGraph
if(DART_BUILD_GUI_OSG)
  dart_find_package(OpenSceneGraph)
  dart_check_optional_package(OPENTHREADS "dart-gui-osg" "libopenscenegraph-dev")
  dart_check_optional_package(OSG "dart-gui-osg" "libopenscenegraph-dev")
  dart_check_optional_package(OPENSCENEGRAPH "dart-gui-osg" "libopenscenegraph-dev")
else()
  message(STATUS "Skipping OpenSceneGraph (DART_BUILD_GUI_OSG == ${DART_BUILD_GUI_OSG})")
  return()
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_osg_hdrs ${hdrs})
set(dart_gui_osg_srcs ${srcs})

add_subdirectory(render)

# Set local target name
set(target_name ${PROJECT_NAME}-gui-osg)
set(component_name gui-osg)

# Add target
dart_add_library(${target_name} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  PUBLIC
    dart-gui
    ${OPENSCENEGRAPH_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} gui)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} OpenSceneGraph)

# Generate header for this namespace
dart_get_filename_components(header_names "gui osg headers" ${hdrs})
list(APPEND header_names "render/render.hpp")
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/osg.hpp"
  "dart/gui/osg/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/osg.hpp
  DESTINATION include/dart/gui/osg
  COMPONENT headers
)

dart_format_add(
  RealTimeWorldNode.hpp
  RealTimeWorldNode.cpp
)
