# Set local target name
set(target_name ${PROJECT_NAME}-gui-osg)
set(component_name gui-osg)

# Dependency checks
dart_check_dependent_target(${target_name} dart-gui)

# Minimum required OSG version
set(min_osg_version 3.0.0)
# OpenSceneGraph 3.6.5 or less are not compatible with macOS 10.15 (Catalina) or greater
# See:
#   - https://github.com/openscenegraph/OpenSceneGraph/issues/926
#   - https://github.com/dartsim/dart/issues/1439
if(APPLE)
  if(NOT ${CMAKE_SYSTEM_VERSION} VERSION_LESS 19)
    set(min_osg_version 3.7.0)
  endif()
endif()

# OpenSceneGraph
if(DART_BUILD_GUI_OSG)
  dart_find_package(OpenSceneGraph)
  dart_check_optional_package(OpenSceneGraph "dart-gui-osg" "OpenSceneGraph" "3.0")
else()
  message(STATUS "Skipping OpenSceneGraph (DART_BUILD_GUI_OSG == ${DART_BUILD_GUI_OSG})")
  return()
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

set(dart_gui_osg_hdrs ${hdrs} ${detail_hdrs})
set(dart_gui_osg_srcs ${srcs} ${detail_srcs})

add_subdirectory(render)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
target_link_libraries(${target_name} dart-gui osg::osg)

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
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/gui/osg/detail
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
