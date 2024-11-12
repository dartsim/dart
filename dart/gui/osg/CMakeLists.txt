if(NOT DART_BUILD_GUI_OSG)
  message(STATUS "Skipping OpenSceneGraph (DART_BUILD_GUI_OSG == ${DART_BUILD_GUI_OSG})")
  return()
endif()

# Set local target name
set(target_name ${PROJECT_NAME}-gui-osg)
set(component_name gui-osg)
set(component_dependencies )
set(component_dependency_packages )
set(link_libraries )

# Dependency checks
dart_check_dependent_target(${target_name} dart-gui)

# OpenSceneGraph
dart_find_package(OpenSceneGraph)
dart_check_required_package(OpenSceneGraph "dart-gui-osg" "OpenSceneGraph")
list(APPEND component_dependency_packages OpenSceneGraph)

# ImGui
if(DART_USE_SYSTEM_IMGUI)
  dart_find_package(imgui)
  dart_check_required_package(imgui "imgui")
  list(APPEND component_dependency_packages imgui)
  list(APPEND link_libraries imgui::imgui)
else()
  list(APPEND component_dependencies external-imgui)
  list(APPEND link_libraries ${PROJECT_NAME}-external-imgui)
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
target_link_libraries(${target_name} dart-gui osg::osg ${link_libraries})

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} gui ${component_dependencies})
add_component_dependency_packages(${PROJECT_NAME} ${component_name} ${component_dependency_packages})

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
