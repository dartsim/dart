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
dart_check_dependent_target(${target_name} dart-utils)

# OpenSceneGraph
dart_find_package(OpenSceneGraph)
dart_check_required_package(OpenSceneGraph "dart-gui-osg" "OpenSceneGraph")
list(APPEND component_dependency_packages OpenSceneGraph)

# ImGui
if(DART_USE_SYSTEM_IMGUI)
  list(APPEND component_dependency_packages imgui)
  list(APPEND link_libraries imgui::imgui)
else()
  list(APPEND component_dependencies imgui)
  list(APPEND link_libraries dart-imgui-lib)
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

set(dart_gui_osg_hdrs ${hdrs} ${detail_hdrs})
set(dart_gui_osg_srcs ${srcs} ${detail_srcs})

add_subdirectory(render)

file(GLOB render_hdrs "render/*.hpp")

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
target_link_libraries(${target_name} dart-utils osg::osg ${link_libraries})

# Suppress deprecated warnings for files using backward-compatible API
# TODO: MeshShapeNode needs proper refactoring to use TriMesh API + material system - see mesh_branch_reimplementation_plan.md
set_source_files_properties(
  render/MeshShapeNode.cpp
  PROPERTIES
  COMPILE_FLAGS "-Wno-deprecated-declarations"
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} utils ${component_dependencies})
add_component_dependency_packages(${PROJECT_NAME} ${component_name} ${component_dependency_packages})

# Generate header for this namespace
dart_get_filename_components(header_names "gui osg headers" ${hdrs})
list(APPEND header_names "render/all.hpp")
dart_generate_component_headers(
  COMPONENT_NAME osg
  TARGET_DIR "dart/gui/osg/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs} ${detail_hdrs} ${render_hdrs}
)

# Install (includes all.hpp, osg.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/all.hpp ${CMAKE_CURRENT_BINARY_DIR}/osg.hpp
  DESTINATION include/dart/gui/osg
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/gui/osg/detail
  COMPONENT headers
)
# Install generated compat headers while preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/gui/osg
)

dart_format_add(${hdrs} ${srcs} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
