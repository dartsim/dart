if(NOT DART_BUILD_GUI)
  message(STATUS "Skipping OpenSceneGraph (DART_BUILD_GUI == ${DART_BUILD_GUI})")
  return()
endif()

# Set local target/component names
set(target_name ${PROJECT_NAME}-gui)
set(component_name gui)
set(component_dependencies )
set(component_dependency_packages )
set(link_libraries )

# Dependency checks
dart_check_dependent_target(${target_name} dart-utils)

# OpenSceneGraph
dart_find_package(OpenSceneGraph)
dart_check_required_package(OpenSceneGraph "dart-gui" "OpenSceneGraph")
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
set(component_header_extras render/All.hpp)

add_subdirectory(render)

file(GLOB render_hdrs "render/*.hpp")

# Add target
dart_add_library(
  NAME ${target_name}
  HEADERS ${dart_gui_osg_hdrs}
  SOURCES ${dart_gui_osg_srcs}
  PUBLIC_LINK_LIBRARIES dart-utils osg::osg ${link_libraries}
)

if(MSVC AND BUILD_SHARED_LIBS)
  if(CMAKE_CONFIGURATION_TYPES)
    foreach(_cfg ${CMAKE_CONFIGURATION_TYPES})
      string(TOUPPER "${_cfg}" _cfg_upper)
      set_target_properties(
        ${target_name}
        PROPERTIES
          ARCHIVE_OUTPUT_DIRECTORY_${_cfg_upper}
            "${DART_BINARY_DIR}/lib/${_cfg}"
      )
    endforeach()
  else()
    set_target_properties(
      ${target_name}
      PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}"
    )
  endif()
endif()

# Component
dart_add_component(
  NAME ${component_name}
  PACKAGE ${PROJECT_NAME}
  TARGETS ${target_name}
  DEPENDENCIES utils ${component_dependencies}
  DEPENDENCY_PACKAGES ${component_dependency_packages}
  COMPONENT_HEADERS ${hdrs} ${detail_hdrs}
  COMPONENT_HEADER_EXTRAS ${component_header_extras}
  COMPONENT_HEADER_SOURCES ${hdrs} ${detail_hdrs} ${render_hdrs}
  COMPONENT_TARGET_DIR "dart/gui/osg/"
  COMPONENT_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
)

dart_format_add(${hdrs} ${srcs} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
