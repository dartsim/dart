if(NOT DART_BUILD_GUI)
  message(STATUS "Skipping GUI (DART_BUILD_GUI == ${DART_BUILD_GUI})")
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

set(dart_gui_hdrs ${hdrs} ${detail_hdrs})
set(dart_gui_srcs ${srcs} ${detail_srcs})

add_subdirectory(render)

file(GLOB render_hdrs "render/*.hpp")

if(DART_BUILD_GUI_RAYLIB)
  add_subdirectory(raylib)
endif()

# Add target
dart_add_library(${target_name} ${dart_gui_hdrs} ${dart_gui_srcs})
target_link_libraries(${target_name} dart-utils osg::osg ${link_libraries})

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
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} utils ${component_dependencies})
add_component_dependency_packages(${PROJECT_NAME} ${component_name} ${component_dependency_packages})

# Generate header for this namespace
dart_get_filename_components(header_names "gui headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME gui
  TARGET_DIR "dart/gui/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs} ${detail_hdrs} ${render_hdrs}
)

# Install (includes gui.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/gui.hpp ${CMAKE_CURRENT_BINARY_DIR}/All.hpp
  DESTINATION include/dart/gui
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/gui/detail
  COMPONENT headers
)
# Install generated compatibility headers preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/gui
)

dart_format_add(${hdrs} ${srcs} ${dart_gui_hdrs} ${dart_gui_srcs})
