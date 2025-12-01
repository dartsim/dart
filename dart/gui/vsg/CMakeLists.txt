if(NOT DART_BUILD_GUI_VSG)
  message(
    STATUS
    "Skipping VulkanSceneGraph GUI (DART_BUILD_GUI_VSG == ${DART_BUILD_GUI_VSG})")
  return()
endif()

set(target_name ${PROJECT_NAME}-gui-vsg)
set(component_name gui-vsg)
set(component_dependencies )
set(component_dependency_packages )

dart_check_dependent_target(${target_name} dart-utils)

# VulkanSceneGraph
dart_find_package(VulkanSceneGraph)
dart_check_required_package(
  VulkanSceneGraph
  "dart-gui-vsg"
  "VulkanSceneGraph")
list(APPEND component_dependency_packages VulkanSceneGraph)

file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_vsg_hdrs ${hdrs})
set(dart_gui_vsg_srcs ${srcs})

dart_add_library(${target_name} ${dart_gui_vsg_hdrs} ${dart_gui_vsg_srcs})
target_link_libraries(${target_name} dart-utils vsg::vsg)

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
add_component_dependencies(
  ${PROJECT_NAME} ${component_name} utils ${component_dependencies})
add_component_dependency_packages(
  ${PROJECT_NAME} ${component_name} ${component_dependency_packages})

dart_get_filename_components(header_names "gui vsg headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME vsg
  TARGET_DIR "dart/gui/vsg/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/vsg.hpp
  DESTINATION include/dart/gui/vsg
  COMPONENT headers
)

dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/gui/vsg
)

dart_format_add(${hdrs} ${srcs})
