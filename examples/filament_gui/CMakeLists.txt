cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(example_target dart_${example_name})

if(DART_IN_SOURCE_BUILD AND NOT DART_BUILD_GUI_FILAMENT)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Skipping ${example_name} example (requires DART_BUILD_GUI_FILAMENT=ON)")
  endif()
  return()
endif()

if(NOT DART_IN_SOURCE_BUILD)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")
  find_package(DART 7.0.0 QUIET COMPONENTS io utils-urdf gui-experimental imgui CONFIG)
  if(NOT DART_FOUND)
    find_package(DART 7.0.0 REQUIRED COMPONENTS io utils-urdf gui-experimental CONFIG)
  endif()
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/filament_sources.cmake")

dart_filament_gui_add_example(
  ${example_target}
  GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated"
  OUTPUT_NAME "${example_name}"
  SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}"
)
