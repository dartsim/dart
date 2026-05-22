cmake_minimum_required(VERSION 3.22.1)

get_filename_component(application_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${application_name})

set(application_target dartsim)

if(DART_IN_SOURCE_BUILD AND NOT DART_BUILD_GUI)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(
      STATUS
      "Skipping ${application_name} application (requires DART_BUILD_GUI=ON)"
    )
  endif()
  return()
endif()

if(NOT DART_IN_SOURCE_BUILD)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")
  find_package(DART 7.0.0 QUIET COMPONENTS gui CONFIG)
  if(NOT DART_FOUND)
    find_package(DART 7.0.0 REQUIRED COMPONENTS gui CONFIG)
  endif()
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/detail/backend_sources.cmake")

dart_gui_add_application(
  ${application_target}
  GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated"
  OUTPUT_NAME "dartsim"
  SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}"
)
