# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License.

# GUI layer for the dartsim simulator: wires the headless editor engine
# (dartsim/engine) to the backend-hidden GUI panels exposed by dart-gui.
# Kept as a separate library because dartsim/app is restricted to a single
# main.cpp by dart_gui_add_application. See
# docs/design/dartsim_gui_simulator.md.

set(target_name dartsim_ui)

find_package(nfd CONFIG REQUIRED)

add_library(
  ${target_name}
  STATIC
  src/console_actions.cpp
  src/editor.cpp
  src/history_actions.cpp
  src/inspector_actions.cpp
  src/memory_actions.cpp
  src/outliner_actions.cpp
  src/palette_actions.cpp
  src/project_actions.cpp
  src/project_file_dialog.cpp
  src/relationship_actions.cpp
  src/simulation_actions.cpp
  src/watch_actions.cpp
  src/viewport_actions.cpp
)

target_include_directories(
  ${target_name}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(
  ${target_name}
  PUBLIC dartsim_engine dart-gui
  PRIVATE nfd::nfd
)

set_target_properties(${target_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(COMMAND dart_format_add)
  file(
    GLOB_RECURSE _dartsim_ui_format_files
    "${CMAKE_CURRENT_SOURCE_DIR}/include/dartsim_ui/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  )
  dart_format_add(${_dartsim_ui_format_files})
endif()
