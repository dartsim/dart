# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License.

# Headless editor engine for the dartsim GUI simulator.
#
# This is an app-support static library, not a public dart:: component (the
# dartsim app is a runtime executable distribution, not a library). It is
# intentionally separate from dartsim/app/ because the application entry point
# is restricted to a single main.cpp by dart_gui_add_application; reusable code
# must live outside that directory. The engine has no dependency on dart-gui or
# any rendering backend (it stays backend-hidden). See
# docs/design/dartsim_gui_simulator.md.

set(target_name dartsim_engine)

add_library(
  ${target_name}
  STATIC
  src/scene_model.cpp
  src/name_manager.cpp
  src/object_manager.cpp
  src/selection_manager.cpp
  src/command.cpp
  src/commands.cpp
  src/simulation_controller.cpp
  src/recorder.cpp
  src/scene_io.cpp
  src/sim_engine.cpp
)

target_include_directories(
  ${target_name}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${target_name} PUBLIC dart-simulation Eigen3::Eigen)

# Allow linking into shared targets (e.g. the GUI app) later.
set_target_properties(${target_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Register sources with the repository formatter so `pixi run lint` covers them.
if(COMMAND dart_format_add)
  file(
    GLOB_RECURSE _dartsim_engine_format_files
    "${CMAKE_CURRENT_SOURCE_DIR}/include/dartsim_engine/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  )
  dart_format_add(${_dartsim_engine_format_files})
endif()
