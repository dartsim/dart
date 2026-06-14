cmake_minimum_required(VERSION 4.2.3)

project(dart-examples)

include("${CMAKE_CURRENT_LIST_DIR}/gui_scene_example.cmake")

if(DART_BUILD_GUI OR NOT DART_IN_SOURCE_BUILD)
  add_subdirectory(demos)

  # differentiable_gui stays standalone: it is special-build-coupled
  # on the opt-in differentiable build (DART_BUILD_DIFF / DART_HAS_DIFF), which is
  # off in the default build that compiles dart-demos. Its own CMakeLists returns
  # early when GUI, simulation, or DART_HAS_DIFF are unavailable.
  add_subdirectory(differentiable_gui)
endif()
