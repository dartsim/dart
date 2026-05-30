cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

include("${CMAKE_CURRENT_LIST_DIR}/gui_scene_example.cmake")

add_subdirectory(unified_loading)
if(TARGET dart-gui OR NOT DART_IN_SOURCE_BUILD)
  add_subdirectory(gui_scene_diagnostics)
endif()
add_subdirectory(headless_simulation)
add_subdirectory(speed_test)
add_subdirectory(csv_logger)

if(DART_BUILD_GUI OR NOT DART_IN_SOURCE_BUILD)
  # hello_world stays standalone as the minimal "link DART in a CMake project"
  # template. Every other GUI example is a registered scene in the
  # dart-demos app (examples/demos), switchable at runtime from the sidebar.
  add_subdirectory(hello_world)
  add_subdirectory(demos)

  # experimental_differentiable_gui stays standalone: it is special-build-coupled
  # on the opt-in differentiable build (DART_BUILD_DIFF / DART_HAS_DIFF), which is
  # off in the default build that compiles dart-demos. Its own CMakeLists returns
  # early when GUI, experimental simulation, or DART_HAS_DIFF are unavailable.
  add_subdirectory(experimental_differentiable_gui)
endif()
