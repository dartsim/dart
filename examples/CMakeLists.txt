cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

add_subdirectory(unified_loading)
if(TARGET dart-gui-experimental OR NOT DART_IN_SOURCE_BUILD)
  add_subdirectory(gui_scene_diagnostics)
endif()
add_subdirectory(headless_simulation)
add_subdirectory(speed_test)
add_subdirectory(csv_logger)
add_subdirectory(rerun)
