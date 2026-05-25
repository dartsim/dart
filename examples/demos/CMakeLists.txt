cmake_minimum_required(VERSION 3.22.1)

project(dart-demos)

include("${CMAKE_CURRENT_LIST_DIR}/../gui_scene_example.cmake")

# Absolute source paths so the formatter and add_executable resolve regardless
# of how dart_build_gui_example resolves relative paths.
set(dart_demos_sources
  "${CMAKE_CURRENT_LIST_DIR}/app/main.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/registry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/hello_world.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/shapes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/boxes.cpp"
)

dart_build_gui_example(dart-demos ${dart_demos_sources})

if(TARGET dart-demos)
  target_include_directories(dart-demos PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
endif()
