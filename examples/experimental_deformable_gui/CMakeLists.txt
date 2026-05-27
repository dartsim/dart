cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${example_name})

if(NOT DART_IN_SOURCE_BUILD)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake")
  find_package(DART 7.0.0 QUIET COMPONENTS gui simulation-experimental CONFIG)
endif()

if(NOT TARGET dart-gui OR NOT TARGET dart-simulation-experimental)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(
      STATUS
      "Skipping ${example_name} example (requires dart-gui and dart-simulation-experimental)"
    )
  endif()
  return()
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../gui_scene_example.cmake")
dart_build_gui_example(${example_name} main.cpp)

if(TARGET ${example_name})
  target_link_libraries(${example_name} PRIVATE dart-simulation-experimental)
endif()
