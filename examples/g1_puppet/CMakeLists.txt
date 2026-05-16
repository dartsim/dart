cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${example_name})

include("${CMAKE_CURRENT_LIST_DIR}/../gui_scene_example.cmake")
dart_build_gui_example(${example_name} "${CMAKE_CURRENT_LIST_DIR}/main.cpp")
if(TARGET ${example_name})
  target_link_libraries(${example_name} PRIVATE dart-io dart-utils)
endif()
