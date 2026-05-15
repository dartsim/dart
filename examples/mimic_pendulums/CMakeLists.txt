cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${example_name})

include("${CMAKE_CURRENT_LIST_DIR}/../gui_scene_example.cmake")
dart_build_gui_scene_example(${example_name} "mimic-pendulums")
