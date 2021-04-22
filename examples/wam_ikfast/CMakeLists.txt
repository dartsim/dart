# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

add_subdirectory(ikfast)

set(required_components io gui)
set(required_libraries dart7-io dart7-gui wamIk)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  return()
endif()

find_package(DART7 7.0.0 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
target_link_libraries(${PROJECT_NAME} PUBLIC wamIk)
