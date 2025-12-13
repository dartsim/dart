cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components dart)
set(required_libraries dart raylib::raylib)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  return()
endif()

find_package(DART 6.16.0 REQUIRED COMPONENTS ${required_components} CONFIG)
find_package(raylib CONFIG REQUIRED)

if(NOT TARGET raylib::raylib)
  if(TARGET raylib)
    add_library(raylib::raylib ALIAS raylib)
  endif()
endif()

if(NOT TARGET raylib::raylib)
  message(FATAL_ERROR "raylib was found but no CMake target was provided by the package.")
endif()

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
