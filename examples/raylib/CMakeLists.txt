cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(example_target dart_${example_name})

set(required_components dart gui)
set(required_libraries dart-gui dart-gui-raylib)

if(DART_IN_SOURCE_BUILD AND NOT DART_BUILD_GUI_RAYLIB)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Skipping ${example_name} example (requires DART_BUILD_GUI_RAYLIB=ON)")
  endif()
  return()
endif()

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_target} LINK_LIBRARIES ${required_libraries})
  set_target_properties(${example_target} PROPERTIES OUTPUT_NAME ${example_name})
  return()
endif()

find_package(DART 7.0.0 REQUIRED COMPONENTS ${required_components} CONFIG)
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
add_executable(${example_target} ${srcs})
set_target_properties(${example_target} PROPERTIES OUTPUT_NAME ${example_name})
target_link_libraries(${example_target} PUBLIC ${required_libraries})
