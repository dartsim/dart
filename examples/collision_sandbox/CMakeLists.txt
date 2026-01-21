cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

if(NOT DART_BUILD_GUI_VSG)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Skipping ${example_name} example (requires DART_BUILD_GUI_VSG=ON)")
  endif()
  return()
endif()

if(NOT DART_BUILD_COLLISION_EXPERIMENTAL)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Skipping ${example_name} example (requires DART_BUILD_COLLISION_EXPERIMENTAL=ON)")
  endif()
  return()
endif()

if(NOT TARGET vsgImGui AND NOT TARGET vsgImGui::vsgImGui)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Skipping ${example_name} example (requires vsgImGui)")
  endif()
  return()
endif()

find_package(CLI11 2.4 CONFIG REQUIRED)

set(required_libraries dart-gui-vsg dart-collision-experimental)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries} CLI11::CLI11)
  return()
endif()

find_package(DART 7.0.0 REQUIRED CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries} CLI11::CLI11)
