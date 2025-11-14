cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

if(NOT TARGET dart-utils-usd)
  return()
endif()

set(required_components utils-usd gui-osg)
set(required_libraries dart dart-utils-usd dart-gui-osg)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  return()
endif()

find_package(DART 6.16.0 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
