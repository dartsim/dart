cmake_minimum_required(VERSION 3.10.2)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

add_subdirectory(ikfast)

set(required_components utils-urdf gui-osg)
set(required_libraries dart dart-utils-urdf dart-gui-osg wamIk)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  return()
endif()

find_package(DART 6.6.0 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
target_link_libraries(${PROJECT_NAME} PUBLIC wamIk)
