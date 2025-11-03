cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components gui-osg utils-urdf)
set(required_libraries dart dart-gui-osg dart-utils-urdf)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS
  "app/*.cpp"
  "core/*.cpp"
  "registry/*.cpp"
  "scenes/*.cpp")

if(DART_IN_SOURCE_BUILD)
  add_executable(${example_name} ${srcs})
  target_include_directories(${example_name} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/app
    ${CMAKE_CURRENT_LIST_DIR}/core
    ${CMAKE_CURRENT_LIST_DIR}/registry
    ${CMAKE_CURRENT_LIST_DIR}/scenes)
  target_link_libraries(${example_name} PUBLIC ${required_libraries})
  set_target_properties(${example_name}
    PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin")
  dart_add_example(${example_name})
  return()
endif()

find_package(DART 6.15.0 REQUIRED COMPONENTS ${required_components} CONFIG)

add_executable(${example_name} ${srcs})
target_include_directories(${example_name} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/app
  ${CMAKE_CURRENT_LIST_DIR}/core
  ${CMAKE_CURRENT_LIST_DIR}/registry
  ${CMAKE_CURRENT_LIST_DIR}/scenes)
target_link_libraries(${example_name} PUBLIC ${required_libraries})
