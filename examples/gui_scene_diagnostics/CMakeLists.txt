cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components gui-experimental)
set(required_libraries dart dart-gui-experimental)

if(DART_IN_SOURCE_BUILD)
  if(NOT TARGET dart-gui-experimental)
    if(DART_VERBOSE)
      message(STATUS "Skipping ${example_name} example (dart-gui-experimental unavailable)")
    endif()
    return()
  endif()

  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  return()
endif()

find_package(DART 7.0.0 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
