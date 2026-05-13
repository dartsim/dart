cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components gui utils io)
set(required_libraries dart dart-gui dart-io dart-utils)

if(DART_BUILD_COLLISION_BULLET)
  list(APPEND required_components collision-bullet)
  list(APPEND required_libraries dart-collision-bullet)
endif()

if(DART_BUILD_COLLISION_ODE)
  list(APPEND required_components collision-ode)
  list(APPEND required_libraries dart-collision-ode)
endif()

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  return()
endif()

find_package(DART 6.15.0 REQUIRED COMPONENTS ${required_components} CONFIG)
find_package(CLI11 2.4 CONFIG REQUIRED)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries} CLI11::CLI11)
target_include_directories(${example_name} PUBLIC ${DART_INCLUDE_DIRS})
