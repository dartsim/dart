cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_libraries dart dart-gui-osg)

if(NOT TARGET Python3::Python)
  message(STATUS "Skipping ${example_name}: Python3 development target not found")
  return()
endif()

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(
    ${example_name}
    LINK_LIBRARIES ${required_libraries} Python3::Python
  )
  return()
endif()

find_package(DART 6.17 REQUIRED COMPONENTS gui-osg CONFIG)

# Python embedding is optional: skip the example gracefully when the Python
# development files are unavailable, mirroring the in-source guard above.
find_package(Python3 COMPONENTS Interpreter Development)
if(NOT Python3_FOUND)
  message(STATUS "Skipping ${example_name}: Python3 development files not found")
  return()
endif()

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries} Python3::Python)
