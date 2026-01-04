cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components utils-urdf gui io)
set(required_libraries dart dart-io dart-utils-urdf dart-gui)

if(DART_IN_SOURCE_BUILD)
  file(GLOB srcs "*.cpp" "*.hpp" "robots/*.cpp" "robots/*.hpp")
  add_executable(${example_name} ${srcs})
  target_link_libraries(${example_name} ${required_libraries})
  set_target_properties(
    ${example_name}
    PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
  )
  dart_format_add(${srcs})
  dart_add_example(${example_name})
  return()
endif()

find_package(DART 7.0.0 REQUIRED COMPONENTS ${required_components} CONFIG)
find_package(CLI11 2.4 CONFIG REQUIRED)

file(GLOB srcs "*.cpp" "*.hpp" "robots/*.cpp" "robots/*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
target_link_libraries(${example_name} PRIVATE CLI11::CLI11)
