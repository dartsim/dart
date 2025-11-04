cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

# Aggregate dependencies
set(required_components gui-osg)
set(required_libraries dart dart-gui-osg)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  return()
endif()

find_package(DART 6.15.0 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
target_include_directories(${example_name} PUBLIC ${DART_INCLUDE_DIRS})

option(DART_COUPLER_CONSTRAINT_ENABLE_GUI "Launch the coupler constraint GUI example" OFF)
if(DART_COUPLER_CONSTRAINT_ENABLE_GUI)
  message(STATUS "${example_name}: GUI support enabled")
else()
  message(STATUS "${example_name}: building in headless mode (set DART_COUPLER_CONSTRAINT_ENABLE_GUI=ON to enable viewer)")
  target_compile_definitions(${example_name} PRIVATE DART_COUPLER_HEADLESS_DEFAULT)
endif()
