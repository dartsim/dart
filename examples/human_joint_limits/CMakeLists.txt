cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(TinyDNN_USE_SERIALIZER ON)
find_package(TinyDNN QUIET)
if(NOT TinyDNN_FOUND)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Failed to find TinyDNN. ${example_name} is disabled.")
  endif()
  return()
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads QUIET)
if(NOT Threads_FOUND)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Failed to find Threads. ${example_name} is disabled.")
  endif()
  return()
endif()

set(missing_collision_features)
if(NOT DART_BUILD_COLLISION_ODE)
  list(APPEND missing_collision_features "DART_BUILD_COLLISION_ODE")
endif()
if(NOT DART_BUILD_COLLISION_BULLET)
  list(APPEND missing_collision_features "DART_BUILD_COLLISION_BULLET")
endif()

if(missing_collision_features)
  list(JOIN missing_collision_features ", " _missing_collision_features_string)
  set(_human_limits_message "${example_name} requires ${_missing_collision_features_string}.")
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Skipping ${example_name} example (${_human_limits_message})")
  endif()
  return()
endif()

set(required_components utils-urdf gui io)
set(
  required_libraries
  dart
  dart-io
  dart-utils-urdf
  dart-gui
  ${TinyDNN_LIBRARIES}
  Threads::Threads
)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name}
    LINK_LIBRARIES ${required_libraries}
    COMPILE_FEATURES cxx_std_20
  )
  return()
endif()

find_package(DART 7.0.0 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
target_compile_features(${example_name} PUBLIC cxx_std_20)
