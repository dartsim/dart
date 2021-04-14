cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(TinyDNN_USE_SERIALIZER ON)
find_package(TinyDNN QUIET)
if(NOT TinyDNN_FOUND)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Failed to find TinyDNN. glut_human_joint_limits is disabled.")
  endif()
  return()
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads QUIET)
if(NOT Threads_FOUND)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Failed to find Threads. glut_human_joint_limits is disabled.")
  endif()
  return()
endif()

set(required_components utils-urdf gui collision-ode collision-bullet)
set(required_libraries
  dart
  dart-utils-urdf
  dart-gui
  dart-collision-ode
  dart-collision-bullet
  ${TinyDNN_LIBRARIES}
  Threads::Threads
)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name}
    LINK_LIBRARIES ${required_libraries}
    COMPILE_FEATURES cxx_std_14
  )
  return()
endif()

find_package(DART 7.0.0 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
target_compile_features(dart PUBLIC cxx_std_14)
