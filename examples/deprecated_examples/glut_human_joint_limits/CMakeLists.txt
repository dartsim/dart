cmake_minimum_required(VERSION 3.5.1)

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
)

if(DART_IN_SOURCE_BUILD)
  if(THREADS_HAVE_PTHREAD_ARG)
    dart_build_example_in_source(${example_name}
      LINK_LIBRARIES ${required_libraries}
      COMPILE_OPTIONS "-pthread"
      COMPILE_FEATURES cxx_std_14
    )
  elseif(CMAKE_THREAD_LIBS_INIT)
    dart_build_example_in_source(${example_name}
      LINK_LIBRARIES ${required_libraries} ${CMAKE_THREAD_LIBS_INIT}
      COMPILE_OPTIONS "-pthread"
      COMPILE_FEATURES cxx_std_14
    )
  endif()

  return()
endif()

find_package(DART 6.6.0 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
target_compile_features(dart PUBLIC cxx_std_14)

# Thread
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(${example_name} PUBLIC "-pthread")
elseif(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(${example_name} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
endif()
