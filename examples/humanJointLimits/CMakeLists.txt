cmake_minimum_required(VERSION 3.5.1)

if(DART_IN_SOURCE_BUILD)
  include(${CMAKE_CURRENT_SOURCE_DIR}/InSourceBuild.cmake)
  return()
endif()

project(humanJointLimits)

find_package(DART 7.0 REQUIRED
  COMPONENTS
    io-urdf gui collision-ode collision-bullet
  CONFIG
)

set(TinyDNN_USE_SERIALIZER ON)
find_package(TinyDNN REQUIRED)

find_package(Threads REQUIRED)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs})
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    dart
    dart-utils-urdf
    dart-gui
    dart-collision-ode
    dart-collision-bullet
    ${TinyDNN_LIBRARIES}
)
target_compile_features(dart PUBLIC cxx_std_14)

# Thread
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(${example_name} PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(${example_name} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
endif()
