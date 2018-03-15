cmake_minimum_required(VERSION 2.8.12)

if(DART_IN_SOURCE_BUILD)
  include(${CMAKE_CURRENT_SOURCE_DIR}/InSourceBuild.cmake)
  return()
endif()

project(humanJointLimits)

find_package(DART 6.4 REQUIRED
  COMPONENTS
    dart-utils-urdf dart-gui dart-collision-ode dart-collision-bullet
  CONFIG
)

set(TinyDNN_USE_SERIALIZER ON)
find_package(TinyDNN REQUIRED)

find_package(Threads REQUIRED)

target_compile_options(${PROJECT_NAME} PUBLIC -std=c++14)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} ${TinyDNN_LIBRARIES})

# Thread
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(${example_name} PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(${example_name} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
endif()
