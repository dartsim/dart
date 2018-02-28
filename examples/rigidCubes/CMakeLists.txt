cmake_minimum_required(VERSION 2.8.12)

if(DART_IN_SOURCE_BUILD)
  include(${CMAKE_CURRENT_SOURCE_DIR}/InSourceBuild.cmake)
  return()
endif()

project(rigidCubes)

find_package(DART 6.2.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})
