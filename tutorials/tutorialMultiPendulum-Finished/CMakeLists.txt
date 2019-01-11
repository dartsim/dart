cmake_minimum_required(VERSION 3.5.1)

if(DART_IN_SOURCE_BUILD)
  include(${CMAKE_CURRENT_SOURCE_DIR}/InSourceBuild.cmake)
  return()
endif()

project(tutorialMultiPendulum-Finished)

find_package(DART 7 REQUIRED COMPONENTS io-urdf gui CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs})
target_link_libraries(${PROJECT_NAME} PUBLIC dart dart-utils-urdf dart-gui)
