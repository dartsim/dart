cmake_minimum_required(VERSION 2.8.12)

if(DART_IN_SOURCE_BUILD)
  include(${CMAKE_CURRENT_SOURCE_DIR}/InSourceBuild.cmake)
  return()
endif()

project(tutorialBiped)

find_package(DART 6.6.0 REQUIRED
  COMPONENTS collision-bullet utils-urdf gui
  CONFIG
)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs})
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    dart dart-collision-bullet dart-utils-urdf dart-gui
)
