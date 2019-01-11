cmake_minimum_required(VERSION 3.5.1)

if(DART_IN_SOURCE_BUILD)
  include(${CMAKE_CURRENT_SOURCE_DIR}/InSourceBuild.cmake)
  return()
endif()

project(osgWamIkfast)

find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf gui-osg)

add_subdirectory(ikfast)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${PROJECT_NAME} ${srcs})
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} wamIk)
