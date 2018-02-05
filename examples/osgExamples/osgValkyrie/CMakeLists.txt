cmake_minimum_required(VERSION 2.8.12)

project(valyrie)

find_package(DART 6.4.0 REQUIRED COMPONENTS utils-urdf gui gui-osg CONFIG)

add_compile_options(-std=c++11)

file(GLOB srcs "*.cpp" "*.hpp")

SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)

add_executable(${PROJECT_NAME} ${srcs})
target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})
