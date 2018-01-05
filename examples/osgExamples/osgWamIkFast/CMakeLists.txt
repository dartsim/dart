cmake_minimum_required(VERSION 2.8.12)

project(osgWamIkfast)

find_package(DART 6.1.1 REQUIRED COMPONENTS io-urdf gui-osg external-ikfast)

add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS})

add_subdirectory(ikfast)

set(srcs osgWamIkfast.cpp)
add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} wamIkfast)
