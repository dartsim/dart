cmake_minimum_required(VERSION 2.8.12)

project(wamIkfast)

find_package(DART 6.1.1 REQUIRED COMPONENTS external-ikfast)

add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS})

add_library(wamIk ikfast71.Transform6D.4_6_9_10_11_12_f8.cpp)
target_link_libraries(wamIk ${DART_LIBRARIES})
target_compile_definitions(wamIk PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
target_compile_options(wamIk PRIVATE -w)
