cmake_minimum_required(VERSION 2.6)
project (fcl)

file (GLOB srcs "src/*.cpp")
file (GLOB hdrs "include/fcl/*.h")

set (CMAKE_CXX_FLAGS "-DUSE_PQP=0 -DUSE_SVMLIGHT=0")
include_directories (include ../include)
link_directories (fcl "${CMAKE_SOURCE_DIR}/../lib")
add_library (fcl ${srcs})
set_target_properties (fcl PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/../lib")

install (FILES ${hdrs} DESTINATION "${CMAKE_SOURCE_DIR}/../include/fcl")
install (TARGETS fcl ARCHIVE DESTINATION "${CMAKE_SOURCE_DIR}/../lib")

