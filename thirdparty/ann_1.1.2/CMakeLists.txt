cmake_minimum_required(VERSION 2.6)
project (ann)

file (GLOB srcs "src/*.cpp")
file (GLOB hdrs "include/ANN/*.h")

include_directories (include)
add_library (ann ${srcs})
set_target_properties (ann PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/../lib")

install (FILES ${hdrs} DESTINATION "${CMAKE_SOURCE_DIR}/../include/ann")
install (TARGETS ann ARCHIVE DESTINATION "${CMAKE_SOURCE_DIR}/../lib")
