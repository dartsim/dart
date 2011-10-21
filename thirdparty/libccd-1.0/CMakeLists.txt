cmake_minimum_required(VERSION 2.6)
project (libccd)

file (GLOB srcs "src/*.c")
file (GLOB hdrs "src/ccd/*.h")

include_directories (src)
add_library (ccd ${srcs})
set_target_properties (ccd PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/../lib")

install (FILES ${hdrs} DESTINATION "${CMAKE_SOURCE_DIR}/../include/ccd")
install (TARGETS ccd ARCHIVE DESTINATION "${CMAKE_SOURCE_DIR}/../lib")
