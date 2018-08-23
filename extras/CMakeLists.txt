include_directories("${CMAKE_CURRENT_SOURCE_DIR}/dartc/include")

add_subdirectory(dartc/src)
add_subdirectory(tests)

file(GLOB_RECURSE dartc_headers "${CMAKE_CURRENT_SOURCE_DIR}/dartc/include/*.h")
dart_format_add(${dartc_headers})
