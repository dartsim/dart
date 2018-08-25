include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_subdirectory(src)
add_subdirectory(tests)

file(GLOB_RECURSE dartc_headers "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
dart_format_add(${dartc_headers})
