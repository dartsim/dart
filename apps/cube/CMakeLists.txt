###############################################
# apps/cube
file(GLOB cube_srcs "*.cpp")
file(GLOB cube_hdrs "*.h")
add_executable(cube ${cube_srcs} ${cube_hdrs})
target_link_libraries(cube dart)
set_target_properties(cube PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
