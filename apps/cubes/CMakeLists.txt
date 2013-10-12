###############################################
# apps/cubes
file(GLOB cubes_srcs "*.cpp")
file(GLOB cubes_hdrs "*.h")
add_executable(cubes ${cubes_srcs} ${cubes_hdrs})
target_link_libraries(cubes dart)
set_target_properties(cubes PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
