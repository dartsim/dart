###############################################
# apps/rigidCubes
file(GLOB rigidCubes_srcs "*.cpp")
file(GLOB rigidCubes_hdrs "*.h")
add_executable(rigidCubes ${rigidCubes_srcs} ${rigidCubes_hdrs})
target_link_libraries(rigidCubes kido)
set_target_properties(rigidCubes PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
