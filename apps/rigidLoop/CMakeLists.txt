###############################################
# apps/rigidLoop
file(GLOB rigidLoop_srcs "*.cpp")
file(GLOB rigidLoop_hdrs "*.h")
add_executable(rigidLoop ${rigidLoop_srcs} ${rigidLoop_hdrs})
target_link_libraries(rigidLoop kido)
set_target_properties(rigidLoop PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
