###############################################
# apps/rigidChain
file(GLOB rigidChain_srcs "*.cpp")
file(GLOB rigidChain_hdrs "*.hpp")
add_executable(rigidChain ${rigidChain_srcs} ${rigidChain_hdrs})
target_link_libraries(rigidChain kido kido-gui)
set_target_properties(rigidChain PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
