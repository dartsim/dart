###############################################
# apps/jointConstraints
file(GLOB jointConstraints_srcs "*.cpp")
file(GLOB jointConstraints_hdrs "*.hpp")
add_executable(jointConstraints ${jointConstraints_srcs} ${jointConstraints_hdrs})
target_link_libraries(jointConstraints kido kido-gui)
set_target_properties(jointConstraints PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
