###############################################
# apps/jointConstraints
file(GLOB jointConstraints_srcs "*.cpp")
file(GLOB jointConstraints_hdrs "*.h")
add_executable(jointConstraints ${jointConstraints_srcs} ${jointConstraints_hdrs})
target_link_libraries(jointConstraints dart dart-gui)
set_target_properties(jointConstraints PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
