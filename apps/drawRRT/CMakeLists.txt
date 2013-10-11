###############################################
# apps/drawRRT

file(GLOB drawRRT_srcs "*.cpp")
file(GLOB drawRRT_hdrs "*.h")
add_executable(drawRRT ${drawRRT_srcs} ${drawRRT_hdrs})
target_link_libraries(drawRRT dart)
set_target_properties(drawRRT PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
