###############################################
# apps/closedLoop
file(GLOB closedLoop_srcs "*.cpp")
file(GLOB closedLoop_hdrs "*.h")
add_executable(closedLoop ${closedLoop_srcs} ${closedLoop_hdrs})
target_link_libraries(closedLoop dart)
set_target_properties(closedLoop PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
