###############################################
# apps/hanging
file(GLOB hanging_srcs "*.cpp")
file(GLOB hanging_hdrs "*.h")
add_executable(hanging ${hanging_srcs} ${hanging_hdrs})
target_link_libraries(hanging dart)
set_target_properties(hanging PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
