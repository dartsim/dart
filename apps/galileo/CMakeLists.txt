###############################################
# apps/Galileo

project(galileo)
file(GLOB galileo_srcs "*.cpp")
file(GLOB galileo_hdrs "*.h")
add_executable(galileo ${galileo_srcs} ${galileo_hdrs})
target_link_libraries(galileo dart ${DARTExt_LIBRARIES})
set_target_properties(galileo PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
