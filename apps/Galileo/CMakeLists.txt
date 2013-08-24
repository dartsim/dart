###############################################
# apps/Galileo

project(Galileo)
file(GLOB Galileo_srcs "*.cpp")
file(GLOB Galileo_hdrs "*.h")
add_executable(Galileo ${Galileo_srcs} ${Galileo_hdrs})
target_link_libraries(Galileo dart ${DARTExt_LIBRARIES})
set_target_properties(Galileo PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
