###############################################
# apps/hybridDynamics2
file(GLOB hybridDynamics2_srcs "*.cpp")
file(GLOB hybridDynamics2_hdrs "*.h")
add_executable(hybridDynamics2 ${hybridDynamics2_srcs} ${hybridDynamics2_hdrs})
target_link_libraries(hybridDynamics2 dart)
set_target_properties(hybridDynamics2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
