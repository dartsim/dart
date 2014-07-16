###############################################
# apps/forwardSim
file(GLOB forwardSim_srcs "*.cpp")
file(GLOB forwardSim_hdrs "*.h")
add_executable(forwardSim ${forwardSim_srcs} ${forwardSim_hdrs})
target_link_libraries(forwardSim dart)
set_target_properties(forwardSim PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
