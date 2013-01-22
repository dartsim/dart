###############################################
# apps/forwardSim

project(forwardSim)
file(GLOB forwardSim_srcs "*.cpp")
file(GLOB forwardSim_hdrs "*.h")
add_executable(forwardSim ${forwardSim_srcs} ${forwardSim_hdrs})
target_link_libraries(forwardSim kinematics dynamics integration yui renderer ${DARTExt_LIBRARIES})
# kinematics dynamics yui renderer integration optimized fcl debug fcld ${DARTExt_LIBRARIES}
set_target_properties(forwardSim PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
