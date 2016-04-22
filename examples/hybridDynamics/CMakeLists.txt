###############################################
# apps/hybridDynamics
file(GLOB hybridDynamics_srcs "*.cpp")
file(GLOB hybridDynamics_hdrs "*.h")
add_executable(hybridDynamics ${hybridDynamics_srcs} ${hybridDynamics_hdrs})
target_link_libraries(hybridDynamics dart dart-gui)
set_target_properties(hybridDynamics PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
