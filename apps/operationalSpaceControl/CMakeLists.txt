###############################################
# apps/operationalSpaceControl
file(GLOB operationalSpaceControl_srcs "*.cpp")
file(GLOB operationalSpaceControl_hdrs "*.hpp")
add_executable(operationalSpaceControl ${operationalSpaceControl_srcs} ${operationalSpaceControl_hdrs})
target_link_libraries(operationalSpaceControl dart dart-gui dart-utils-urdf)
set_target_properties(operationalSpaceControl PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
