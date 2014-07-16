###############################################
# apps/meshCollision
file(GLOB meshCollision_srcs "*.cpp")
file(GLOB meshCollision_hdrs "*.h")
add_executable(meshCollision ${meshCollision_srcs} ${meshCollision_hdrs})
target_link_libraries(meshCollision dart)
set_target_properties(meshCollision PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
