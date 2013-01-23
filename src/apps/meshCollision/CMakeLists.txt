###############################################
# apps/meshCollision
project(meshCollision)
file(GLOB meshCollision_srcs "*.cpp")
file(GLOB meshCollision_hdrs "*.h")
add_executable(meshCollision ${meshCollision_srcs} ${meshCollision_hdrs})
target_link_libraries(meshCollision lcpsolver collision kinematics dynamics yui renderer integration assimp optimized fcl debug fcld ${DARTExt_LIBRARIES})
set_target_properties(meshCollision PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
