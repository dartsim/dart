###############################################
# apps/exampleCollisionWithSkelton

project(exampleCollisionWithSkelton)
file(GLOB collision_srcs "*.cpp")
file(GLOB collision_hdrs "*.h")
add_executable(exampleCollisionWithSkelton ${collision_srcs} ${collision_hdrs})
target_link_libraries(exampleCollisionWithSkelton lcpsolver collision kinematics dynamics yui renderer integration ccd fcl debug fcld ${DARTExt_LIBRARIES})
set_target_properties(exampleCollisionWithSkelton PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
