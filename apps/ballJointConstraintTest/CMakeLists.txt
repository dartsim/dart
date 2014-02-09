###############################################
# apps/ballJointConstraintTest
file(GLOB ballJointConstraintTest_srcs "*.cpp")
file(GLOB ballJointConstraintTest_hdrs "*.h")
add_executable(ballJointConstraintTest ${ballJointConstraintTest_srcs} ${ballJointConstraintTest_hdrs})
target_link_libraries(ballJointConstraintTest dart)
set_target_properties(ballJointConstraintTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
