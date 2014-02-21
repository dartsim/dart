###############################################
# apps/atlasRobot
file(GLOB atlasRobot_srcs "*.cpp")
file(GLOB atlasRobot_hdrs "*.h")
add_executable(atlasRobot ${atlasRobot_srcs} ${atlasRobot_hdrs})
target_link_libraries(atlasRobot dart)
set_target_properties(atlasRobot PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
