###############################################
# apps/jointLimitTest
file(GLOB jointLimitTest_srcs "*.cpp")
file(GLOB jointLimitTest_hdrs "*.h")
add_executable(jointLimitTest ${jointLimitTest_srcs} ${jointLimitTest_hdrs})
target_link_libraries(jointLimitTest dart)
set_target_properties(jointLimitTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
