###############################################
# apps/constraintTest
file(GLOB constraintTest_srcs "*.cpp")
file(GLOB constraintTest_hdrs "*.h")
add_executable(constraintTest ${constraintTest_srcs} ${constraintTest_hdrs})
target_link_libraries(constraintTest dart)
set_target_properties(constraintTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
