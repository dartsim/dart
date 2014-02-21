###############################################
# apps/harnessTest
file(GLOB harnessTest_srcs "*.cpp")
file(GLOB harnessTest_hdrs "*.h")
add_executable(harnessTest ${harnessTest_srcs} ${harnessTest_hdrs})
target_link_libraries(harnessTest dart)
set_target_properties(harnessTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
