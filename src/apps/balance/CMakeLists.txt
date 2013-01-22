###############################################
# apps/balance

project(balance)
file(GLOB balance_srcs "*.cpp")
file(GLOB balance_hdrs "*.h")
add_executable(balance ${balance_srcs} ${balance_hdrs})
target_link_libraries(balance kinematics dynamics yui renderer integration collision lcpsolver optimized fcl debug fcld ${DARTExt_LIBRARIES})

set_target_properties(balance PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
