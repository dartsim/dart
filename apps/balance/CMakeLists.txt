###############################################
# apps/balance
project(balance)
file(GLOB balance_srcs "*.cpp")
file(GLOB balance_hdrs "*.h")
add_executable(balance ${balance_srcs} ${balance_hdrs})
target_link_libraries(balance dart ${DART_DEPENDENCIES})
set_target_properties(balance PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
