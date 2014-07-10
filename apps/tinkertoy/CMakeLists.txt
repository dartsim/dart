###############################################
# apps/tinkertoy

project(tinkertoy)
file(GLOB tinkertoy_srcs "*.cpp")
file(GLOB tinkertoy_hdrs "*.h")
add_executable(tinkertoy ${tinkertoy_srcs} ${tinkertoy_hdrs})
target_link_libraries(tinkertoy dart ${DARTExt_LIBRARIES})
set_target_properties(tinkertoy PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
