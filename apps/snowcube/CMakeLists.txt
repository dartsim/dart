###############################################
# apps/snowcube

project(snowcube)
file(GLOB snowcube_srcs "*.cpp")
file(GLOB snowcube_hdrs "*.h")
add_executable(snowcube ${snowcube_srcs} ${snowcube_hdrs})
target_link_libraries(snowcube dart ${DARTExt_LIBRARIES})
set_target_properties(snowcube PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
