###############################################
# apps/sdfLoader
project(sdfLoader)
file(GLOB sdfLoader_srcs "*.cpp")
file(GLOB sdfLoader_hdrs "*.h")
add_executable(sdfLoader ${sdfLoader_srcs} ${sdfLoader_hdrs})
target_link_libraries(sdfLoader dart ${DARTExt_LIBRARIES})
set_target_properties(sdfLoader PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
