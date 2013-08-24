###############################################
# apps/freethrow

project(freethrow)
file(GLOB freethrow_srcs "*.cpp")
file(GLOB freethrow_hdrs "*.h")
add_executable(freethrow ${freethrow_srcs} ${freethrow_hdrs})
target_link_libraries(freethrow dart ${DARTExt_LIBRARIES})
set_target_properties(freethrow PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
