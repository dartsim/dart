###############################################
# apps/tutorial

project(tutorial)
file(GLOB tutorial_srcs "*.cpp")
file(GLOB tutorial_hdrs "*.h")
add_executable(tutorial ${tutorial_srcs} ${tutorial_hdrs})
target_link_libraries(tutorial dart ${DARTExt_LIBRARIES})
set_target_properties(tutorial PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
