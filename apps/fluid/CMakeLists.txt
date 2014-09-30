###############################################
# apps/fluid

project(tinkertoy)
file(GLOB fluid_srcs "*.cpp")
file(GLOB fluid_hdrs "*.h")
add_executable(fluid ${fluid_srcs} ${fluid_hdrs})
target_link_libraries(fluid dart ${DARTExt_LIBRARIES})
set_target_properties(fluid PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
