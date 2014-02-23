###############################################
# apps/rigidBodies

project(rigidBodies)
file(GLOB rigidBodies_srcs "*.cpp")
file(GLOB rigidBodies_hdrs "*.h")
include_directories(../../src/dynamics)
add_executable(rigidBodies ${rigidBodies_srcs} ${rigidBodies_hdrs})
target_link_libraries(rigidBodies dart ${DARTExt_LIBRARIES})
set_target_properties(rigidBodies PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
