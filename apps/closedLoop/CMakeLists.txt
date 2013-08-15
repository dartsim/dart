###############################################
# apps/closedLoop
PROJECT(closedLoop)
FILE(GLOB closedLoop_srcs "*.cpp")
FILE(GLOB closedLoop_hdrs "*.h")
ADD_EXECUTABLE(closedLoop ${closedLoop_srcs} ${closedLoop_hdrs})
target_link_libraries(closedLoop dart ${DARTExt_LIBRARIES})
set_target_properties(closedLoop PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
 
