###############################################
# apps/twister
PROJECT(twister)
FILE(GLOB twister_srcs "*.cpp")
FILE(GLOB twister_hdrs "*.h")
ADD_EXECUTABLE(twister ${twister_srcs} ${twister_hdrs})
TARGET_LINK_LIBRARIES(twister dart ${DARTExt_LIBRARIES})

SET_TARGET_PROPERTIES(twister PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
