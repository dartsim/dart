###############################################
# apps/legIK
PROJECT(legIK)
FILE(GLOB legIK_srcs "*.cpp")
FILE(GLOB legIK_hdrs "*.h")
ADD_EXECUTABLE(legIK ${legIK_srcs} ${legIK_hdrs})
TARGET_LINK_LIBRARIES(legIK dart ${DARTExt_LIBRARIES})

SET_TARGET_PROPERTIES(legIK PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
