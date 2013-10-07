###############################################
# apps/humanIK
PROJECT(humanIK)
FILE(GLOB humanIK_srcs "*.cpp")
FILE(GLOB humanIK_hdrs "*.h")
ADD_EXECUTABLE(humanIK ${humanIK_srcs} ${humanIK_hdrs})
TARGET_LINK_LIBRARIES(humanIK dart ${DARTExt_LIBRARIES})

SET_TARGET_PROPERTIES(humanIK PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
