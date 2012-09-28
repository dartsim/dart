###############################################
# apps/balance
PROJECT(balance)
FILE(GLOB balance_srcs "*.cpp")
FILE(GLOB balance_hdrs "*.h")
ADD_EXECUTABLE(balance ${balance_srcs} ${balance_hdrs})
TARGET_LINK_LIBRARIES(balance lcpsolver collision kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_GLUT(balance)

SET_TARGET_PROPERTIES(balance PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
