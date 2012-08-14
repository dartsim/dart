###############################################
# apps/openLoop
PROJECT(openLoop)
FILE(GLOB openLoop_srcs "*.cpp")
FILE(GLOB openLoop_hdrs "*.h")
ADD_EXECUTABLE(openLoop ${openLoop_srcs} ${openLoop_hdrs})
TARGET_LINK_LIBRARIES(openLoop kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_FREE_IMAGE(openLoop)
LINK_THIRDPARTY_GFLAGS(openLoop)
LINK_THIRDPARTY_GLOG(openLoop)
LINK_THIRDPARTY_GLUT(openLoop)
LINK_THIRDPARTY_TICPP(openLoop)

SET_TARGET_PROPERTIES(openLoop PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
