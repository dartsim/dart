###############################################
# apps/motionAnalysis
PROJECT(motionAnalysis)
FILE(GLOB motionAnalysis_srcs "*.cpp")
FILE(GLOB motionAnalysis_hdrs "*.h")
ADD_EXECUTABLE(motionAnalysis ${motionAnalysis_srcs} ${motionAnalysis_hdrs})
TARGET_LINK_LIBRARIES(motionAnalysis kinematics dynamics yui renderer)
LINK_THIRDPARTY_FREE_IMAGE(motionAnalysis)
LINK_THIRDPARTY_GFLAGS(motionAnalysis)
LINK_THIRDPARTY_GLOG(motionAnalysis)
LINK_THIRDPARTY_GLUT(motionAnalysis)
LINK_THIRDPARTY_TICPP(motionAnalysis)

SET_TARGET_PROPERTIES(motionAnalysis PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
