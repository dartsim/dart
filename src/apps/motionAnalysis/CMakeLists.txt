###############################################
# apps/motionAnalysis
PROJECT(motionAnalysis)
FILE(GLOB motionAnalysis_srcs "*.cpp")
FILE(GLOB motionAnalysis_hdrs "*.h")
ADD_EXECUTABLE(motionAnalysis ${motionAnalysis_srcs} ${motionAnalysis_hdrs})
TARGET_LINK_LIBRARIES(motionAnalysis kinematics dynamics yui renderer optimized fcl debug fcld)
LINK_THIRDPARTY_GLUT(motionAnalysis)

SET_TARGET_PROPERTIES(motionAnalysis PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
