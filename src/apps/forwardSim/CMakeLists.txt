###############################################
# apps/forwardSim
PROJECT(forwardSim)
FILE(GLOB forwardSim_srcs "*.cpp")
FILE(GLOB forwardSim_hdrs "*.h")
ADD_EXECUTABLE(forwardSim ${forwardSim_srcs} ${forwardSim_hdrs})
TARGET_LINK_LIBRARIES(forwardSim kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_FREE_IMAGE(forwardSim)
LINK_THIRDPARTY_GFLAGS(forwardSim)
LINK_THIRDPARTY_GLOG(forwardSim)
LINK_THIRDPARTY_GLUT(forwardSim)
LINK_THIRDPARTY_TICPP(forwardSim)

SET_TARGET_PROPERTIES(forwardSim PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
