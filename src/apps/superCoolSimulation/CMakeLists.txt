###############################################
# apps/superCoolSimulation
PROJECT(superCoolSimulation)
FILE(GLOB viewer_srcs "*.cpp")
FILE(GLOB viewer_hdrs "*.h")
ADD_EXECUTABLE(superCoolSimulation ${viewer_srcs} ${viewer_hdrs})
TARGET_LINK_LIBRARIES(superCoolSimulation lcpsolver collision kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_FREE_IMAGE(superCoolSimulation)
LINK_THIRDPARTY_GFLAGS(superCoolSimulation)
LINK_THIRDPARTY_GLOG(superCoolSimulation)
LINK_THIRDPARTY_GLUT(superCoolSimulation)
LINK_THIRDPARTY_TICPP(superCoolSimulation)

SET_TARGET_PROPERTIES(superCoolSimulation PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
