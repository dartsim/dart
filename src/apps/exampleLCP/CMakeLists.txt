###############################################
# apps/exampleLCP
PROJECT(exampleLCP)
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")
ADD_EXECUTABLE(exampleLCP ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(exampleLCP lcpsolver collision kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_FREE_IMAGE(exampleLCP)
LINK_THIRDPARTY_GFLAGS(exampleLCP)
LINK_THIRDPARTY_GLOG(exampleLCP)
LINK_THIRDPARTY_GLUT(exampleLCP)

SET_TARGET_PROPERTIES(exampleLCP PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
