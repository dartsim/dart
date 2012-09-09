###############################################
# apps/hybrid
PROJECT(hybrid)
FILE(GLOB hybrid_srcs "*.cpp")
FILE(GLOB hybrid_hdrs "*.h")
ADD_EXECUTABLE(hybrid ${hybrid_srcs} ${hybrid_hdrs})
TARGET_LINK_LIBRARIES(hybrid kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_FREE_IMAGE(hybrid)
LINK_THIRDPARTY_GFLAGS(hybrid)
LINK_THIRDPARTY_GLOG(hybrid)
LINK_THIRDPARTY_GLUT(hybrid)
LINK_THIRDPARTY_TICPP(hybrid)

SET_TARGET_PROPERTIES(hybrid PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
