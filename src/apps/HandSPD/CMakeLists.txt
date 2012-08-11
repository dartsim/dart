###############################################
# apps/pdController
PROJECT(pdController)
FILE(GLOB pdController_srcs "*.cpp")
FILE(GLOB pdController_hdrs "*.h")
ADD_EXECUTABLE(pdController ${pdController_srcs} ${pdController_hdrs})
TARGET_LINK_LIBRARIES(pdController kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_FREE_IMAGE(pdController)
LINK_THIRDPARTY_GFLAGS(pdController)
LINK_THIRDPARTY_GLOG(pdController)
LINK_THIRDPARTY_GLUT(pdController)
#LINK_THIRDPARTY_TICPP(pdController)

SET_TARGET_PROPERTIES(pdController PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
