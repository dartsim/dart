###############################################
# apps/exampleCollisionWithSkelton
PROJECT(exampleCollisionWithSkelton)
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")
ADD_EXECUTABLE(exampleCollisionWithSkelton ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(exampleCollisionWithSkelton lcpsolver collision kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_FREE_IMAGE(exampleCollisionWithSkelton)
LINK_THIRDPARTY_GFLAGS(exampleCollisionWithSkelton)
LINK_THIRDPARTY_GLOG(exampleCollisionWithSkelton)
LINK_THIRDPARTY_GLUT(exampleCollisionWithSkelton)
LINK_THIRDPARTY_TICPP(exampleCollisionWithSkelton)

SET_TARGET_PROPERTIES(exampleCollisionWithSkelton PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
