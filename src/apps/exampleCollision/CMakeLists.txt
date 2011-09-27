###############################################
# apps/exampleCollision
PROJECT(exampleCollision)
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")
ADD_EXECUTABLE(exampleCollision ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(exampleCollision collision dynamics yui renderer)
LINK_THIRDPARTY_FREE_IMAGE(exampleCollision)
LINK_THIRDPARTY_GFLAGS(exampleCollision)
LINK_THIRDPARTY_GLOG(exampleCollision)
LINK_THIRDPARTY_GLUT(exampleCollision)
LINK_THIRDPARTY_TICPP(exampleCollision)

SET_TARGET_PROPERTIES(exampleCollision PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
