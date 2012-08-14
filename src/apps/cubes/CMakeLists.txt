###############################################
# apps/cubes
PROJECT(cubes)
FILE(GLOB cubes_srcs "*.cpp")
FILE(GLOB cubes_hdrs "*.h")
ADD_EXECUTABLE(cubes ${cubes_srcs} ${cubes_hdrs})
TARGET_LINK_LIBRARIES(cubes lcpsolver collision kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_FREE_IMAGE(cubes)
LINK_THIRDPARTY_GFLAGS(cubes)
LINK_THIRDPARTY_GLOG(cubes)
LINK_THIRDPARTY_GLUT(cubes)
LINK_THIRDPARTY_TICPP(cubes)

SET_TARGET_PROPERTIES(cubes PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
