###############################################
# apps/exampleDynamics
PROJECT(exampleDynamics)
FILE(GLOB example_dynamics_srcs "*.cpp")
FILE(GLOB example_dynamics_hdrs "*.h")
ADD_EXECUTABLE(exampleDynamics ${example_dynamics_srcs} ${example_dynamics_hdrs})
TARGET_LINK_LIBRARIES(exampleDynamics kinematics dynamics yui renderer)
LINK_THIRDPARTY_FREE_IMAGE(exampleDynamics)
LINK_THIRDPARTY_GFLAGS(exampleDynamics)
LINK_THIRDPARTY_GLOG(exampleDynamics)
LINK_THIRDPARTY_GLUT(exampleDynamics)

SET_TARGET_PROPERTIES(exampleDynamics PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
