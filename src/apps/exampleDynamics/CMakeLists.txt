###############################################
# apps/exampleDynamics
PROJECT(exampleDynamics)
FILE(GLOB example_dynamics_srcs "*.cpp")
FILE(GLOB example_dynamics_hdrs "*.h")
ADD_EXECUTABLE(exampleDynamics ${example_dynamics_srcs} ${example_dynamics_hdrs})
TARGET_LINK_LIBRARIES(exampleDynamics kinematics dynamics yui renderer gflags glog freeimage)
if (WIN32)
    TARGET_LINK_LIBRARIES(exampleDynamics debug ticpp_d optimized ticpp)
else()
    TARGET_LINK_LIBRARIES(exampleDynamics ticpp)
endif()

TARGET_LINK_LIBRARIES(exampleDynamics ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
SET_TARGET_PROPERTIES(exampleDynamics PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
