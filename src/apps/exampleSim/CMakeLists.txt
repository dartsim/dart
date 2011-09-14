###############################################
# apps/exampleSim
PROJECT(exampleSim)
FILE(GLOB viewer_srcs "*.cpp")
FILE(GLOB viewer_hdrs "*.h")
ADD_EXECUTABLE(exampleSim ${viewer_srcs} ${viewer_hdrs})
TARGET_LINK_LIBRARIES(exampleSim kinematics dynamics yui renderer freeimage)
if (WIN32)
    TARGET_LINK_LIBRARIES(exampleSim debug ticpp_d optimized ticpp)
else()
    TARGET_LINK_LIBRARIES(exampleSim ticpp)
endif()

TARGET_LINK_LIBRARIES(exampleSim ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
TARGET_LINK_LIBRARIES(exampleSim glog gflags)
SET_TARGET_PROPERTIES(exampleSim PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
