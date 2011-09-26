###############################################
# apps/handTracker
PROJECT(handTracker)
FILE(GLOB handTracker_srcs "*.cpp")
FILE(GLOB handTracker_hdrs "*.h")
ADD_EXECUTABLE(handTracker ${handTracker_srcs} ${handTracker_hdrs})
TARGET_LINK_LIBRARIES(handTracker kinematics dynamics integration yui renderer freeimage)
if (WIN32)
    TARGET_LINK_LIBRARIES(handTracker debug ticpp_d optimized ticpp)
else()
    TARGET_LINK_LIBRARIES(handTracker ticpp)
endif()

TARGET_LINK_LIBRARIES(handTracker ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
TARGET_LINK_LIBRARIES(handTracker glog gflags)
SET_TARGET_PROPERTIES(handTracker PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

