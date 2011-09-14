###############################################
# apps/viewer
PROJECT(viewer)
FILE(GLOB viewer_srcs "*.cpp")
FILE(GLOB viewer_hdrs "*.h")
ADD_EXECUTABLE(viewer ${viewer_srcs} ${viewer_hdrs})
TARGET_LINK_LIBRARIES(viewer kinematics yui renderer freeimage)
if (WIN32)
    TARGET_LINK_LIBRARIES(viewer debug ticpp_d optimized ticpp)
else()
    TARGET_LINK_LIBRARIES(viewer ticpp)
endif()

TARGET_LINK_LIBRARIES(viewer ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
TARGET_LINK_LIBRARIES(viewer glog gflags)
SET_TARGET_PROPERTIES(viewer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
