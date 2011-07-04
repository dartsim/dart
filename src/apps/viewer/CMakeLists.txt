###############################################
# apps/viewer
PROJECT(viewer)
FILE(GLOB viewer_srcs "*.cpp")
FILE(GLOB viewer_hdrs "*.h")
ADD_EXECUTABLE(viewer ${viewer_srcs} ${viewer_hdrs})
TARGET_LINK_LIBRARIES(viewer model3d ticpp YUI renderer)
TARGET_LINK_LIBRARIES(viewer glut GL GLU glog gflags)
SET_TARGET_PROPERTIES(viewer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
