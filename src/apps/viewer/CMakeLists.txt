###############################################
# apps/viewer
PROJECT(viewer)
FILE(GLOB viewer_srcs "*.cpp")
FILE(GLOB viewer_hdrs "*.h")
ADD_EXECUTABLE(viewer ${viewer_srcs} ${viewer_hdrs})
TARGET_LINK_LIBRARIES(viewer kinematics yui renderer)
LINK_THIRDPARTY_GLUT(viewer)
SET_TARGET_PROPERTIES(viewer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

