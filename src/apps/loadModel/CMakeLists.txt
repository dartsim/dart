###############################################
# apps/loadModel
PROJECT(loadModel)
FILE(GLOB load_model_srcs "*.cpp")
FILE(GLOB load_model_hdrs "*.h")
ADD_EXECUTABLE(loadModel ${load_model_srcs} ${load_model_hdrs})
TARGET_LINK_LIBRARIES(loadModel kinematics yui renderer)
LINK_THIRDPARTY_FREE_IMAGE(loadModel)
LINK_THIRDPARTY_GFLAGS(loadModel)
LINK_THIRDPARTY_GLOG(loadModel)
LINK_THIRDPARTY_GLUT(loadModel)
LINK_THIRDPARTY_TICPP(loadModel)

SET_TARGET_PROPERTIES(loadModel PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
