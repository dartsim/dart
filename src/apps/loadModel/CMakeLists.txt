###############################################
# apps/loadModel
PROJECT(loadModel)
FILE(GLOB load_model_srcs "*.cpp")
FILE(GLOB load_model_hdrs "*.h")
ADD_EXECUTABLE(loadModel ${load_model_srcs} ${load_model_hdrs})
TARGET_LINK_LIBRARIES(loadModel model3d yui renderer gflags glog freeimage)
if (WIN32)
    TARGET_LINK_LIBRARIES(loadModel debug ticpp_d optimized ticpp)
else()
    TARGET_LINK_LIBRARIES(loadModel ticpp)
endif()

TARGET_LINK_LIBRARIES(loadModel ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
SET_TARGET_PROPERTIES(loadModel PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
