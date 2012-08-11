###############################################
# apps/viewer
if(DART_UI)
	PROJECT(viewer)
	FILE(GLOB viewer_srcs "*.cpp")
	FILE(GLOB viewer_hdrs "*.h")
	ADD_EXECUTABLE(viewer ${viewer_srcs} ${viewer_hdrs})
	TARGET_LINK_LIBRARIES(viewer kinematics yui renderer)
	LINK_THIRDPARTY_FREE_IMAGE(viewer)
	LINK_THIRDPARTY_GFLAGS(viewer)
	LINK_THIRDPARTY_GLOG(viewer)
	LINK_THIRDPARTY_GLUT(viewer)
	
	SET_TARGET_PROPERTIES(viewer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)