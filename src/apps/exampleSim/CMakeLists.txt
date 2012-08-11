###############################################
# apps/exampleSim
if(DART_UI)
	PROJECT(exampleSim)
	FILE(GLOB viewer_srcs "*.cpp")
	FILE(GLOB viewer_hdrs "*.h")
	ADD_EXECUTABLE(exampleSim ${viewer_srcs} ${viewer_hdrs})
	TARGET_LINK_LIBRARIES(exampleSim kinematics dynamics yui renderer integration)
	LINK_THIRDPARTY_FREE_IMAGE(exampleSim)
	LINK_THIRDPARTY_GFLAGS(exampleSim)
	LINK_THIRDPARTY_GLOG(exampleSim)
	LINK_THIRDPARTY_GLUT(exampleSim)
	
	SET_TARGET_PROPERTIES(exampleSim PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
