###############################################
# apps/handTracker
if(DART_UI)
	PROJECT(handTracker)
	FILE(GLOB handTracker_srcs "*.cpp")
	FILE(GLOB handTracker_hdrs "*.h")
	ADD_EXECUTABLE(handTracker ${handTracker_srcs} ${handTracker_hdrs})
	TARGET_LINK_LIBRARIES(handTracker kinematics dynamics integration yui renderer)
	LINK_THIRDPARTY_FREE_IMAGE(handTracker)
	LINK_THIRDPARTY_GFLAGS(handTracker)
	LINK_THIRDPARTY_GLOG(handTracker)
	LINK_THIRDPARTY_GLUT(handTracker)
	
	SET_TARGET_PROPERTIES(handTracker PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)

