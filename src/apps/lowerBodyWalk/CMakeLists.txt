###############################################
# apps/lowerBodyWalk
if(DART_UI)
	PROJECT(lowerBodyWalk)
	FILE(GLOB lowerBodyWalk_srcs "*.cpp")
	FILE(GLOB lowerBodyWalk_hdrs "*.h")
	ADD_EXECUTABLE(lowerBodyWalk ${lowerBodyWalk_srcs} ${lowerBodyWalk_hdrs})
	TARGET_LINK_LIBRARIES(lowerBodyWalk lcpsolver collision kinematics dynamics yui renderer integration geometry)
	LINK_THIRDPARTY_FREE_IMAGE(lowerBodyWalk)
	LINK_THIRDPARTY_GFLAGS(lowerBodyWalk)
	LINK_THIRDPARTY_GLOG(lowerBodyWalk)
	LINK_THIRDPARTY_GLUT(lowerBodyWalk)
	
	SET_TARGET_PROPERTIES(lowerBodyWalk PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
