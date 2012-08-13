###############################################
# apps/rightLeg
if(DART_UI)
	PROJECT(rightLeg)
	FILE(GLOB rightLeg_srcs "*.cpp")
	FILE(GLOB rightLeg_hdrs "*.h")
	ADD_EXECUTABLE(rightLeg ${rightLeg_srcs} ${rightLeg_hdrs})
	TARGET_LINK_LIBRARIES(rightLeg lcpsolver collision kinematics dynamics yui renderer integration geometry)
	LINK_THIRDPARTY_FREE_IMAGE(rightLeg)
	LINK_THIRDPARTY_GFLAGS(rightLeg)
	LINK_THIRDPARTY_GLOG(rightLeg)
	LINK_THIRDPARTY_GLUT(rightLeg)
	
	SET_TARGET_PROPERTIES(rightLeg PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
