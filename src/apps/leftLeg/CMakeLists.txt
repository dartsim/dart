###############################################
# apps/leftLeg
if(DART_UI)
	PROJECT(leftLeg)
	FILE(GLOB leftLeg_srcs "*.cpp")
	FILE(GLOB leftLeg_hdrs "*.h")
	ADD_EXECUTABLE(leftLeg ${leftLeg_srcs} ${leftLeg_hdrs})
	TARGET_LINK_LIBRARIES(leftLeg lcpsolver collision kinematics dynamics yui renderer integration geometry)
	LINK_THIRDPARTY_GFLAGS(leftLeg)
	LINK_THIRDPARTY_GLOG(leftLeg)
	LINK_THIRDPARTY_GLUT(leftLeg)
	
	SET_TARGET_PROPERTIES(leftLeg PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
