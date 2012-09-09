###############################################
# apps/exampleJointLimit
if(DART_UI)
	PROJECT(exampleJointLimit)
	FILE(GLOB JointLimit_srcs "*.cpp")
	FILE(GLOB JointLimit_hdrs "*.h")
	ADD_EXECUTABLE(exampleJointLimit ${JointLimit_srcs} ${JointLimit_hdrs})
	TARGET_LINK_LIBRARIES(exampleJointLimit lcpsolver kinematics dynamics yui renderer integration)
	LINK_THIRDPARTY_GFLAGS(exampleJointLimit)
	LINK_THIRDPARTY_GLOG(exampleJointLimit)
	LINK_THIRDPARTY_GLUT(exampleJointLimit)
	
	SET_TARGET_PROPERTIES(exampleJointLimit PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
