###############################################
# apps/meshTest
if(DART_UI)
	PROJECT(meshTest)
	FILE(GLOB meshTest_srcs "*.cpp")
	FILE(GLOB meshTest_hdrs "*.h")
	ADD_EXECUTABLE(meshTest ${meshTest_srcs} ${meshTest_hdrs})
	TARGET_LINK_LIBRARIES(meshTest lcpsolver collision kinematics dynamics yui renderer integration geometry)
	LINK_THIRDPARTY_GFLAGS(meshTest)
	LINK_THIRDPARTY_GLOG(meshTest)
	LINK_THIRDPARTY_GLUT(meshTest)
	
	SET_TARGET_PROPERTIES(meshTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
