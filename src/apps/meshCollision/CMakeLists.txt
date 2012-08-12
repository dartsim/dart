###############################################
# apps/meshCollision
if(DART_UI)
	PROJECT(meshCollision)
	FILE(GLOB meshCollision_srcs "*.cpp")
	FILE(GLOB meshCollision_hdrs "*.h")
	ADD_EXECUTABLE(meshCollision ${meshCollision_srcs} ${meshCollision_hdrs})
	TARGET_LINK_LIBRARIES(meshCollision lcpsolver collision kinematics dynamics yui renderer integration geometry)
	LINK_THIRDPARTY_FREE_IMAGE(meshCollision)
	LINK_THIRDPARTY_GFLAGS(meshCollision)
	LINK_THIRDPARTY_GLOG(meshCollision)
	LINK_THIRDPARTY_GLUT(meshCollision)
	
	SET_TARGET_PROPERTIES(meshCollision PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
