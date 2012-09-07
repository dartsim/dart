###############################################
# apps/cubes
if(DART_UI)
	PROJECT(collisionProfiling)
	FILE(GLOB collision_srcs "*.cpp")
	FILE(GLOB collision_hdrs "*.h")
	ADD_EXECUTABLE(collisionProfiling ${collision_srcs} ${collision_hdrs})
	TARGET_LINK_LIBRARIES(collisionProfiling lcpsolver collision kinematics dynamics yui renderer integration geometry)
	LINK_THIRDPARTY_FREE_IMAGE(collisionProfiling)
	LINK_THIRDPARTY_GFLAGS(collisionProfiling)
	LINK_THIRDPARTY_GLOG(collisionProfiling)
	LINK_THIRDPARTY_GLUT(collisionProfiling)
	
	SET_TARGET_PROPERTIES(collisionProfiling PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
