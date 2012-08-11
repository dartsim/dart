###############################################
# apps/grab
if(DART_UI)
	PROJECT(grab)
	FILE(GLOB collision_srcs "*.cpp")
	FILE(GLOB collision_hdrs "*.h")
	ADD_EXECUTABLE(grab ${collision_srcs} ${collision_hdrs})
	TARGET_LINK_LIBRARIES(grab lcpsolver collision kinematics dynamics yui renderer integration)
	LINK_THIRDPARTY_FREE_IMAGE(grab)
	LINK_THIRDPARTY_GFLAGS(grab)
	LINK_THIRDPARTY_GLOG(grab)
	LINK_THIRDPARTY_GLUT(grab)

	SET_TARGET_PROPERTIES(grab PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)