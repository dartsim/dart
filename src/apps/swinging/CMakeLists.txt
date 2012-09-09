###############################################
# apps/lowerBodyWalk
if(DART_UI)
	PROJECT(swinging)
	FILE(GLOB swinging_srcs "*.cpp")
	FILE(GLOB swinging_hdrs "*.h")
	ADD_EXECUTABLE(swinging ${swinging_srcs} ${swinging_hdrs})
	TARGET_LINK_LIBRARIES(swinging lcpsolver collision kinematics dynamics yui renderer integration geometry)
	LINK_THIRDPARTY_GLUT(swinging)
	
	SET_TARGET_PROPERTIES(swinging PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
