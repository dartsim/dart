###############################################
# apps/cubes
if(DART_UI)
	PROJECT(cubes)
	FILE(GLOB collision_srcs "*.cpp")
	FILE(GLOB collision_hdrs "*.h")
	ADD_EXECUTABLE(cubes ${collision_srcs} ${collision_hdrs})
	TARGET_LINK_LIBRARIES(cubes lcpsolver collision kinematics dynamics yui renderer integration)
	LINK_THIRDPARTY_GFLAGS(cubes)
	LINK_THIRDPARTY_GLUT(cubes)
	
	SET_TARGET_PROPERTIES(cubes PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
