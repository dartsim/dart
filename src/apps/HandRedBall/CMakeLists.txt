###############################################
# apps/manipulator
if(DART_UI)
	PROJECT(manipulator)
	FILE(GLOB manipulator_srcs "*.cpp")
	FILE(GLOB manipulator_hdrs "*.h")
	ADD_EXECUTABLE(manipulator ${manipulator_srcs} ${manipulator_hdrs})
	TARGET_LINK_LIBRARIES(manipulator kinematics dynamics yui renderer integration)
	LINK_THIRDPARTY_GFLAGS(manipulator)
	LINK_THIRDPARTY_GLUT(manipulator)
	
	SET_TARGET_PROPERTIES(manipulator PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
