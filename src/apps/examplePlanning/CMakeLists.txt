###############################################
# apps/examplePlanning
if(DART_UI AND PLANNING)
	PROJECT(examplePlanning)
	FILE(GLOB srcs "*.cpp")
	FILE(GLOB hdrs "*.h")
	ADD_EXECUTABLE(examplePlanning ${srcs} ${hdrs})
	TARGET_LINK_LIBRARIES(examplePlanning collision kinematics dynamics yui renderer integration planning robotics parser optimized fcl debug fcld ccd)
	LINK_THIRDPARTY_GLUT(examplePlanning)
	
	SET_TARGET_PROPERTIES(examplePlanning PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif()
