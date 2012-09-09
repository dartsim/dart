###############################################
# apps/lowerBody
if(DART_UI)
	PROJECT(lowerBody)
	FILE(GLOB lowerBody_srcs "*.cpp")
	FILE(GLOB lowerBody_hdrs "*.h")
	ADD_EXECUTABLE(lowerBody ${lowerBody_srcs} ${lowerBody_hdrs})
	TARGET_LINK_LIBRARIES(lowerBody lcpsolver collision kinematics dynamics yui renderer integration geometry)
	LINK_THIRDPARTY_GLUT(lowerBody)
	
	SET_TARGET_PROPERTIES(lowerBody PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
