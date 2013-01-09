###############################################
# apps/exampleCollisionWithSkelton
if(DART_UI)
	PROJECT(exampleCollisionWithSkelton)
	FILE(GLOB collision_srcs "*.cpp")
	FILE(GLOB collision_hdrs "*.h")
	ADD_EXECUTABLE(exampleCollisionWithSkelton ${collision_srcs} ${collision_hdrs})
	TARGET_LINK_LIBRARIES(exampleCollisionWithSkelton lcpsolver collision kinematics dynamics yui renderer integration ccd fcl)
	LINK_THIRDPARTY_GLUT(exampleCollisionWithSkelton)
	
	SET_TARGET_PROPERTIES(exampleCollisionWithSkelton PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
