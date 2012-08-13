###############################################
# apps/lowerBodyFreeFall
if(DART_UI)
	PROJECT(lowerBodyFreeFall)
	FILE(GLOB lowerBodyFreeFall_srcs "*.cpp")
	FILE(GLOB lowerBodyFreeFall_hdrs "*.h")
	ADD_EXECUTABLE(lowerBodyFreeFall ${lowerBodyFreeFall_srcs} ${lowerBodyFreeFall_hdrs})
	TARGET_LINK_LIBRARIES(lowerBodyFreeFall lcpsolver collision kinematics dynamics yui renderer integration geometry)
	LINK_THIRDPARTY_FREE_IMAGE(lowerBodyFreeFall)
	LINK_THIRDPARTY_GFLAGS(lowerBodyFreeFall)
	LINK_THIRDPARTY_GLOG(lowerBodyFreeFall)
	LINK_THIRDPARTY_GLUT(lowerBodyFreeFall)
	
	SET_TARGET_PROPERTIES(lowerBodyFreeFall PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
