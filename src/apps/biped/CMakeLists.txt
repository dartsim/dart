###############################################
# apps/biped
<<<<<<< HEAD
if(DART_UI)
	PROJECT(biped)
	FILE(GLOB collision_srcs "*.cpp")
	FILE(GLOB collision_hdrs "*.h")
	ADD_EXECUTABLE(biped ${collision_srcs} ${collision_hdrs})
	TARGET_LINK_LIBRARIES(biped lcpsolver collision kinematics dynamics yui renderer integration)
	LINK_THIRDPARTY_FREE_IMAGE(biped)
	LINK_THIRDPARTY_GFLAGS(biped)
	LINK_THIRDPARTY_GLOG(biped)
	LINK_THIRDPARTY_GLUT(biped)
	
	SET_TARGET_PROPERTIES(biped PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif(DART_UI)
=======
PROJECT(biped)
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")
ADD_EXECUTABLE(biped ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(biped lcpsolver collision kinematics dynamics yui renderer integration)
TARGET_LINK_LIBRARIES(biped optimizer snopt_c snprint_c blas_c f2c)
LINK_THIRDPARTY_FREE_IMAGE(biped)
LINK_THIRDPARTY_GFLAGS(biped)
LINK_THIRDPARTY_GLOG(biped)
LINK_THIRDPARTY_GLUT(biped)
LINK_THIRDPARTY_TICPP(biped)

SET_TARGET_PROPERTIES(biped PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
>>>>>>> karen
