###############################################
# apps/exampleCollisionWithSkelton
PROJECT(exampleCollisionWithSkelton)
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")
ADD_EXECUTABLE(exampleCollisionWithSkelton ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(exampleCollisionWithSkelton collision kinematics dynamics yui renderer integration freeimage)
if (WIN32)
    TARGET_LINK_LIBRARIES(exampleCollisionWithSkelton debug ticpp_d optimized ticpp)
else()
    TARGET_LINK_LIBRARIES(exampleCollisionWithSkelton ticpp)
endif()

TARGET_LINK_LIBRARIES(exampleCollisionWithSkelton ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
TARGET_LINK_LIBRARIES(exampleCollisionWithSkelton glog gflags)
SET_TARGET_PROPERTIES(exampleCollisionWithSkelton PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
