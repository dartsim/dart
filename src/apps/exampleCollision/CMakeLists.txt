###############################################
# apps/exampleCollision
PROJECT(exampleCollision)
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")
ADD_EXECUTABLE(exampleCollision ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(exampleCollision collision model3d dynamics yui renderer freeimage)
if (WIN32)
    TARGET_LINK_LIBRARIES(exampleCollision debug ticpp_d optimized ticpp)
else()
    TARGET_LINK_LIBRARIES(exampleCollision ticpp)
endif()

TARGET_LINK_LIBRARIES(exampleCollision ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
TARGET_LINK_LIBRARIES(exampleCollision glog gflags)
SET_TARGET_PROPERTIES(exampleCollision PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
