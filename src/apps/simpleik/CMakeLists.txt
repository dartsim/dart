###############################################
# apps/simpleik
PROJECT(simpleik)
FILE(GLOB simpleik_srcs "*.cpp")
FILE(GLOB simpleik_hdrs "*.h")
ADD_EXECUTABLE(simpleik ${simpleik_srcs} ${simpleik_hdrs})
TARGET_LINK_LIBRARIES(simpleik model3d optimizer)

if (WIN32)
    TARGET_LINK_LIBRARIES(simpleik debug ticpp_d optimized ticpp)
else()
    TARGET_LINK_LIBRARIES(simpleik ticpp)
endif()

TARGET_LINK_LIBRARIES(simpleik ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
TARGET_LINK_LIBRARIES(simpleik glog gflags gfortran snopt)
SET_TARGET_PROPERTIES(simpleik PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
