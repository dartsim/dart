###############################################
# apps/exampleIK
PROJECT(exampleIK)
FILE(GLOB exampleIK_srcs "*.cpp")
FILE(GLOB exampleIK_hdrs "*.h")
ADD_EXECUTABLE(exampleIK ${exampleIK_srcs} ${exampleIK_hdrs})
TARGET_LINK_LIBRARIES(exampleIK kinematics optimizer)

if (WIN32)
    TARGET_LINK_LIBRARIES(exampleIK debug ticpp_d optimized ticpp)
else()
    TARGET_LINK_LIBRARIES(exampleIK ticpp)
endif()

TARGET_LINK_LIBRARIES(exampleIK ${OPENGL_LIBRARIES} ${GLUT_glut_LIBRARY})
TARGET_LINK_LIBRARIES(exampleIK glog gflags gfortran snopt)
SET_TARGET_PROPERTIES(exampleIK PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
