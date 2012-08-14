###############################################
# apps/exampleIK
PROJECT(exampleIK)
FILE(GLOB exampleIK_srcs "*.cpp")
FILE(GLOB exampleIK_hdrs "*.h")
ADD_EXECUTABLE(exampleIK ${exampleIK_srcs} ${exampleIK_hdrs})
TARGET_LINK_LIBRARIES(exampleIK kinematics optimizer)
LINK_THIRDPARTY_GFLAGS(exampleIK)
LINK_THIRDPARTY_GLOG(exampleIK)
LINK_THIRDPARTY_GLUT(exampleIK)
LINK_THIRDPARTY_TICPP(exampleIK)

TARGET_LINK_LIBRARIES(exampleIK gfortran snopt_c snprint_c blas_c f2c)
SET_TARGET_PROPERTIES(exampleIK PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
