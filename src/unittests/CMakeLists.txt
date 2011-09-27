###############################################
# Unittests
PROJECT(unittests)
# Search all header and source files
FILE(GLOB unittests_srcs "*.cpp")
FILE(GLOB unittests_hdrs "*.h")



ADD_EXECUTABLE(unittests ${unittests_srcs} ${unittests_hdrs})
TARGET_LINK_LIBRARIES(unittests utils kinematics dynamics optimizer)
# TARGET_LINK_LIBRARIES(unittests gfortran snopt)

LINK_THIRDPARTY_GFLAGS(unittests)
LINK_THIRDPARTY_GLOG(unittests)
LINK_THIRDPARTY_GLUT(unittests)
LINK_THIRDPARTY_GTEST(unittests)
LINK_THIRDPARTY_GTEST_MAIN(unittests)
LINK_THIRDPARTY_TICPP(unittests)

if (APPLE)
TARGET_LINK_LIBRARIES(unittests pthread)
elseif (UNIX)
  TARGET_LINK_LIBRARIES(unittests pthread glut)
endif()
SET_TARGET_PROPERTIES(unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
