###############################################
# Unittests
PROJECT(unittests)
# Search all header and source files
FILE(GLOB unittests_srcs "*.cpp")
FILE(GLOB unittests_hdrs "*.h")



ADD_EXECUTABLE(unittests ${unittests_srcs} ${unittests_hdrs})
TARGET_LINK_LIBRARIES(unittests utils model3d optimizer ticpp)
TARGET_LINK_LIBRARIES(unittests glog gflags gfortran snopt)
TARGET_LINK_LIBRARIES(unittests ${GLUT_glut_LIBRARY})

if (APPLE)
TARGET_LINK_LIBRARIES(unittests gtest pthread)
elseif (UNIX)
  TARGET_LINK_LIBRARIES(unittests gtest pthread glut)
endif()
if (WIN32)
  TARGET_LINK_LIBRARIES(unittests debug gtestd optimized gtest)
  TARGET_LINK_LIBRARIES(unittests debug gtest_maind optimized gtest_main)
endif()
SET_TARGET_PROPERTIES(unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
