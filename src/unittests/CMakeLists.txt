###############################################
# Unittests

# Depends on snopt
if(NOT SNOPT)
  MESSAGE(STATUS "Disabling some unittests. Cannot build all tests without snopt")
endif()

PROJECT(unittests)
# Search all header and source files
FILE(GLOB unittests_srcs "*.cpp")
FILE(GLOB unittests_hdrs "*.h")

if(SNOPT)
    FILE(GLOB unittests_snopt "*.cpp" "*.h")
    SET(snopt_lib snopt gfortran)
endif(SNOPT)

ADD_EXECUTABLE(unittests ${unittests_srcs} ${unittests_hdrs} ${unittests_snopt})
TARGET_LINK_LIBRARIES(unittests utils kinematics dynamics optimizer ${snopt_lib})

LINK_THIRDPARTY_GFLAGS(unittests)
LINK_THIRDPARTY_GLUT(unittests)
LINK_THIRDPARTY_GTEST(unittests)
LINK_THIRDPARTY_TINYXML(unittests)

if (APPLE OR UNIX)
  TARGET_LINK_LIBRARIES(unittests pthread)
endif(APPLE OR UNIX)

SET_TARGET_PROPERTIES(unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
