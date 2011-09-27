###############################################
# apps/exampleOpt
PROJECT(exampleOpt)
FILE(GLOB exampleOpt_srcs "*.cpp")
FILE(GLOB exampleOpt_hdrs "*.h")
ADD_EXECUTABLE(exampleOpt ${exampleOpt_srcs} ${exampleOpt_hdrs})
TARGET_LINK_LIBRARIES(exampleOpt optimizer)
TARGET_LINK_LIBRARIES(exampleOpt gfortran snopt)
LINK_THIRDPARTY_GFLAGS(exampleOpt)
LINK_THIRDPARTY_GLOG(exampleOpt)

SET_TARGET_PROPERTIES(exampleOpt PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
