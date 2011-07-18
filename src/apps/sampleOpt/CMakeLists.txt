###############################################
# apps/sampleOpt
PROJECT(sampleOpt)
FILE(GLOB sampleOpt_srcs "*.cpp")
FILE(GLOB sampleOpt_hdrs "*.h")
ADD_EXECUTABLE(sampleOpt ${sampleOpt_srcs} ${sampleOpt_hdrs})
TARGET_LINK_LIBRARIES(sampleOpt optimizer)
TARGET_LINK_LIBRARIES(sampleOpt glog gflags gfortran snopt)
SET_TARGET_PROPERTIES(sampleOpt PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
