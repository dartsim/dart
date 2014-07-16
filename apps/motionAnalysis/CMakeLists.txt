###############################################
# apps/motionAnalysis
FILE(GLOB motionAnalysis_srcs "*.cpp")
FILE(GLOB motionAnalysis_hdrs "*.h")
ADD_EXECUTABLE(motionAnalysis ${motionAnalysis_srcs} ${motionAnalysis_hdrs})
TARGET_LINK_LIBRARIES(motionAnalysis dart)

SET_TARGET_PROPERTIES(motionAnalysis PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
