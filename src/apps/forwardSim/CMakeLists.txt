###############################################
# apps/forwardSim
PROJECT(forwardSim)
FILE(GLOB forwardSim_srcs "*.cpp")
FILE(GLOB forwardSim_hdrs "*.h")
ADD_EXECUTABLE(forwardSim ${forwardSim_srcs} ${forwardSim_hdrs})
TARGET_LINK_LIBRARIES(forwardSim kinematics dynamics yui renderer integration)
LINK_THIRDPARTY_GLUT(forwardSim)

SET_TARGET_PROPERTIES(forwardSim PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
