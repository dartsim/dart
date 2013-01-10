###############################################
# apps/hybrid
PROJECT(hybrid)
FILE(GLOB hybrid_srcs "*.cpp")
FILE(GLOB hybrid_hdrs "*.h")
ADD_EXECUTABLE(hybrid ${hybrid_srcs} ${hybrid_hdrs})
TARGET_LINK_LIBRARIES(hybrid kinematics dynamics yui renderer integration optimized fcl debug fcld)
LINK_THIRDPARTY_GLUT(hybrid)

SET_TARGET_PROPERTIES(hybrid PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
