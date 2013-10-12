###############################################
# apps/hybrid
FILE(GLOB hybrid_srcs "*.cpp")
FILE(GLOB hybrid_hdrs "*.h")
ADD_EXECUTABLE(hybrid ${hybrid_srcs} ${hybrid_hdrs})
TARGET_LINK_LIBRARIES(hybrid dart)

SET_TARGET_PROPERTIES(hybrid PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
