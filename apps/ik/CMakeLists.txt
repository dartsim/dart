###############################################
# apps/ik
FILE(GLOB ik_srcs "*.cpp")
FILE(GLOB ik_hdrs "*.h")
ADD_EXECUTABLE(ik ${ik_srcs} ${ik_hdrs})
TARGET_LINK_LIBRARIES(ik dart)

SET_TARGET_PROPERTIES(ik PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
