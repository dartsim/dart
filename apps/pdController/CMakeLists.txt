###############################################
# apps/pdController
FILE(GLOB pdController_srcs "*.cpp")
FILE(GLOB pdController_hdrs "*.h")
ADD_EXECUTABLE(pdController ${pdController_srcs} ${pdController_hdrs})
TARGET_LINK_LIBRARIES(pdController dart)

SET_TARGET_PROPERTIES(pdController PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
