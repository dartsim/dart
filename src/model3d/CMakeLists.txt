###############################################
# Model3d
PROJECT(model3d)

# Search all header and source files
FILE(GLOB model3d_srcs "*.cpp")
FILE(GLOB model3d_hdrs "*.h")

# Library 
ADD_LIBRARY(model3d ${model3d_srcs} ${model3d_hdrs})
TARGET_LINK_LIBRARIES(model3d utils)
SET_TARGET_PROPERTIES(model3d PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
