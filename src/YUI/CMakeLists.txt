###############################################
# YUI
PROJECT(YUI)
# Search all header and source files
FILE(GLOB YUI_srcs "*.cpp")
FILE(GLOB YUI_hdrs "*.h")
# Library 
ADD_LIBRARY(YUI ${YUI_srcs} ${YUI_hdrs})
SET_TARGET_PROPERTIES(YUI PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
