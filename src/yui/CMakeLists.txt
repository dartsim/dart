###############################################
# YUI
PROJECT(yui)
# Search all header and source files
FILE(GLOB yui_srcs "*.cpp")
FILE(GLOB yui_hdrs "*.h")
# Library 
ADD_LIBRARY(yui ${yui_srcs} ${yui_hdrs})
SET_TARGET_PROPERTIES(yui PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
