###############################################
# Utils
PROJECT(mayaexports)

# Search all header and source files
FILE(GLOB mayaexports_srcs "*.cpp")
FILE(GLOB mayaexports_hdrs "*.h")

# Library 
ADD_LIBRARY(mayaexports ${mayaexports_srcs} ${mayaexports_hdrs})
SET_TARGET_PROPERTIES(mayaexports PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")


