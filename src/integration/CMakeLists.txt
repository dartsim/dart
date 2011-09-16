###############################################
# integration
PROJECT(integration)

# Search all header and source files
FILE(GLOB integration_srcs "*.cpp")
FILE(GLOB integration_hdrs "*.h")

# Library 
ADD_LIBRARY(integration ${integration_srcs} ${integration_hdrs})
SET_TARGET_PROPERTIES(integration PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")


