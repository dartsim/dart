###############################################
# Dynamics
PROJECT(dynamics)

# Search all header and source files
FILE(GLOB dynamics_srcs "*.cpp")
FILE(GLOB dynamics_hdrs "*.h")

# Library 
ADD_LIBRARY(dynamics ${dynamics_srcs} ${dynamics_hdrs})
SET_TARGET_PROPERTIES(dynamics PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")

INSTALL(FILES ${dynamics_hdrs} DESTINATION include/dart/dynamics)
INSTALL(TARGETS dynamics ARCHIVE DESTINATION lib)