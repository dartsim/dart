###############################################
# geometry
PROJECT(geometry)

# Search all header and source files
FILE(GLOB geometry_srcs "*.cpp")
FILE(GLOB geometry_hdrs "*.h")

# Library 
ADD_LIBRARY(geometry SHARED ${geometry_srcs} ${geometry_hdrs})
SET_TARGET_PROPERTIES(geometry PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")

INSTALL(FILES ${geometry_hdrs} DESTINATION include/dart/geometry)
INSTALL(TARGETS geometry DESTINATION lib)
