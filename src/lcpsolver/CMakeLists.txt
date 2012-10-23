###############################################
# lcpsolver
PROJECT(lcpsolver)

# Search all header and source files
FILE(GLOB lcpsolver_srcs "*.cpp")
FILE(GLOB lcpsolver_hdrs "*.h")

# Library 
ADD_LIBRARY(lcpsolver ${lcpsolver_srcs} ${lcpsolver_hdrs})
SET_TARGET_PROPERTIES(lcpsolver PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
SET_TARGET_PROPERTIES(lcpsolver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
IF(MSVC)
  SET_TARGET_PROPERTIES(lcpsolver PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
ENDIF()

INSTALL(FILES ${lcpsolver_hdrs} DESTINATION include/dart/lcpsolver)
INSTALL(FILES ${lcpsolver_hpps} DESTINATION include/dart/lcpsolver)
INSTALL(TARGETS lcpsolver DESTINATION lib)
