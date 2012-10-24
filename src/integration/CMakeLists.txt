###############################################
# integration
PROJECT(integration)

# Search all header and source files
FILE(GLOB integration_srcs "*.cpp")
FILE(GLOB integration_hdrs "*.h")

# Library 
ADD_LIBRARY(integration ${integration_srcs} ${integration_hdrs})
SET_TARGET_PROPERTIES(integration PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
SET_TARGET_PROPERTIES(integration PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
IF(MSVC)
  SET_TARGET_PROPERTIES(integration PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
ENDIF()

INSTALL(FILES ${integration_hdrs} DESTINATION include/dart/integration)
INSTALL(FILES ${integration_hpps} DESTINATION include/dart/integration)
INSTALL(TARGETS integration DESTINATION lib)
