# ----------------------------
# Planning project
# ----------------------------
PROJECT( planning )

# Search all header and source files
FILE( GLOB planning_srcs "*.cpp" )
FILE( GLOB planning_hdrs "*.h" )
FILE( GLOB planning_hpps "*.hpp" )
FILE( GLOB gl_srcs GL/GLTexture.cpp )
FILE( GLOB gl_hdrs GL/*.h)

# Library
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIRS})
ADD_LIBRARY( planning ${planning_srcs} ${gl_srcs} ${planning_hdrs} ${planning_hpps} )
TARGET_LINK_LIBRARIES( planning utils)
SET_TARGET_PROPERTIES( planning PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )
SET_TARGET_PROPERTIES( planning PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )

INSTALL(FILES ${planning_hdrs} DESTINATION include/dart/planning)
INSTALL(FILES ${planning_hpps} DESTINATION include/dart/planning)
INSTALL(FILES ${gl_hdrs} DESTINATION include/dart/planning/GL)
INSTALL(TARGETS planning DESTINATION lib)
