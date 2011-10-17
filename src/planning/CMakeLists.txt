# ----------------------------
# Planning project
# ----------------------------
PROJECT( planning )

# Search all header and source files
FILE( GLOB planning_srcs "*.cpp" )
FILE( GLOB planning_hdrs "*.h" )
FILE( GLOB planning_hpps "*.hpp" )

# Library
ADD_LIBRARY( planning ${planning_srcs} ${planning_hdrs} ${planning_hpps} )
TARGET_LINK_LIBRARIES( planning utils planning )
SET_TARGET_PROPERTIES( planning PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )

INSTALL(FILES ${planning_hdrs} DESTINATION include/dart/planning)
INSTALL(FILES ${planning_hpps} DESTINATION include/dart/planning)
INSTALL(TARGETS planning ARCHIVE DESTINATION lib)