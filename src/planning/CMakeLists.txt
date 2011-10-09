# ----------------------------
# Planning project
# ----------------------------
PROJECT( planning )

# Search all header and source files
FILE( GLOB planning_sources "*.cpp" )
FILE( GLOB planning_headers "*.h" )
FILE( GLOB planning_hpps "*.hpp" )

# Library
ADD_LIBRARY( planning ${planning_sources} ${planning_headers} ${planning_hpps} )
TARGET_LINK_LIBRARIES( planning utils kinematics )
SET_TARGET_PROPERTIES( planning PROPERTIES ARCHIVE_OUTPUT_DIRECTORY
 "${CMAKE_SOURCE_DIR}/lib" )
