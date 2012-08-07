# Robotics

PROJECT( robotics )

# Search all header and source files
FILE( GLOB robotics_srcs "*.cpp" )
FILE( GLOB robotics_hdrs "*.h" )
FILE( GLOB robotics_hpps "*.hpps" )
FILE( GLOB gl_srcs GL/GLTexture.cpp )
FILE( GLOB gl_hdrs GL/*.h )

# Library
INCLUDE_DIRECTORIES( ${GLUT_INCLUDE_DIRS} )
ADD_LIBRARY( robotics ${robotics_srcs} ${robotics_hdrs} ${robotics_hpps} )
TARGET_LINK_LIBRARIES( robotics utils )
SET_TARGET_PROPERTIES( robotics PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )
SET_TARGET_PROPERTIES( robotics PROPERTIES ARCHIVE_OUTPUT_DIRECTORY  "${CMAKE_SOURCE_DIR}/lib" )

INSTALL( FILES ${robotics_hdrs} DESTINATION include/dart/robotics )
INSTALL( FILES ${robotics_hpps} DESTINATION include/dart/robotics )
INSTALL( FILES ${gl_hdrs} DESTINATION include/dart/planning/GL )
INSTALL( TARGETS robotics DESTINATION lib )
