###############################################
# Collision
PROJECT(collision)

# Search all header and source files
FILE(GLOB collision_srcs "*.cpp")
FILE(GLOB collision_hdrs "*.h")


# Library 

ADD_LIBRARY(collision SHARED ${collision_srcs} ${collision_hdrs})
TARGET_LINK_LIBRARIES(collision kinematics)
SET_TARGET_PROPERTIES(collision PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")

INSTALL(FILES ${collision_hdrs} DESTINATION include/dart/collision)
INSTALL(TARGETS collision DESTINATION lib)
