###############################################
# Kinematics
PROJECT(kinematics)

# Search all header and source files
FILE(GLOB kinematics_srcs "*.cpp")
FILE(GLOB kinematics_hdrs "*.h")
FILE(GLOB kinematics_hpps "*.hpp")

# Library 
ADD_LIBRARY(kinematics ${kinematics_srcs} ${kinematics_hdrs} ${kinematics_hpps})
TARGET_LINK_LIBRARIES(kinematics utils tinyxml2 assimp)
SET_TARGET_PROPERTIES(kinematics PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
SET_TARGET_PROPERTIES(kinematics PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")

INSTALL(FILES ${kinematics_hdrs} DESTINATION include/dart/kinematics)
INSTALL(FILES ${kinematics_hpps} DESTINATION include/dart/kinematics)
INSTALL(TARGETS kinematics DESTINATION lib)
