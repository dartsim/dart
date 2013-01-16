###############################################
# planning
project(planning)

# Search all header and source files
file(GLOB planning_srcs "*.cpp")
file(GLOB planning_hdrs "*.h")
file(GLOB planning_hpps "*.hpp")
file(GLOB gl_srcs GL/GLTexture.cpp)
file(GLOB gl_hdrs GL/*.h)

# Library
include_directories(${GLUT_INCLUDE_DIRS})
add_library(planning ${planning_srcs} ${gl_srcs} ${planning_hdrs} ${planning_hpps})
target_link_libraries(planning utils)
set_target_properties(planning PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )
set_target_properties(planning PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib" )
if(MSVC)
  set_target_properties(planning PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(FILES ${planning_hdrs} DESTINATION include/dart/planning)
install(FILES ${planning_hpps} DESTINATION include/dart/planning)
install(FILES ${gl_hdrs} DESTINATION include/dart/planning/GL)
install(TARGETS planning DESTINATION lib)

list(APPEND DART_LIBRARIES planning)