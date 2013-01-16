###############################################
# robotics
project(robotics)

# Search all header and source files
file(GLOB robotics_srcs "*.cpp")
file(GLOB robotics_hdrs "*.h")
file(GLOB robotics_hpps "*.hpp")

# Library
include_directories(${GLUT_INCLUDE_DIRS})
add_library(robotics ${robotics_srcs} ${robotics_hdrs} ${robotics_hpps})
target_link_libraries(robotics utils tinyxml2static assimp integration lcpsolver dynamics ${Boost_LIBRARIES})
link_thirdparty_glut(robotics )
set_target_properties(robotics PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties(robotics PROPERTIES ARCHIVE_OUTPUT_DIRECTORY  "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
  set_target_properties(robotics PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(FILES ${robotics_hdrs} DESTINATION include/dart/robotics)
install(FILES ${robotics_hpps} DESTINATION include/dart/robotics)
install(TARGETS robotics DESTINATION lib)

list(APPEND DART_LIBRARIES robotics)