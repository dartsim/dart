###############################################
# Kinematics
project(kinematics)

# Search all header and source files
file(GLOB kinematics_srcs "*.cpp")
file(GLOB kinematics_hdrs "*.h")
file(GLOB kinematics_hpps "*.hpp")

# Library 
add_library(kinematics ${kinematics_srcs} ${kinematics_hdrs} ${kinematics_hpps})
target_link_libraries(kinematics utils tinyxml2 assimp)
set_target_properties(kinematics PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties(kinematics PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
  set_target_properties(kinematics PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(FILES ${kinematics_hdrs} DESTINATION include/dart/kinematics)
install(FILES ${kinematics_hpps} DESTINATION include/dart/kinematics)
install(TARGETS kinematics DESTINATION lib)

list(APPEND DART_LIBRARIES kinematics)