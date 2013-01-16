###############################################
# lcpsolver
project(lcpsolver)

# Search all header and source files
file(GLOB lcpsolver_srcs "*.cpp")
file(GLOB lcpsolver_hdrs "*.h")

# Library 
add_library(lcpsolver ${lcpsolver_srcs} ${lcpsolver_hdrs})
set_target_properties(lcpsolver PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties(lcpsolver PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
  set_target_properties(lcpsolver PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(FILES ${lcpsolver_hdrs} DESTINATION include/dart/lcpsolver)
install(FILES ${lcpsolver_hpps} DESTINATION include/dart/lcpsolver)
install(TARGETS lcpsolver DESTINATION lib)

list(APPEND DART_LIBRARIES lcpsolver)