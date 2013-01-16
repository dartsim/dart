###############################################
# integration
project(integration)

# Search all header and source files
file(GLOB integration_srcs "*.cpp")
file(GLOB integration_hdrs "*.h")

# Library 
add_library(integration ${integration_srcs} ${integration_hdrs})
set_target_properties(integration PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties(integration PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
  set_target_properties(integration PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(FILES ${integration_hdrs} DESTINATION include/dart/integration)
install(FILES ${integration_hpps} DESTINATION include/dart/integration)
install(TARGETS integration DESTINATION lib)

list(APPEND DART_LIBRARIES integration)