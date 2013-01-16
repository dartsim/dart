###############################################
# Utils
project(utils)

# Search all header and source files
file(GLOB utils_srcs "*.cpp")
file(GLOB utils_hdrs "*.h")

# Library 
add_library(utils ${utils_srcs} ${utils_hdrs})
set_target_properties(utils PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties(utils PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
  set_target_properties(utils PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(FILES ${utils_hdrs} DESTINATION include/dart/utils)
install(TARGETS utils DESTINATION lib)

list(APPEND DART_LIBRARIES utils)

add_subdirectory(Parser)