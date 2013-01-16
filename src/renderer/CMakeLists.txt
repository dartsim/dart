###############################################
# renderer
project(renderer)

# Search all header and source files
file(GLOB renderer_srcs "*.cpp")
file(GLOB renderer_hdrs "*.h")

# Library 
add_library(renderer ${renderer_srcs} ${renderer_hdrs})
target_link_libraries(renderer assimp)
set_target_properties(renderer PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties(renderer PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
  set_target_properties(renderer PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(FILES ${renderer_hdrs} DESTINATION include/dart/renderer)
install(TARGETS renderer DESTINATION lib)

list(APPEND DART_LIBRARIES renderer)