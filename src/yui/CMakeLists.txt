###############################################
# YUI
project(yui)

# Search all header and source files
file(GLOB yui_srcs "*.cpp")
file(GLOB yui_hdrs "*.h")

# Library 
add_library(yui ${yui_srcs} ${yui_hdrs})
set_target_properties(yui PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
  set_target_properties(yui PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

list(APPEND DART_LIBRARIES yui)