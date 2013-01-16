###############################################
# optimizer
project(optimizer)

# Search all header and source files
file(GLOB optimizer_srcs "*.cpp")
file(GLOB optimizer_hdrs "*.h")
file(GLOB optimizer_snopt_srcs "snopt/*.cpp")
file(GLOB optimizer_snopt_hdrs "snopt/*.h")

# Library 
add_library(optimizer ${optimizer_srcs} ${optimizer_hdrs} ${optimizer_snopt_srcs} ${optimizer_snopt_hdrs})
target_link_libraries(optimizer utils)
set_target_properties(optimizer PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties(optimizer PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
  set_target_properties(optimizer PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

list(APPEND DART_LIBRARIES optimizer)