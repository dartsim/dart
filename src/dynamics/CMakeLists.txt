###############################################
# Dynamics
project(dynamics)

# Search all header and source files
file(GLOB dynamics_srcs "*.cpp")
file(GLOB dynamics_hdrs "*.h")

# Library 
add_library(dynamics ${dynamics_srcs} ${dynamics_hdrs})
target_link_libraries(dynamics lcpsolver collision)
set_target_properties(dynamics PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties(dynamics PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
  set_target_properties(dynamics PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(FILES ${dynamics_hdrs} DESTINATION include/dart/dynamics)
install(TARGETS dynamics DESTINATION lib)

list(APPEND DART_LIBRARIES dynamics)