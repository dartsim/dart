###############################################
# Collision
project(collision)

# Search all header and source files
file(GLOB collision_srcs "*.cpp")
file(GLOB collision_hdrs "*.h")

# Library
find_package(Boost REQUIRED system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_library(collision ${collision_srcs} ${collision_hdrs})
target_link_libraries(collision kinematics fcl ccd ${Boost_LIBRARIES})
set_target_properties(collision PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties(collision PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
  set_target_properties(collision PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(FILES ${collision_hdrs} DESTINATION include/dart/collision)
install(TARGETS collision DESTINATION lib)

list(APPEND DART_LIBRARIES collision)