# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")
file(GLOB detail_srcs "detail/*.cpp")
file(GLOB detail_hdrs "detail/*.h")

# Add subdirectories
add_subdirectory(dart)
add_subdirectory(fcl)
if(HAVE_BULLET_COLLISION)
  add_subdirectory(bullet)
endif()

set(dart_collision_hdrs ${hdrs} "${dart_collision_hdrs};${detail_hdrs}" PARENT_SCOPE)
set(dart_collision_srcs ${srcs} "${dart_collision_srcs};${detail_srcs}" PARENT_SCOPE)

# Generate header for this namespace
dart_get_filename_components(header_names "collision headers" ${hdrs})
set(
  header_names
  ${header_names}
  dart/dart.h
  fcl/fcl.h
)
if(HAVE_BULLET_COLLISION)
  set(header_names ${header_names} bullet/bullet.h)
endif()
dart_generate_include_header_list(
  collision_headers
  "dart/collision/"
  "collision headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/collision.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/collision.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/collision.h
  DESTINATION include/dart/collision
  COMPONENT headers
)

install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/collision/detail
  COMPONENT headers
)
