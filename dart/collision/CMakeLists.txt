# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")
file(GLOB detail_srcs "detail/*.cpp")
file(GLOB detail_hdrs "detail/*.h")

# Add subdirectories
add_subdirectory(dart)
add_subdirectory(fcl)
add_subdirectory(fcl_mesh)
if(HAVE_BULLET_COLLISION)
  add_subdirectory(bullet)
endif()

set(dart_collision_hdrs ${hdrs} "${dart_collision_hdrs};${detail_hdrs}" PARENT_SCOPE)
set(dart_collision_srcs ${srcs} "${dart_collision_srcs};${detail_srcs}" PARENT_SCOPE)

# Library
#dart_add_library(dart_collision ${srcs} ${hdrs})
#target_link_libraries(
#  dart_collision
#  dart_common
#  dart_math
#  dart_collision_dart
#  dart_collision_fcl
#  dart_collision_fcl_mesh
#  ${DART_CORE_DEPENDENCIES}
#)
#if(HAVE_BULLET_COLLISION)
#  target_link_libraries(dart_collision dart_collision_bullet)
#endif()

# Generate header for this namespace
dart_get_filename_components(header_names "collision headers" ${hdrs})
set(
  header_names
  ${header_names}
  dart/dart.h
  fcl/fcl.h
  fcl_mesh/fcl_mesh.h
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
#install(TARGETS dart_collision EXPORT DARTCoreTargets DESTINATION lib)
#install(TARGETS dart_collision EXPORT DARTTargets DESTINATION lib)
