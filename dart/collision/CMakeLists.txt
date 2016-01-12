# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Add subdirectories
add_subdirectory(dart)
add_subdirectory(fcl)
add_subdirectory(fcl_mesh)
if(HAVE_BULLET_COLLISION)
  add_subdirectory(bullet)
endif()

set(kido_collision_hdrs ${hdrs} ${kido_collision_hdrs} PARENT_SCOPE)
set(kido_collision_srcs ${srcs} ${kido_collision_srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_collision ${srcs} ${hdrs})
#target_link_libraries(
#  kido_collision
#  kido_common
#  kido_math
#  kido_collision_dart
#  kido_collision_fcl
#  kido_collision_fcl_mesh
#  ${KIDO_CORE_DEPENDENCIES}
#)
#if(HAVE_BULLET_COLLISION)
#  target_link_libraries(kido_collision kido_collision_bullet)
#endif()

# Generate header for this namespace
kido_get_filename_components(header_names "collision headers" ${hdrs})
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
kido_generate_include_header_list(
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
#install(TARGETS kido_collision EXPORT KIDOCoreTargets DESTINATION lib)
#install(TARGETS kido_collision EXPORT KIDOTargets DESTINATION lib)
