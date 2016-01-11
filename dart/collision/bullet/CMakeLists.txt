# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_collision_hdrs ${dart_collision_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_collision_srcs ${dart_collision_srcs} ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_collision_bullet ${srcs} ${hdrs})
#target_link_libraries(dart_collision_bullet ${KIDO_CORE_DEPENDENCIES})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_bullet headers" ${hdrs})
dart_generate_include_header_list(
  collision_bullet_headers
  "dart/collision/bullet/"
  "collision_bullet headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bullet.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/bullet.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/bullet.h
  DESTINATION include/dart/collision/bullet
  COMPONENT headers
)
#install(TARGETS dart_collision_bullet EXPORT DARTCoreTargets DESTINATION lib)
#install(TARGETS dart_collision_bullet EXPORT DARTTargets DESTINATION lib)

