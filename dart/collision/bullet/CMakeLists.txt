# Search adapter headers and sources only
set(hdrs
  BulletCollisionDetector.hpp
  BulletCollisionGroup.hpp
  BulletCollisionObject.hpp
  BulletCollisionShape.hpp
  BulletInclude.hpp
)
set(srcs
  BulletCollisionDetector.cpp
  BulletCollisionGroup.cpp
  BulletCollisionObject.cpp
  BulletCollisionShape.cpp
)

dart_add_core_headers(${hdrs})
dart_add_core_sources(${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_bullet headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME bullet
  TARGET_DIR "dart/collision/bullet/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/bullet.hpp
  DESTINATION include/dart/collision/bullet
  COMPONENT headers
)

dart_install_compat_headers(COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS} DESTINATION_PREFIX include/dart/collision/bullet)

dart_format_add(${hdrs} ${srcs})
