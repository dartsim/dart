# Search adapter headers and sources only
set(hdrs
  "${CMAKE_CURRENT_SOURCE_DIR}/BackwardCompatibility.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/FCLCollisionDetector.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/FCLCollisionGroup.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/FCLCollisionObject.hpp"
)
set(srcs
  "${CMAKE_CURRENT_SOURCE_DIR}/BackwardCompatibility.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/FCLCollisionDetector.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/FCLCollisionGroup.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/FCLCollisionObject.cpp"
)
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_fcl headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME fcl
  TARGET_DIR "dart/collision/fcl/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/fcl.hpp ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION include/dart/collision/fcl
  COMPONENT headers
)
