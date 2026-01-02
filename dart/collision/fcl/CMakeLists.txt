# Search adapter headers and sources only
set(hdrs
  BackwardCompatibility.hpp
  FCLCollisionDetector.hpp
  FCLCollisionGroup.hpp
  FCLCollisionObject.hpp
)
set(srcs
  BackwardCompatibility.cpp
  FCLCollisionDetector.cpp
  FCLCollisionGroup.cpp
  FCLCollisionObject.cpp
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
