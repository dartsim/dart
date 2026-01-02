# Search adapter headers and sources only
set(hdrs
  OdeCollisionDetector.hpp
  OdeCollisionGroup.hpp
  OdeCollisionObject.hpp
  OdeStub.hpp
)
set(srcs
  OdeCollisionDetector.cpp
  OdeCollisionGroup.cpp
  OdeCollisionObject.cpp
)

dart_add_core_headers(${hdrs})
dart_add_core_sources(${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_ode headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME ode
  TARGET_DIR "dart/collision/ode/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/ode.hpp
  DESTINATION include/dart/collision/ode
  COMPONENT headers
)

dart_install_compat_headers(COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS} DESTINATION_PREFIX include/dart/collision/ode)

dart_format_add(${hdrs} ${srcs})
