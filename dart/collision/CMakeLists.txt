# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
file(GLOB dart_hdrs "dart/*.hpp")
file(GLOB experimental_backend_hdrs "experimental_backend/*.hpp")
file(GLOB fcl_hdrs "fcl/*.hpp")
file(GLOB ode_hdrs "ode/*.hpp")
file(GLOB ode_detail_hdrs "ode/detail/*.hpp")
file(GLOB bullet_hdrs "bullet/*.hpp")
file(GLOB bullet_detail_hdrs "bullet/detail/*.hpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "collision headers" ${hdrs})
list(REMOVE_ITEM header_names smart_pointer.hpp)
set(
  header_names
  ${header_names}
  dart/All.hpp
  fcl/All.hpp
)
if(DART_BUILD_COLLISION_EXPERIMENTAL)
  list(APPEND header_names experimental_backend/All.hpp)
endif()
set(collision_source_headers
  ${hdrs}
  ${detail_hdrs}
  ${dart_hdrs}
  ${fcl_hdrs}
  ${ode_hdrs}
  ${ode_detail_hdrs}
  ${bullet_hdrs}
  ${bullet_detail_hdrs}
)
if(DART_BUILD_COLLISION_EXPERIMENTAL)
  list(APPEND collision_source_headers ${experimental_backend_hdrs})
endif()
dart_generate_component_headers(
  COMPONENT_NAME collision
  TARGET_DIR "dart/collision/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${collision_source_headers}
)

# Install headers and generated component headers
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/collision.hpp
  DESTINATION include/dart/collision
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/collision/detail
  COMPONENT headers
)

# Install generated PascalCase compatibility headers, preserving directory structure
dart_install_compat_headers(COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS} DESTINATION_PREFIX include/dart/collision)

# Add subdirectories
add_subdirectory(dart)
if(DART_BUILD_COLLISION_EXPERIMENTAL)
  add_subdirectory(experimental_backend)
endif()
add_subdirectory(fcl)
add_subdirectory(ode)
add_subdirectory(bullet)
