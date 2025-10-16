# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "integration headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME integration
  TARGET_DIR "dart/integration/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs} ${detail_hdrs}
)

# Install (includes all.hpp, integration.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/all.hpp ${CMAKE_CURRENT_BINARY_DIR}/integration.hpp ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION include/dart/integration
  COMPONENT headers
)
