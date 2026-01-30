# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "common headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME common
  TARGET_DIR "dart/common/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs} ${detail_hdrs}
)

# Install headers and generated component headers
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/common.hpp
  DESTINATION include/dart/common
  COMPONENT headers
)
install(FILES ${detail_hdrs} DESTINATION include/dart/common/detail COMPONENT headers)

# Install generated PascalCase compatibility headers, preserving directory structure
dart_install_compat_headers(COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS} DESTINATION_PREFIX include/dart/common)
