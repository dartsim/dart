# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_integration_hdrs ${hdrs} PARENT_SCOPE)
set(dart_integration_srcs ${srcs} PARENT_SCOPE)

# Generate header for this namespace
dart_get_filename_components(header_names "integration headers" ${hdrs})
dart_generate_include_header_list(
  integration_headers
  "dart/integration/"
  "integration headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/integration.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/integration.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/integration.h
  DESTINATION include/dart/integration
  COMPONENT headers
)
