# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "constraint headers" ${hdrs})
dart_generate_include_header_list(
  constraint_headers
  "dart/constraint/"
  "constraint headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/constraint.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/constraint.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/constraint.hpp
  DESTINATION include/dart/constraint
  COMPONENT headers
)
