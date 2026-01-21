# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "constraint headers" ${hdrs})
list(REMOVE_ITEM header_names smart_pointer.hpp)
dart_generate_component_headers(
  COMPONENT_NAME constraint
  TARGET_DIR "dart/constraint/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs} ${detail_hdrs}
)

# Install (includes All.hpp, constraint.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/constraint.hpp
  DESTINATION include/dart/constraint
  COMPONENT headers
)
# Install generated compat headers while preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/constraint
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/constraint/detail
  COMPONENT headers
)
