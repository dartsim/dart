# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

dart_add_utils_headers(${hdrs})
dart_add_utils_headers(${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "utils_sdf headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME sdf
  TARGET_DIR "dart/utils/sdf/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/all.hpp ${CMAKE_CURRENT_BINARY_DIR}/sdf.hpp
  DESTINATION include/dart/utils/sdf
  COMPONENT headers
)
