# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs})
dart_add_core_sources(${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "sensor headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME sensor
  TARGET_DIR "dart/sensor/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install (includes All.hpp; the generated sensor.hpp component header is
# omitted to avoid colliding with the Sensor header).
install(FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp DESTINATION include/dart/sensor COMPONENT headers)

# Install generated PascalCase compatibility headers, preserving directory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/sensor
)
