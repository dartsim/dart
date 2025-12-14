# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

dart_add_utils_headers(${hdrs} ${detail_hdrs})
dart_add_utils_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "utils_mjcf headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME mjcf
  TARGET_DIR "dart/utils/mjcf/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/mjcf.hpp
  DESTINATION include/dart/utils/mjcf
  COMPONENT headers
)
