# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")
file(GLOB detail_srcs "detail/*.cpp")
file(GLOB detail_hdrs "detail/*.h")

set(dart_common_hdrs "${hdrs};${detail_hdrs}" PARENT_SCOPE)
set(dart_common_srcs "${srcs};${detail_srcs}" PARENT_SCOPE)

# Generate header for this namespace
dart_get_filename_components(header_names "common headers" ${hdrs})
dart_generate_include_header_list(
  common_headers
  "dart/common/"
  "common headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/common.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/common.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/common.h
  DESTINATION include/dart/common
  COMPONENT headers
)

install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/common/detail
  COMPONENT headers
)
