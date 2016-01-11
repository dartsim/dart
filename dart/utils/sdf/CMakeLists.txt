# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_utils_hdrs ${dart_utils_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_utils_srcs ${dart_utils_srcs} ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_utils_sdf ${srcs} ${hdrs})
#target_link_libraries(dart_utils_sdf ${KIDO_DEPENDENCIES})

# Generate header for this namespace
dart_get_filename_components(header_names "utils_sdf headers" ${hdrs})
dart_generate_include_header_list(
  utils_sdf_headers
  "dart/utils/sdf/"
  "utils_sdf headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/sdf.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/sdf.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/sdf.h
  DESTINATION include/dart/utils/sdf
  COMPONENT headers
)
#install(TARGETS dart_utils_sdf EXPORT DARTTargets DESTINATION lib)

