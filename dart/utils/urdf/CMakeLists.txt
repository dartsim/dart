# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_utils_hdrs ${dart_utils_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_utils_srcs ${dart_utils_srcs} ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_utils_urdf ${srcs} ${hdrs})
#target_link_libraries(dart_utils_urdf ${KIDO_DEPENDENCIES})

# Generate header for this namespace
dart_get_filename_components(header_names "utils_urdf headers" ${hdrs})
dart_generate_include_header_list(
  utils_urdf_headers
  "dart/utils/urdf/"
  "utils_urdf headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/urdf.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/urdf.h
  DESTINATION include/dart/utils/urdf
  COMPONENT headers
)
#install(TARGETS dart_utils_urdf EXPORT DARTTargets DESTINATION lib)

