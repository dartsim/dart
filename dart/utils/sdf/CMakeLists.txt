# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(kido_utils_hdrs ${kido_utils_hdrs} ${hdrs} PARENT_SCOPE)
set(kido_utils_srcs ${kido_utils_srcs} ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_utils_sdf ${srcs} ${hdrs})
#target_link_libraries(kido_utils_sdf ${KIDO_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "utils_sdf headers" ${hdrs})
kido_generate_include_header_list(
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
#install(TARGETS kido_utils_sdf EXPORT KIDOTargets DESTINATION lib)

