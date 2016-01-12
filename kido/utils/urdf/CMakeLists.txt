# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(kido_utils_hdrs ${kido_utils_hdrs} ${hdrs} PARENT_SCOPE)
set(kido_utils_srcs ${kido_utils_srcs} ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_utils_urdf ${srcs} ${hdrs})
#target_link_libraries(kido_utils_urdf ${KIDO_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "utils_urdf headers" ${hdrs})
kido_generate_include_header_list(
  utils_urdf_headers
  "kido/utils/urdf/"
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
  DESTINATION include/kido/utils/urdf
  COMPONENT headers
)
#install(TARGETS kido_utils_urdf EXPORT KIDOTargets DESTINATION lib)

