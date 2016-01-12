# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Add subdirectories
add_subdirectory(sdf)
add_subdirectory(urdf)

set(kido_utils_hdrs ${hdrs} ${kido_utils_hdrs} PARENT_SCOPE)
set(kido_utils_srcs ${srcs} ${kido_utils_srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_utils ${srcs} ${hdrs})
#target_link_libraries(
#  kido_utils
#  kido_utils_sdf
#  kido_utils_urdf
#  kido_collision
#  kido_dynamics
#  kido_simulation
#  ${KIDO_DEPENDENCIES}
#)

# Generate header for this namespace
kido_get_filename_components(header_names "utils headers" ${hdrs})
set(
  header_names
  ${header_names}
  sdf/sdf.h
  urdf/urdf.h
)
kido_generate_include_header_list(
  utils_headers
  "dart/utils/"
  "utils headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/utils.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/utils.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/utils.h
  DESTINATION include/dart/utils
  COMPONENT headers
)
#install(TARGETS kido_utils EXPORT KIDOTargets DESTINATION lib)

