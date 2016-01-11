# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Add subdirectories
add_subdirectory(sdf)
add_subdirectory(urdf)

set(dart_utils_hdrs ${hdrs} ${dart_utils_hdrs} PARENT_SCOPE)
set(dart_utils_srcs ${srcs} ${dart_utils_srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_utils ${srcs} ${hdrs})
#target_link_libraries(
#  dart_utils
#  dart_utils_sdf
#  dart_utils_urdf
#  dart_collision
#  dart_dynamics
#  dart_simulation
#  ${KIDO_DEPENDENCIES}
#)

# Generate header for this namespace
dart_get_filename_components(header_names "utils headers" ${hdrs})
set(
  header_names
  ${header_names}
  sdf/sdf.h
  urdf/urdf.h
)
dart_generate_include_header_list(
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
#install(TARGETS dart_utils EXPORT DARTTargets DESTINATION lib)

