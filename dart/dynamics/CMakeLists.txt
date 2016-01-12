# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")
file(GLOB detail "detail/*.h")

set(kido_dynamics_hdrs "${hdrs};${detail}" PARENT_SCOPE)
set(kido_dynamics_srcs ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_dynamics ${srcs} ${hdrs})
#target_link_libraries(
#  kido_dynamics
#  kido_common
#  kido_math
#  ${KIDO_CORE_DEPENDENCIES}
#)

# Generate header for this namespace
kido_get_filename_components(header_names "dynamics headers" ${hdrs})
kido_generate_include_header_list(
  dynamics_headers
  "kido/dynamics/"
  "dynamics headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamics.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/dynamics.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/dynamics.h
  DESTINATION include/kido/dynamics
  COMPONENT headers
)

install(
  FILES ${detail}
  DESTINATION include/kido/dynamics/detail
  COMPONENT headers
)

#install(TARGETS kido_dynamics EXPORT KIDOCoreTargets DESTINATION lib)
#install(TARGETS kido_dynamics EXPORT KIDOTargets DESTINATION lib)
