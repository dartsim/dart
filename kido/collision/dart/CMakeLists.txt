# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(kido_collision_hdrs ${kido_collision_hdrs} ${hdrs} PARENT_SCOPE)
set(kido_collision_srcs ${kido_collision_srcs} ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_collision_kido ${srcs} ${hdrs})
#target_link_libraries(kido_collision_kido ${KIDO_CORE_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "collision_kido headers" ${hdrs})
kido_generate_include_header_list(
  collision_kido_headers
  "kido/collision/kido/"
  "collision_kido headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/kido.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/kido.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/kido.h
  DESTINATION include/kido/collision/kido
  COMPONENT headers
)
#install(TARGETS kido_collision_kido EXPORT KIDOCoreTargets DESTINATION lib)
#install(TARGETS kido_collision_kido EXPORT KIDOTargets DESTINATION lib)

