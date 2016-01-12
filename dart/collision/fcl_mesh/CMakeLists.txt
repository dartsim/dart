# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(kido_collision_hdrs ${kido_collision_hdrs} ${hdrs} PARENT_SCOPE)
set(kido_collision_srcs ${kido_collision_srcs} ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_collision_fcl_mesh ${srcs} ${hdrs})
#target_link_libraries(kido_collision_fcl_mesh ${KIDO_CORE_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "collision_fcl_mesh headers" ${hdrs})
kido_generate_include_header_list(
  collision_fcl_mesh_headers
  "dart/collision/fcl_mesh/"
  "collision_fcl_mesh headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/fcl_mesh.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/fcl_mesh.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/fcl_mesh.h
  DESTINATION include/dart/collision/fcl_mesh
  COMPONENT headers
)
#install(TARGETS kido_collision_fcl_mesh EXPORT DARTCoreTargets DESTINATION lib)
#install(TARGETS kido_collision_fcl_mesh EXPORT DARTTargets DESTINATION lib)

