# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(kido_collision_hdrs ${kido_collision_hdrs} ${hdrs} PARENT_SCOPE)
set(kido_collision_srcs ${kido_collision_srcs} ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_collision_fcl ${srcs} ${hdrs})
#target_link_libraries(kido_collision_fcl ${KIDO_CORE_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "collision_fcl headers" ${hdrs})
kido_generate_include_header_list(
  collision_fcl_headers
  "kido/collision/fcl/"
  "collision_fcl headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/fcl.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/fcl.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/fcl.hpp
  DESTINATION include/kido/collision/fcl
  COMPONENT headers
)
#install(TARGETS kido_collision_fcl EXPORT KIDOCoreTargets DESTINATION lib)
#install(TARGETS kido_collision_fcl EXPORT KIDOTargets DESTINATION lib)

