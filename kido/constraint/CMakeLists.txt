# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(kido_constraint_hdrs ${hdrs} PARENT_SCOPE)
set(kido_constraint_srcs ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_constraint ${srcs} ${hdrs})
#target_link_libraries(
#  kido_constraint
#  kido_collision
#  kido_lcpsolver
#  ${KIDO_CORE_DEPENDENCIES}
#)

# Generate header for this namespace
kido_get_filename_components(header_names "constraint headers" ${hdrs})
kido_generate_include_header_list(
  constraint_headers
  "kido/constraint/"
  "constraint headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/constraint.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/constraint.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/constraint.hpp
  DESTINATION include/kido/constraint
  COMPONENT headers
)
#install(TARGETS kido_constraint EXPORT KIDOCoreTargets DESTINATION lib)
#install(TARGETS kido_constraint EXPORT KIDOTargets DESTINATION lib)

