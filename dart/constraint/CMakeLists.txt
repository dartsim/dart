# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_constraint_hdrs ${hdrs} PARENT_SCOPE)
set(dart_constraint_srcs ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_constraint ${srcs} ${hdrs})
#target_link_libraries(
#  dart_constraint
#  dart_collision
#  dart_lcpsolver
#  ${KIDO_CORE_DEPENDENCIES}
#)

# Generate header for this namespace
dart_get_filename_components(header_names "constraint headers" ${hdrs})
dart_generate_include_header_list(
  constraint_headers
  "dart/constraint/"
  "constraint headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/constraint.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/constraint.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/constraint.h
  DESTINATION include/dart/constraint
  COMPONENT headers
)
#install(TARGETS dart_constraint EXPORT DARTCoreTargets DESTINATION lib)
#install(TARGETS dart_constraint EXPORT DARTTargets DESTINATION lib)

