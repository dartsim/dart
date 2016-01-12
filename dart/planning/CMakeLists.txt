# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(kido_planning_hdrs ${hdrs} PARENT_SCOPE)
set(kido_planning_srcs ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_planning ${srcs} ${hdrs})
#target_link_libraries(kido_planning ${KIDO_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "planning headers" ${hdrs})
kido_generate_include_header_list(
  planning_headers
  "dart/planning/"
  "planning headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/planning.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/planning.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/planning.h
  DESTINATION include/dart/planning
  COMPONENT headers
)
#install(TARGETS kido_planning EXPORT KIDOTargets DESTINATION lib)

