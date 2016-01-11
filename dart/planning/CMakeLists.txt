# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_planning_hdrs ${hdrs} PARENT_SCOPE)
set(dart_planning_srcs ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_planning ${srcs} ${hdrs})
#target_link_libraries(dart_planning ${KIDO_DEPENDENCIES})

# Generate header for this namespace
dart_get_filename_components(header_names "planning headers" ${hdrs})
dart_generate_include_header_list(
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
#install(TARGETS dart_planning EXPORT DARTTargets DESTINATION lib)

