# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_optimizer_hdrs ${dart_optimizer_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_optimizer_srcs ${dart_optimizer_srcs} ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_optimizer_ipopt ${srcs} ${hdrs})
#target_link_libraries(dart_optimizer_ipopt ${DART_DEPENDENCIES})

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer_ipopt headers" ${hdrs})
dart_generate_include_header_list(
  optimizer_ipopt_headers
  "dart/optimizer/ipopt/"
  "optimizer_ipopt headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ipopt.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/ipopt.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/ipopt.h
  DESTINATION include/dart/optimizer/ipopt
  COMPONENT headers
)
#install(TARGETS dart_optimizer_ipopt EXPORT DARTTargets DESTINATION lib)

