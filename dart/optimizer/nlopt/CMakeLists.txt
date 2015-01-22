# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_optimizer_hdrs ${dart_optimizer_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_optimizer_srcs ${dart_optimizer_srcs} ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_optimizer_nlopt ${srcs} ${hdrs})
#target_link_libraries(dart_optimizer_nlopt ${DART_DEPENDENCIES})

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer_nlopt headers" ${hdrs})
dart_generate_include_header_list(
  optimizer_nlopt_headers
  "dart/optimizer/nlopt/"
  "optimizer_nlopt headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/nlopt.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/nlopt.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/nlopt.h
  DESTINATION include/dart/optimizer/nlopt
  COMPONENT headers
)
#install(TARGETS dart_optimizer_nlopt EXPORT DARTTargets DESTINATION lib)

