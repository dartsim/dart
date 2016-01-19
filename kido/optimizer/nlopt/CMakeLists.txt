# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
kido_add_library(kido-optimizer-nlopt ${srcs} ${hdrs})
target_link_libraries(kido-optimizer-nlopt kido-core ${NLOPT_LIBRARIES})

# Generate header for this namespace
kido_get_filename_components(header_names "optimizer_nlopt headers" ${hdrs})
kido_generate_include_header_list(
  optimizer_nlopt_headers
  "kido/optimizer/nlopt/"
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
  DESTINATION include/kido/optimizer/nlopt
  COMPONENT headers
)
install(TARGETS kido-optimizer-nlopt EXPORT KIDOTargets DESTINATION lib)
