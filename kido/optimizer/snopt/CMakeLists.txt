# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
kido_add_library(kido-optimizer-snopt ${srcs} ${hdrs})
target_link_libraries(kido-optimizer-snopt kido)

# Generate header for this namespace
kido_get_filename_components(header_names "optimizer_snopt headers" ${hdrs})
kido_generate_include_header_list(
  optimizer_snopt_headers
  "kido/optimizer/snopt/"
  "optimizer_snopt headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/snopt.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/snopt.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/snopt.h
  DESTINATION include/kido/optimizer/snopt
  COMPONENT headers
)
install(TARGETS kido-optimizer-snopt EXPORT KIDOSNOPTTargets DESTINATION lib)

