# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

include_directories(SYSTEM ${IPOPT_INCLUDE_DIRS})

# Library
kido_add_library(kido-optimizer-ipopt ${srcs} ${hdrs})
target_link_libraries(kido-optimizer-ipopt kido ${IPOPT_LIBRARIES})

# Generate header for this namespace
kido_get_filename_components(header_names "optimizer_ipopt headers" ${hdrs})
kido_generate_include_header_list(
  optimizer_ipopt_headers
  "kido/optimizer/ipopt/"
  "optimizer_ipopt headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ipopt.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/ipopt.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/ipopt.hpp
  DESTINATION include/kido/optimizer/ipopt
  COMPONENT headers
)
install(TARGETS kido-optimizer-ipopt EXPORT KIDOTargets DESTINATION lib)
