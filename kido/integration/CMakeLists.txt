# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(kido_integration_hdrs ${hdrs} PARENT_SCOPE)
set(kido_integration_srcs ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_integration ${srcs} ${hdrs})
#target_link_libraries(kido_integration ${KIDO_CORE_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "integration headers" ${hdrs})
kido_generate_include_header_list(
  integration_headers
  "kido/integration/"
  "integration headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/integration.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/integration.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/integration.hpp
  DESTINATION include/kido/integration
  COMPONENT headers
)
#install(TARGETS kido_integration EXPORT KIDOCoreTargets DESTINATION lib)
#install(TARGETS kido_integration EXPORT KIDOTargets DESTINATION lib)

