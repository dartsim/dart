# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_integration_hdrs ${hdrs} PARENT_SCOPE)
set(dart_integration_srcs ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_integration ${srcs} ${hdrs})
#target_link_libraries(dart_integration ${KIDO_CORE_DEPENDENCIES})

# Generate header for this namespace
dart_get_filename_components(header_names "integration headers" ${hdrs})
dart_generate_include_header_list(
  integration_headers
  "dart/integration/"
  "integration headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/integration.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/integration.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/integration.h
  DESTINATION include/dart/integration
  COMPONENT headers
)
#install(TARGETS dart_integration EXPORT DARTCoreTargets DESTINATION lib)
#install(TARGETS dart_integration EXPORT DARTTargets DESTINATION lib)

