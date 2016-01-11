# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_lcpsolver_hdrs ${hdrs} PARENT_SCOPE)
set(dart_lcpsolver_srcs ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_lcpsolver ${srcs} ${hdrs})
#target_link_libraries(dart_lcpsolver ${KIDO_CORE_DEPENDENCIES})

# Generate header for this namespace
dart_get_filename_components(header_names "lcpsolver headers" ${hdrs})
dart_generate_include_header_list(
  lcpsolver_headers
  "dart/lcpsolver/"
  "lcpsolver headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/lcpsolver.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/lcpsolver.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/lcpsolver.h
  DESTINATION include/dart/lcpsolver
  COMPONENT headers
)
#install(TARGETS dart_lcpsolver EXPORT DARTCoreTargets DESTINATION lib)
#install(TARGETS dart_lcpsolver EXPORT DARTTargets DESTINATION lib)

