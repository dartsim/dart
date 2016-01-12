# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(kido_lcpsolver_hdrs ${hdrs} PARENT_SCOPE)
set(kido_lcpsolver_srcs ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_lcpsolver ${srcs} ${hdrs})
#target_link_libraries(kido_lcpsolver ${KIDO_CORE_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "lcpsolver headers" ${hdrs})
kido_generate_include_header_list(
  lcpsolver_headers
  "kido/lcpsolver/"
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
  DESTINATION include/kido/lcpsolver
  COMPONENT headers
)
#install(TARGETS kido_lcpsolver EXPORT KIDOCoreTargets DESTINATION lib)
#install(TARGETS kido_lcpsolver EXPORT KIDOTargets DESTINATION lib)

