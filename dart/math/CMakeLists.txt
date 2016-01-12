# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(kido_math_hdrs ${hdrs} PARENT_SCOPE)
set(kido_math_srcs ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_math ${srcs} ${hdrs})
#target_link_libraries(kido_math ${KIDO_CORE_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "math headers" ${hdrs})
kido_generate_include_header_list(
  math_headers
  "kido/math/"
  "math headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/math.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/math.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/math.h
  DESTINATION include/kido/math
  COMPONENT headers
)
#install(TARGETS kido_math EXPORT KIDOCoreTargets DESTINATION lib)
#install(TARGETS kido_math EXPORT KIDOTargets DESTINATION lib)

