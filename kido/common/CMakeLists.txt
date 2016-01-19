# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")

set(kido_common_hdrs "${hdrs};${detail_hdrs}" PARENT_SCOPE)
set(kido_common_srcs "${srcs};${detail_srcs}" PARENT_SCOPE)

# Library
#kido_add_library(kido_common ${srcs} ${hdrs})
#target_link_libraries(kido_common ${KIDO_CORE_DEPENDENCIES})

# Generate header for this namespace
kido_get_filename_components(header_names "common headers" ${hdrs})
kido_generate_include_header_list(
  common_headers
  "kido/common/"
  "common headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/common.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/common.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/common.hpp
  DESTINATION include/kido/common
  COMPONENT headers
)

install(
  FILES ${detail_hdrs}
  DESTINATION include/kido/common/detail
  COMPONENT headers
)

#install(TARGETS kido_common EXPORT KIDOCoreTargets DESTINATION lib)
#install(TARGETS kido_common EXPORT KIDOTargets DESTINATION lib)

