# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

dart_add_utils_headers(${hdrs})
dart_add_utils_sources(${srcs})

set(
  DART_UTILS_BACKWARDCOMPATIBILITY_HPP_IN
  ${CMAKE_CURRENT_SOURCE_DIR}/BackwardCompatibility.hpp.in
)
set(
  DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT
  ${CMAKE_CURRENT_BINARY_DIR}/BackwardCompatibility.hpp
)
if(DART_VERBOSE)
  message(STATUS ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT})
endif()
configure_file(
  ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_IN}
  ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT}
  @ONLY
)
install(
  FILES ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT}
  DESTINATION include/dart/utils/urdf
)
# Also in the headers component; the rule above keeps the default component's
# contents unchanged.
install(
  FILES ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT}
  DESTINATION include/dart/utils/urdf
  COMPONENT headers
  EXCLUDE_FROM_ALL
)

# Generate header for this namespace
dart_get_filename_components(header_names "utils_urdf headers" ${hdrs})
# TODO: remove below line once the files are completely removed.
list(REMOVE_ITEM header_names "URDFTypes.hpp")
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/urdf.hpp"
  "dart/utils/urdf/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/urdf.hpp
  DESTINATION include/dart/utils/urdf
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
