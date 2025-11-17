# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

set(DART_MATH_LCP_PUBLIC_HEADERS "")
set(DART_MATH_LCP_DETAIL_HEADERS "")
add_subdirectory(lcp)

# Generate header for this namespace
set(math_component_headers ${hdrs} ${DART_MATH_LCP_PUBLIC_HEADERS})
dart_get_filename_components(header_names "math headers" ${math_component_headers})
dart_generate_component_headers(
  COMPONENT_NAME math
  TARGET_DIR "dart/math/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${math_component_headers} ${detail_hdrs} ${DART_MATH_LCP_DETAIL_HEADERS}
)

# Install (includes All.hpp, math.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/math.hpp
  DESTINATION include/dart/math
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/math/detail
  COMPONENT headers
)
# Install generated compatibility headers preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/math
)
