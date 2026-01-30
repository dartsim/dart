# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
file(GLOB opt_hdrs "optimization/*.hpp")
file(GLOB opt_srcs "optimization/*.cpp")

dart_add_core_headers(${hdrs} ${detail_hdrs} ${opt_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs} ${opt_srcs})

set(DART_MATH_LCP_PUBLIC_HEADERS "")
set(DART_MATH_LCP_PUBLIC_HEADER_NAMES "")
set(DART_MATH_LCP_DETAIL_HEADERS "")
add_subdirectory(lcp)

# Generate header for this namespace
set(math_component_headers ${hdrs} ${DART_MATH_LCP_PUBLIC_HEADERS})
dart_get_filename_components(header_names "math headers" ${hdrs})
list(APPEND header_names ${DART_MATH_LCP_PUBLIC_HEADER_NAMES})
dart_generate_component_headers(
  COMPONENT_NAME math
  TARGET_DIR "dart/math/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${math_component_headers} ${detail_hdrs} ${DART_MATH_LCP_DETAIL_HEADERS}
)

# Generate headers for optimization sub-namespace
if(opt_hdrs)
  dart_get_filename_components(opt_header_names "math optimization headers" ${opt_hdrs})
  dart_generate_component_headers(
    COMPONENT_NAME optimization
    TARGET_DIR "dart/math/optimization/"
    OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/optimization"
    HEADERS ${opt_header_names}
    SOURCE_HEADERS ${opt_hdrs}
  )
endif()

# Install (includes All.hpp, math.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/math.hpp
  DESTINATION include/dart/math
  COMPONENT headers
)
install(FILES ${detail_hdrs} DESTINATION include/dart/math/detail COMPONENT headers)
if(opt_hdrs)
  install(
    FILES ${opt_hdrs} ${CMAKE_CURRENT_BINARY_DIR}/optimization/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/optimization/optimization.hpp
    DESTINATION include/dart/math/optimization
    COMPONENT headers
  )
endif()

# Install generated compatibility headers preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/math
)
