# Gather public headers and sources
file(GLOB math_lcp_hdrs "*.hpp")
file(GLOB math_lcp_srcs "*.cpp")
dart_add_core_headers(${math_lcp_hdrs})
dart_add_core_sources(${math_lcp_srcs})

# Dantzig implementation headers and sources (remain public API)
file(GLOB math_lcp_dantzig_hdrs "Dantzig/*.hpp")
file(GLOB math_lcp_dantzig_srcs "Dantzig/*.cpp")
dart_add_core_headers(${math_lcp_dantzig_hdrs})
dart_add_core_sources(${math_lcp_dantzig_srcs})

# Expose header lists to parent math/CMakeLists for component generation
set(DART_MATH_LCP_PUBLIC_HEADERS ${math_lcp_hdrs} PARENT_SCOPE)
set(DART_MATH_LCP_DETAIL_HEADERS ${math_lcp_dantzig_hdrs} PARENT_SCOPE)

# Install new headers under dart/math/lcp
install(
  FILES ${math_lcp_hdrs}
  DESTINATION include/dart/math/lcp
  COMPONENT headers
)

install(
  FILES ${math_lcp_dantzig_hdrs}
  DESTINATION include/dart/math/lcp/Dantzig
  COMPONENT headers
)
