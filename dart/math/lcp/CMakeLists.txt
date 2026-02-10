# Gather public headers and sources
file(GLOB math_lcp_hdrs "*.hpp")
file(GLOB math_lcp_srcs "*.cpp")
dart_add_core_headers(${math_lcp_hdrs})
dart_add_core_sources(${math_lcp_srcs})

# Pivoting methods (Lemke, Dantzig wrappers, etc.)
file(GLOB math_lcp_pivoting_hdrs "pivoting/*.hpp")
file(GLOB math_lcp_pivoting_srcs "pivoting/*.cpp")
file(GLOB math_lcp_pivoting_dantzig_hdrs "pivoting/dantzig/*.hpp")
file(GLOB math_lcp_pivoting_dantzig_srcs "pivoting/dantzig/*.cpp")
dart_add_core_headers(${math_lcp_pivoting_hdrs})
dart_add_core_sources(${math_lcp_pivoting_srcs})
dart_add_core_headers(${math_lcp_pivoting_dantzig_hdrs})
dart_add_core_sources(${math_lcp_pivoting_dantzig_srcs})

# Projection methods (PGS, PSOR, BGS, etc. - future)
file(GLOB math_lcp_projection_hdrs "projection/*.hpp")
file(GLOB math_lcp_projection_srcs "projection/*.cpp")
dart_add_core_headers(${math_lcp_projection_hdrs})
dart_add_core_sources(${math_lcp_projection_srcs})

# Newton methods (future)
file(GLOB math_lcp_newton_hdrs "newton/*.hpp")
file(GLOB math_lcp_newton_srcs "newton/*.cpp")
dart_add_core_headers(${math_lcp_newton_hdrs})
dart_add_core_sources(${math_lcp_newton_srcs})

# Other methods (interior point, etc. - future)
file(GLOB math_lcp_other_hdrs "other/*.hpp")
file(GLOB math_lcp_other_srcs "other/*.cpp")
dart_add_core_headers(${math_lcp_other_hdrs})
dart_add_core_sources(${math_lcp_other_srcs})

# Compute relative header names (e.g., lcp/pivoting/lemke_solver.hpp, lcp/dantzig/common.hpp)
set(math_lcp_header_names "")
set(math_lcp_public_hdrs
  ${math_lcp_hdrs}
  ${math_lcp_pivoting_hdrs}
  ${math_lcp_pivoting_dantzig_hdrs}
  ${math_lcp_projection_hdrs}
  ${math_lcp_newton_hdrs}
  ${math_lcp_other_hdrs}
)

foreach(header ${math_lcp_public_hdrs})
  file(RELATIVE_PATH rel_path "${CMAKE_CURRENT_SOURCE_DIR}" ${header})
  list(APPEND math_lcp_header_names "lcp/${rel_path}")
endforeach()

# Expose header lists to parent math/CMakeLists for component generation
set(DART_MATH_LCP_PUBLIC_HEADERS ${math_lcp_public_hdrs} PARENT_SCOPE)
set(DART_MATH_LCP_PUBLIC_HEADER_NAMES ${math_lcp_header_names} PARENT_SCOPE)
set(DART_MATH_LCP_DETAIL_HEADERS ${math_lcp_pivoting_dantzig_hdrs} PARENT_SCOPE)

# Install new headers under dart/math/lcp
install(
  FILES ${math_lcp_hdrs}
  DESTINATION include/dart/math/lcp
  COMPONENT headers
)

if(math_lcp_pivoting_dantzig_hdrs)
  install(
    FILES ${math_lcp_pivoting_dantzig_hdrs}
    DESTINATION include/dart/math/lcp/pivoting/dantzig
    COMPONENT headers
  )
endif()

if(math_lcp_pivoting_hdrs)
  install(
    FILES ${math_lcp_pivoting_hdrs}
    DESTINATION include/dart/math/lcp/pivoting
    COMPONENT headers
  )
endif()

if(math_lcp_projection_hdrs)
  install(
    FILES ${math_lcp_projection_hdrs}
    DESTINATION include/dart/math/lcp/projection
    COMPONENT headers
  )
endif()

if(math_lcp_newton_hdrs)
  install(
    FILES ${math_lcp_newton_hdrs}
    DESTINATION include/dart/math/lcp/newton
    COMPONENT headers
  )
endif()

if(math_lcp_other_hdrs)
  install(
    FILES ${math_lcp_other_hdrs}
    DESTINATION include/dart/math/lcp/other
    COMPONENT headers
  )
endif()
