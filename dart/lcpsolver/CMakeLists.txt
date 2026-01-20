# Compatibility headers that forward to dart/math/lcp
file(GLOB compat_hdrs "*.hpp" "*.h")
file(GLOB compat_dantzig_hdrs "dantzig/*.h" "dantzig/*.hpp")
file(GLOB compat_detail_hdrs "detail/*.hpp")

dart_add_core_headers(${compat_hdrs} ${compat_dantzig_hdrs} ${compat_detail_hdrs})

dart_generate_case_compat_headers(
  HEADERS ${compat_hdrs} ${compat_dantzig_hdrs} ${compat_detail_hdrs}
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  COMPONENT_PATH "dart/lcpsolver"
  RELATIVE_TO "${CMAKE_CURRENT_SOURCE_DIR}"
)

install(
  FILES ${compat_hdrs}
  DESTINATION include/dart/lcpsolver
  COMPONENT headers
)

install(
  FILES ${compat_dantzig_hdrs}
  DESTINATION include/dart/lcpsolver/dantzig
  COMPONENT headers
)

install(
  FILES ${compat_detail_hdrs}
  DESTINATION include/dart/lcpsolver/detail
  COMPONENT headers
)

dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/lcpsolver
)
