# Compatibility headers that forward to dart/math/lcp
file(GLOB compat_hdrs "*.hpp" "*.h")
file(GLOB compat_dantzig_hdrs "dantzig/*.h" "dantzig/*.hpp")
file(GLOB compat_detail_hdrs "detail/*.hpp")

dart_add_core_headers(${compat_hdrs} ${compat_dantzig_hdrs} ${compat_detail_hdrs})

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
