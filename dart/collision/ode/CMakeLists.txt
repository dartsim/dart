file(GLOB facade_hdrs "*.hpp")
file(GLOB compat_hdrs "compat/*.hpp")
dart_format_add(${facade_hdrs} ${compat_hdrs})

install(
  FILES ${facade_hdrs}
  DESTINATION include/dart/collision/ode
  COMPONENT headers
)
install(
  FILES ${compat_hdrs}
  DESTINATION include/dart/collision/ode/compat
  COMPONENT headers
)
