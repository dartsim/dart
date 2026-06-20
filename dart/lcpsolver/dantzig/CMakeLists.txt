file(GLOB hdrs "*.hpp" "*.h")
file(GLOB srcs "*.cpp")

dart_add_core_headers(${hdrs})
dart_add_core_sources(${srcs})

install(
  FILES ${hdrs}
  DESTINATION include/dart/lcpsolver/dantzig
  COMPONENT headers
)
