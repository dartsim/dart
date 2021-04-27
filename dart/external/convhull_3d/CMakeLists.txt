# Search all header and source files
file(GLOB hdrs "*.h")

# Install
install(
  FILES ${hdrs}
  DESTINATION include/dart/external/convhull_3d
  COMPONENT headers
)
