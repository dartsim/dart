# Search all header and source files
file(GLOB hdrs "*.h")

# Install
if(NOT DART_BUILD_DARTPY)
  install(
    FILES ${hdrs}
    DESTINATION include/dart/external/ikfast
    COMPONENT headers
  )
endif()
