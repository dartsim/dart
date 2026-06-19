# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "math headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/math.hpp"
  "dart/math/"
  ${header_names}
)

set(convhull_compat_header_dir
  "${DART_BINARY_DIR}/dart/external/convhull_3d"
)
set(convhull_compat_headers
  "${convhull_compat_header_dir}/convhull_3d.h"
  "${convhull_compat_header_dir}/safe_convhull_3d.h"
)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/detail/convhull_3d_compat.h.in"
  "${convhull_compat_header_dir}/convhull_3d.h"
  COPYONLY
)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/detail/safe_convhull_3d_compat.h.in"
  "${convhull_compat_header_dir}/safe_convhull_3d.h"
  COPYONLY
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/math.hpp
  DESTINATION include/dart/math
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/math/detail
  COMPONENT headers
)
install(
  FILES ${convhull_compat_headers}
  DESTINATION include/dart/external/convhull_3d
  COMPONENT headers
)
