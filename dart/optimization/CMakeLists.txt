# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "optimization headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/optimization.hpp"
  "dart/optimization/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/optimization.hpp
  DESTINATION include/dart/optimization
  COMPONENT headers
)

# Add subdirectories (components)
add_subdirectory(ipopt)
add_subdirectory(nlopt)
add_subdirectory(pagmo)
add_subdirectory(snopt)
