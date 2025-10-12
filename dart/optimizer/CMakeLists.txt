# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME optimizer
  TARGET_DIR "dart/optimizer/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/all.hpp ${CMAKE_CURRENT_BINARY_DIR}/optimizer.hpp
  DESTINATION include/dart/optimizer
  COMPONENT headers
)

# Add subdirectories (components)
add_subdirectory(ipopt)
add_subdirectory(nlopt)
add_subdirectory(pagmo)
add_subdirectory(snopt)
