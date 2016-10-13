# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer headers" ${hdrs})
dart_generate_include_header_list(
  optimizer_headers
  "dart/optimizer/"
  "optimizer headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/optimizer.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/optimizer.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/optimizer.hpp
  DESTINATION include/dart/optimizer
  COMPONENT headers
)

# Add subdirectories (components)
if(HAVE_IPOPT)
  add_subdirectory(ipopt)
endif()
if(HAVE_NLOPT)
  add_subdirectory(nlopt)
endif()
if(HAVE_SNOPT)
  add_subdirectory(snopt)
endif()
