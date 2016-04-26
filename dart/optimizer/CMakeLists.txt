# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Add subdirectories
if(HAVE_IPOPT)
  add_subdirectory(ipopt)
endif()
if(HAVE_NLOPT)
  add_subdirectory(nlopt)
endif()
if(HAVE_SNOPT)
  add_subdirectory(snopt)
endif()

set(dart_optimizer_hdrs ${hdrs} ${dart_optimizer_hdrs} PARENT_SCOPE)
set(dart_optimizer_srcs ${srcs} ${dart_optimizer_srcs} PARENT_SCOPE)

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer headers" ${hdrs})
if(HAVE_IPOPT)
  set(header_names ${header_names} ipopt/ipopt.hpp)
endif()
if(HAVE_NLOPT)
  set(header_names ${header_names} nlopt/nlopt.hpp)
endif()
if(HAVE_SNOPT)
  set(header_names ${header_names} snopt/snopt.hpp)
endif()
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
