# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(dart_simulation_hdrs ${hdrs} PARENT_SCOPE)
set(dart_simulation_srcs ${srcs} PARENT_SCOPE)

# Generate header for this namespace
dart_get_filename_components(header_names "simulation headers" ${hdrs})
dart_generate_include_header_list(
  simulation_headers
  "dart/simulation/"
  "simulation headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/simulation.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/simulation.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/simulation.hpp
  DESTINATION include/dart/simulation
  COMPONENT headers
)
