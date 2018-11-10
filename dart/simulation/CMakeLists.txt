# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

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
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/simulation/detail
  COMPONENT headers
)

dart_format_add(
  detail/World-impl.hpp
)
