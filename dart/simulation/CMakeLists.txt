# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "simulation headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME simulation
  TARGET_DIR "dart/simulation/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/all.hpp ${CMAKE_CURRENT_BINARY_DIR}/simulation.hpp
  DESTINATION include/dart/simulation
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/simulation/detail
  COMPONENT headers
)
