# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
file(GLOB solver_hdrs "solver/*.hpp")
file(GLOB solver_srcs "solver/*.cpp")
dart_add_core_headers(${hdrs} ${detail_hdrs} ${solver_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs} ${solver_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "simulation headers" ${hdrs} ${solver_hdrs})
list(REMOVE_ITEM header_names SmartPointer.hpp)
dart_generate_component_headers(
  COMPONENT_NAME simulation
  TARGET_DIR "dart/simulation/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs} ${detail_hdrs} ${solver_hdrs}
)

# Install (includes All.hpp, simulation.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/simulation.hpp
  DESTINATION include/dart/simulation
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/simulation/detail
  COMPONENT headers
)
install(
  FILES ${solver_hdrs}
  DESTINATION include/dart/simulation/solver
  COMPONENT headers
)
# Install generated compatibility headers preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/simulation
)
