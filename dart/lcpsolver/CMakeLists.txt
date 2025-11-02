# Search all header and source files in lcpsolver directory
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs})
dart_add_core_sources(${srcs})

# Add dantzig LCP solver sources and headers
# Include both .h and .hpp files since public headers include implementation headers
file(GLOB dantzig_hdrs "dantzig/*.h" "dantzig/*.hpp")
file(GLOB dantzig_srcs "dantzig/*.cpp")
dart_add_core_headers(${dantzig_hdrs})
dart_add_core_sources(${dantzig_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "lcpsolver headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME lcpsolver
  TARGET_DIR "dart/lcpsolver/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install (includes all.hpp, lcpsolver.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/all.hpp ${CMAKE_CURRENT_BINARY_DIR}/lcpsolver.hpp ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION include/dart/lcpsolver
  COMPONENT headers
)

# Install dantzig headers
install(
  FILES ${dantzig_hdrs}
  DESTINATION include/dart/lcpsolver/dantzig
  COMPONENT headers
)
