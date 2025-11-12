# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_core_headers(${hdrs})
dart_add_core_sources(${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME optimizer
  TARGET_DIR "dart/optimizer/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install (includes All.hpp, optimizer.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/optimizer.hpp ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION include/dart/optimizer
  COMPONENT headers
)

# Add subdirectories (components)
if(NOT (DART_BUILD_OPTIMIZER_IPOPT STREQUAL "OFF"))
  add_subdirectory(ipopt)
endif()

if(NOT (DART_BUILD_OPTIMIZER_NLOPT STREQUAL "OFF"))
  add_subdirectory(nlopt)
endif()

# Skip pagmo when building wheels to avoid dependency issues
if(NOT DART_BUILD_WHEELS AND NOT (DART_BUILD_OPTIMIZER_PAGMO STREQUAL "OFF"))
  add_subdirectory(pagmo)
elseif(DART_BUILD_WHEELS)
  # Wheels build: component cannot be built, keep flag disabled
  set(DART_HAVE_PAGMO FALSE CACHE BOOL "Whether PaGMO support is enabled" FORCE)
endif()

add_subdirectory(snopt)
