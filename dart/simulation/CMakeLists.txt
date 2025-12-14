# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
file(GLOB comps_hdrs "comps/*.hpp")
file(GLOB comps_srcs "comps/*.cpp")
file(GLOB object_hdrs "object/*.hpp")
file(GLOB object_srcs "object/*.cpp")
file(GLOB solver_hdrs "solver/*.hpp" "solver/*/*.hpp")
file(GLOB solver_srcs "solver/*.cpp" "solver/*/*.cpp")
dart_add_core_headers(
  ${hdrs}
  ${detail_hdrs}
  ${comps_hdrs}
  ${object_hdrs}
  ${solver_hdrs}
)
dart_add_core_sources(
  ${srcs}
  ${detail_srcs}
  ${comps_srcs}
  ${object_srcs}
  ${solver_srcs}
)

# Generate header for this namespace
set(header_names "")
foreach(header ${hdrs} ${comps_hdrs} ${object_hdrs} ${solver_hdrs})
  file(RELATIVE_PATH header_relpath "${CMAKE_CURRENT_SOURCE_DIR}" "${header}")
  list(APPEND header_names "${header_relpath}")
endforeach()
list(REMOVE_ITEM header_names SmartPointer.hpp)
list(FILTER header_names EXCLUDE REGEX ".*-impl\\.hpp")
list(REMOVE_DUPLICATES header_names)
dart_generate_component_headers(
  COMPONENT_NAME simulation
  TARGET_DIR "dart/simulation/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS
    ${hdrs}
    ${detail_hdrs}
    ${comps_hdrs}
    ${object_hdrs}
    ${solver_hdrs}
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
  FILES ${comps_hdrs}
  DESTINATION include/dart/simulation/comps
  COMPONENT headers
)
install(
  FILES ${object_hdrs}
  DESTINATION include/dart/simulation/object
  COMPONENT headers
)
install(
  DIRECTORY solver/
  DESTINATION include/dart/simulation/solver
  COMPONENT headers
  FILES_MATCHING
  PATTERN "*.hpp"
)
# Install generated compatibility headers preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/simulation
)
