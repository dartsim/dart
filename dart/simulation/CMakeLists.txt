# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_simulation_hdrs ${hdrs} PARENT_SCOPE)
set(dart_simulation_srcs ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_simulation ${srcs} ${hdrs})
#target_link_libraries(
#  dart_simulation
#  dart_constraint
#  dart_integration
#  ${KIDO_CORE_DEPENDENCIES}
#)

# Generate header for this namespace
dart_get_filename_components(header_names "simulation headers" ${hdrs})
dart_generate_include_header_list(
  simulation_headers
  "dart/simulation/"
  "simulation headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/simulation.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/simulation.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/simulation.h
  DESTINATION include/dart/simulation
  COMPONENT headers
)
#install(TARGETS dart_simulation EXPORT DARTCoreTargets DESTINATION lib)
#install(TARGETS dart_simulation EXPORT DARTTargets DESTINATION lib)

