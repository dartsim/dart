# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(kido_simulation_hdrs ${hdrs} PARENT_SCOPE)
set(kido_simulation_srcs ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_simulation ${srcs} ${hdrs})
#target_link_libraries(
#  kido_simulation
#  kido_constraint
#  kido_integration
#  ${KIDO_CORE_DEPENDENCIES}
#)

# Generate header for this namespace
kido_get_filename_components(header_names "simulation headers" ${hdrs})
kido_generate_include_header_list(
  simulation_headers
  "kido/simulation/"
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
  DESTINATION include/kido/simulation
  COMPONENT headers
)
#install(TARGETS kido_simulation EXPORT KIDOCoreTargets DESTINATION lib)
#install(TARGETS kido_simulation EXPORT KIDOTargets DESTINATION lib)

