# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

include_directories(SYSTEM ${FLANN_INCLUDE_DIRS})

# Library
kido_add_library(kido-planning ${srcs} ${hdrs})
target_link_libraries(kido-planning kido ${FLANN_LIBRARIES})

# Generate header for this namespace
kido_get_filename_components(header_names "planning headers" ${hdrs})
kido_generate_include_header_list(
  planning_headers
  "kido/planning/"
  "planning headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/planning.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/planning.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/planning.hpp
  DESTINATION include/kido/planning
  COMPONENT headers
)
install(TARGETS kido-planning EXPORT KIDOTargets DESTINATION lib)
