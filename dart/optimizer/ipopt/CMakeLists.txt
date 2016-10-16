# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-optimizer-ipopt)
set(component_name optimizer-ipopt)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC ${IPOPT_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  dart
  ${IPOPT_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)

# Coverage test
dart_coveralls_add(${hdrs} ${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer_ipopt headers" ${hdrs})
dart_generate_include_header_list(
  optimizer_ipopt_headers
  "dart/optimizer/ipopt/"
  "optimizer_ipopt headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ipopt.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/ipopt.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/ipopt.hpp
  DESTINATION include/dart/optimizer/ipopt
  COMPONENT headers
)
