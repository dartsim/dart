# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

include_directories(SYSTEM ${NLOPT_INCLUDE_DIRS})

# Library
dart_add_library(dart-optimizer-ipopt ${srcs} ${hdrs})
target_link_libraries(dart-optimizer-ipopt dart ${IPOPT_LIBRARIES})

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
install(TARGETS dart-optimizer-ipopt EXPORT DART-OPTIMIZER-IPOPTTargets DESTINATION lib)
install(EXPORT DART-OPTIMIZER-IPOPTTargets DESTINATION share/dart)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
