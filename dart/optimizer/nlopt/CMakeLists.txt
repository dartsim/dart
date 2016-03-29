# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

include_directories(SYSTEM ${IPOPT_INCLUDE_DIRS})

# Library
dart_add_library(dart-optimizer-nlopt ${srcs} ${hdrs})
target_link_libraries(dart-optimizer-nlopt dart ${NLOPT_LIBRARIES})

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer_nlopt headers" ${hdrs})
dart_generate_include_header_list(
  optimizer_nlopt_headers
  "dart/optimizer/nlopt/"
  "optimizer_nlopt headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/nlopt.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/nlopt.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/nlopt.hpp
  DESTINATION include/dart/optimizer/nlopt
  COMPONENT headers
)
install(TARGETS dart-optimizer-nlopt EXPORT DARTTargets DESTINATION lib)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
