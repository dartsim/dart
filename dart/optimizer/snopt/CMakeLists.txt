# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
dart_add_library(dart-optimizer-snopt ${srcs} ${hdrs})
target_link_libraries(dart-optimizer-snopt dart)

# Generate header for this namespace
dart_get_filename_components(header_names "optimizer_snopt headers" ${hdrs})
dart_generate_include_header_list(
  optimizer_snopt_headers
  "dart/optimizer/snopt/"
  "optimizer_snopt headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/snopt.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/snopt.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/snopt.h
  DESTINATION include/dart/optimizer/snopt
  COMPONENT headers
)
install(TARGETS dart-optimizer-snopt
  EXPORT DARTTargets
  DESTINATION lib
)
