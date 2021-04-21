return()
# TODO(JS): SNOPT optimization is currently outdated.

# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

# Library
dart_add_library(dart-optimization-snopt ${srcs} ${hdrs})
target_link_libraries(dart-optimization-snopt dart)

# Generate header for this namespace
dart_get_filename_components(header_names "optimization_snopt headers" ${hdrs})
dart_generate_meta_header(
  "${CMAKE_CURRENT_BINARY_DIR}/snopt.hpp"
  "dart/optimization/snopt/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/snopt.h
  DESTINATION include/dart/optimization/snopt
  COMPONENT headers
)
install(TARGETS dart-optimization-snopt
  EXPORT DARTTargets
  DESTINATION lib
)

dart_format_add(${hdrs} ${srcs})
