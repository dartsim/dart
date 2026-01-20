# Alias headers for backward compatibility
set(hdrs
  all.hpp
  function.hpp
  gradient_descent_solver.hpp
  problem.hpp
  solver.hpp
)
list(TRANSFORM hdrs PREPEND "${CMAKE_CURRENT_LIST_DIR}/")

dart_add_core_headers(${hdrs})

# Generate header for this namespace so legacy includes like
# <dart/optimizer/optimizer.hpp> keep working.
dart_get_filename_components(header_names "optimizer headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME optimizer
  TARGET_DIR "dart/optimizer/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

install(
  FILES ${hdrs}
        ${CMAKE_CURRENT_BINARY_DIR}/All.hpp
        ${CMAKE_CURRENT_BINARY_DIR}/optimizer.hpp
  DESTINATION include/dart/optimizer
  COMPONENT headers
)

dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/optimizer
)
