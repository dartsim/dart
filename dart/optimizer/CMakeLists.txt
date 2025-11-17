# Alias headers for backward compatibility
set(hdrs
  All.hpp
  Function.hpp
  GradientDescentSolver.hpp
  Problem.hpp
  Solver.hpp
)
list(TRANSFORM hdrs PREPEND "${CMAKE_CURRENT_LIST_DIR}/")

dart_add_core_headers(${hdrs})

install(
  FILES ${hdrs}
  DESTINATION include/dart/optimizer
  COMPONENT headers
)
