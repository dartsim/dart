# Copyright (c) 2011-2025, The DART development contributors
#
# Benchmark tests for LCP solvers

# Combined benchmark: Dantzig vs ODE baseline comparison
add_executable(BM_LCPSOLVER bm_lcpsolver.cpp)
target_link_libraries(BM_LCPSOLVER dart dart-baseline-ode benchmark::benchmark benchmark::benchmark_main)
set_target_properties(BM_LCPSOLVER PROPERTIES FOLDER "Benchmarks")

# Add to formatting
dart_format_add(
  bm_lcpsolver.cpp
)
