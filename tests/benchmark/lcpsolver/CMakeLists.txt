# Copyright (c) 2011-2025, The DART development contributors
#
# Benchmark tests for LCP solvers

# Combined benchmark: Dantzig vs ODE baseline comparison
add_executable(BM_LCPSOLVER bm_lcpsolver.cpp)
target_link_libraries(BM_LCPSOLVER dart dart-baseline-ode benchmark::benchmark benchmark::benchmark_main)
set_target_properties(BM_LCPSOLVER PROPERTIES FOLDER "Benchmarks")

# Benchmark: math::LcpSolver implementations (Dantzig, PGS)
add_executable(BM_LCPSOLVER_SOLVERS bm_lcpsolver_solvers.cpp)
target_link_libraries(BM_LCPSOLVER_SOLVERS dart benchmark::benchmark benchmark::benchmark_main)
set_target_properties(BM_LCPSOLVER_SOLVERS PROPERTIES FOLDER "Benchmarks")

# Row swapping benchmark: Pointer vs Eigen comparison
add_executable(BM_ROW_SWAPPING bm_row_swapping.cpp)
target_link_libraries(BM_ROW_SWAPPING dart benchmark::benchmark)
set_target_properties(BM_ROW_SWAPPING PROPERTIES FOLDER "Benchmarks")

# Dot product benchmark: Manual unrolling vs Eigen SIMD
add_executable(BM_DOT_PRODUCT bm_dot_product.cpp)
target_link_libraries(BM_DOT_PRODUCT benchmark::benchmark Eigen3::Eigen)
set_target_properties(BM_DOT_PRODUCT PROPERTIES FOLDER "Benchmarks")

# Matrix multiply benchmark: Raw pointer vs Eigen SIMD comparison
add_executable(BM_MATRIX_MULTIPLY bm_matrix_multiply.cpp)
target_link_libraries(BM_MATRIX_MULTIPLY dart benchmark::benchmark)
set_target_properties(BM_MATRIX_MULTIPLY PROPERTIES FOLDER "Benchmarks")

# Add to formatting
dart_format_add(
  bm_lcpsolver.cpp
  bm_lcpsolver_solvers.cpp
  bm_row_swapping.cpp
  bm_dot_product.cpp
  bm_matrix_multiply.cpp
)
