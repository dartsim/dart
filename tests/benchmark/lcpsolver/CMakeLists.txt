# Copyright (c) 2011, The DART development contributors
#
# Benchmark tests for LCP solvers

# Combined benchmark: Dantzig vs ODE baseline comparison
add_executable(BM_LCPSOLVER bm_lcpsolver.cpp)
target_link_libraries(
  BM_LCPSOLVER
  dart
  dart-baseline-ode
  benchmark::benchmark
  benchmark::benchmark_main
)
set_target_properties(BM_LCPSOLVER PROPERTIES FOLDER "Benchmarks")

# Benchmark: math::LcpSolver implementations (Dantzig, PGS)
add_executable(BM_LCPSOLVER_SOLVERS bm_lcpsolver_solvers.cpp)
target_link_libraries(
  BM_LCPSOLVER_SOLVERS
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
set_target_properties(BM_LCPSOLVER_SOLVERS PROPERTIES FOLDER "Benchmarks")

# Benchmark: solver-agnostic comparison harness
add_executable(BM_LCP_COMPARE bm_lcp_compare.cpp)
target_link_libraries(
  BM_LCP_COMPARE
  dart
  benchmark::benchmark
  benchmark::benchmark_main
)
if(DART_ENABLE_SIMD)
  target_compile_definitions(
    BM_LCP_COMPARE
    PRIVATE DART_BM_LCP_COMPARE_DART_ENABLE_SIMD=1
  )
else()
  target_compile_definitions(
    BM_LCP_COMPARE
    PRIVATE DART_BM_LCP_COMPARE_DART_ENABLE_SIMD=0
  )
endif()
if(DART_SIMD_FORCE_SCALAR)
  target_compile_definitions(
    BM_LCP_COMPARE
    PRIVATE DART_BM_LCP_COMPARE_DART_SIMD_FORCE_SCALAR=1
  )
else()
  target_compile_definitions(
    BM_LCP_COMPARE
    PRIVATE DART_BM_LCP_COMPARE_DART_SIMD_FORCE_SCALAR=0
  )
endif()
if(DART_ENABLE_EXPERIMENTAL_CUDA)
  target_compile_definitions(
    BM_LCP_COMPARE
    PRIVATE DART_BM_LCP_COMPARE_DART_ENABLE_EXPERIMENTAL_CUDA=1
  )
else()
  target_compile_definitions(
    BM_LCP_COMPARE
    PRIVATE DART_BM_LCP_COMPARE_DART_ENABLE_EXPERIMENTAL_CUDA=0
  )
endif()
if(TARGET dart-simulation)
  target_link_libraries(BM_LCP_COMPARE dart-simulation)
  if(TARGET EnTT::EnTT)
    target_link_libraries(BM_LCP_COMPARE EnTT::EnTT)
  endif()
  target_compile_definitions(
    BM_LCP_COMPARE
    PRIVATE DART_BM_LCP_COMPARE_HAS_SIMULATION=1
  )
endif()
if(TARGET dart-simulation-cuda)
  target_link_libraries(BM_LCP_COMPARE dart-simulation-cuda)
  target_compile_definitions(
    BM_LCP_COMPARE
    PRIVATE DART_BM_LCP_COMPARE_HAS_SIMULATION_CUDA=1
  )
endif()
set_target_properties(BM_LCP_COMPARE PROPERTIES FOLDER "Benchmarks")

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

foreach(
  benchmark_target
  IN
  ITEMS
    BM_LCPSOLVER
    BM_LCPSOLVER_SOLVERS
    BM_LCP_COMPARE
    BM_ROW_SWAPPING
    BM_DOT_PRODUCT
    BM_MATRIX_MULTIPLY
)
  target_compile_features(${benchmark_target} PRIVATE cxx_std_23)
endforeach()

# Add to formatting
dart_format_add(
  bm_lcpsolver.cpp
  bm_lcpsolver_solvers.cpp
  bm_lcp_compare.cpp
  bm_row_swapping.cpp
  bm_dot_product.cpp
  bm_matrix_multiply.cpp
)
