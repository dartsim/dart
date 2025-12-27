# LCP Solver Add Checklist

Use this checklist when adding a new solver implementation.

## Implementation

- Implement `dart::math::LcpSolver::solve` and populate `LcpResult` fields
  (status, iterations, residual, complementarity, message).
- Wire solver-specific parameters through `LcpOptions::customOptions`.
- Ensure `validateProblem()` and `computeEffectiveBounds()` usage matches
  existing solvers (bounds + friction index rules).
- Add the header to `dart/math/lcp/All.hpp` if it is public API.

## Test Integration

- Add the solver to the solver-agnostic harness in
  `tests/unit/math/lcp/test_LcpComparisonHarness.cpp`.
- Use fixtures from `tests/common/lcpsolver/LcpTestFixtures.hpp`.
- Keep `LcpOptions` aligned with `01_comparison_contract.md`.
- Run unit coverage:
  - Suggested (Unverified): `DART_PARALLEL_JOBS=<N> CTEST_PARALLEL_LEVEL=<N> pixi run test`
  - Suggested (Unverified): `ctest --test-dir build/<env>/cpp/Release -R UNIT_math_lcp`

## Benchmark Integration

- Add the solver to `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` (standard,
  boxed, friction-index categories as applicable).
- Use the shared `MakeBenchmarkOptions()` profile and the same fixtures.
- Run the smoke benchmark:
  - Suggested (Unverified): `pixi run bm --target BM_LCP_COMPARE -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`

## Reporting

- Record results in `docs/dev_tasks/lcp/11_result_reporting_template.md`.
- Highlight regressions vs existing solvers (time, iterations, residual).
