# Projection / Sweeping Methods

This folder is reserved for iterative projection-based LCP solvers (e.g., PGS, PSOR, BGS, NNCG).

Implemented solvers:

- `PgsSolver`: Projected Gauss-Seidel for standard and boxed LCPs (supports
  `lo/hi/findex` for friction-limited contact).

When adding a solver:

1. Implement a class deriving from `LcpSolver`.
2. Expose the solver header here and include it from `lcp/All.hpp` if desired.
3. Add focused unit tests under `tests/unit/math/lcp`.
