# LCP Solver Status (Current Snapshot)

This note captures the current LCP solver APIs, tests, and benchmarks so the
comparison plan can build on verified repo state.

## Solver Interfaces

- `dart/math/lcp/LcpTypes.hpp` defines `LcpProblem`, `LcpOptions`, `LcpResult`,
  and solver status codes.
- `dart/math/lcp/LcpSolver.hpp` defines the solver interface; implementations
  live under `dart/math/lcp/pivoting`, `dart/math/lcp/projection`,
  `dart/math/lcp/newton`, and `dart/math/lcp/other`.
- `dart/math/lcp/LcpValidation.hpp` provides shared problem validation and
  residual/complementarity metrics.
- `dart/constraint/ConstraintSolver` builds `LcpProblem` and uses
  `math::DantzigSolver` with `math::PgsSolver` fallback. Deprecated boxed solver
  adapters remain in `dart/constraint/*BoxedLcp*`.

## Boxed LCP + Friction Index Representation

- `LcpProblem` carries `A, b, lo, hi, findex` with DART's `w = Ax - b`
  convention.
- Standard LCP uses `lo = 0`, `hi = +inf`, `findex = -1`.
- Friction coupling uses `findex[i] = j` to bind tangential variable `i` to
  normal variable `j`:
  - `hi[i]` stores the finite coefficient `mu`.
  - Effective bounds are `|x[i]| <= |mu| * |x[j]|` via
    `computeEffectiveBounds()` in `LcpValidation.hpp`.
  - `validateProblem()` rejects out-of-range/self-referential findex and
    non-finite friction coefficients.

## Existing Tests

- Solver-agnostic harness coverage in
  `tests/unit/math/lcp/test_LcpComparisonHarness.cpp` using shared fixtures and
  the contract checks in `tests/common/lcpsolver/LcpTestHarness.hpp`.
- Deterministic fixtures in `tests/common/lcpsolver/LcpTestFixtures.hpp` cover
  standard, boxed, and friction-index cases with known solutions.
- Legacy fixtures in `tests/common/lcpsolver/LCPTestProblems.hpp` (standard LCP
  only) are still used by Dantzig-vs-ODE checks and the older benchmarks.

## Existing Benchmarks

- Google Benchmark harnesses in `tests/benchmark/lcpsolver`:
  - `BM_LCPSOLVER`: low-level Dantzig (float/double) vs ODE baseline on the
    standard-problem fixtures.
  - `BM_LCPSOLVER_SOLVERS`: Dantzig/PGS/Lemke on generated standard, boxed, and
    friction-index problems.
  - `BM_LCP_COMPARE`: solver-agnostic comparisons across standard, boxed,
    friction-index, and scaled categories with contract counters.

## Notes

- PenalizedFischerBurmeisterNewton occasionally fails the contract on
  `BM_LCP_COMPARE` standard SPD size 24 (contract_ok=0, residual and
  complementarity around 1.6e-2 with default benchmark options).
