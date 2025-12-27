# LCP Solver Status (Current Snapshot)

This note captures the current LCP solver APIs, tests, and benchmarks so the
comparison plan can build on verified repo state.

## Solver Interfaces

- `dart/math/lcp/LcpTypes.hpp` defines `LcpProblem`, `LcpOptions`, `LcpResult`,
  and solver status codes.
- `dart/math/lcp/LcpSolver.hpp` defines the solver interface; implementations
  live under `dart/math/lcp/pivoting` (Dantzig, Lemke) and
  `dart/math/lcp/projection` (PGS).
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

- Unit coverage in `tests/unit/math/lcp` for Dantzig, PGS, Lemke, including:
  - Small deterministic standard/boxed problems (incl. friction index).
  - Edge cases (empty problems, invalid bounds/findex, warm-start sizing).
  - Stress tests on random SPD problems and friction-index cases with relaxed
    tolerances.
- Shared standard-problem fixtures in
  `tests/common/lcpsolver/LCPTestProblems.hpp` (standard LCP only).

## Existing Benchmarks

- Google Benchmark harness in `tests/benchmark/lcpsolver`:
  - `BM_LCPSOLVER`: low-level Dantzig (float/double) vs ODE baseline on the
    standard-problem fixtures.
  - `BM_LCPSOLVER_SOLVERS`: Dantzig/PGS/Lemke on generated standard, boxed, and
    friction-index problems.

## Gaps For Apples-to-Apples Comparison

- No solver-agnostic test harness enforcing uniform residual/complementarity
  checks and reporting.
- Boxed and friction-index fixtures are sparse and not organized as a shared
  contract-style matrix (degenerate, scaling, infeasible, etc.).
- Benchmarks do not use the same problem instances or normalized stopping
  criteria across solvers, and outputs are only the default Google Benchmark
  format (no JSON/CSV contract).
