# LCP Solver Test Matrix + Fixtures (Current)

This document summarizes the solver-agnostic fixtures and how they map to tests
and benchmarks. It pairs with the solver comparison contract in
`01_comparison_contract.md`.

## Solver-Agnostic Fixtures (tests/common/lcpsolver/LcpTestFixtures.hpp)

Deterministic fixtures with known solutions:

- Standard: `standard_2d_spd` (2x2 SPD).
- Boxed: `boxed_active_upper` (2x2), `boxed_mixed_bounds` (3x3).
- Friction index: `findex_single_contact_interior` (3x3),
  `findex_single_contact_at_bounds` (3x3),
  `findex_two_contacts_mixed` (6x6, two contacts).

Assertions:

- Contract invariants are checked via `tests/common/lcpsolver/LcpTestHarness.hpp`.
- Unit tests compare `x` against expected solutions with solver-appropriate
  tolerances.

## Solver-Agnostic Harness Coverage

- Implemented in `tests/unit/math/lcp/test_LcpComparisonHarness.cpp`.
- Covers pivoting, projection, Newton, and other solvers where applicable.
- Uses per-solver options tuned for algorithm behavior while keeping the same
  validation logic.

## Legacy/Helper Fixtures

- `tests/common/lcpsolver/LCPTestProblems.hpp` provides standard SPD and
  ill-formed problems (1D, 2D, 4D, 6D, 12D, 24D, 48D; ill-formed 4D/6D/12D).
- Used by legacy tests and benchmarks (e.g., `test_DantzigVsODE`,
  `BM_LCPSOLVER`).

## Benchmark Fixture Alignment

- `BM_LCP_COMPARE` uses deterministic generators in
  `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` for standard, boxed,
  friction-index, and scaled categories (see `03_benchmark_plan.md`).

## Extensions (Optional)

- Add degenerate/infeasible/scaled/large categories to
  `tests/common/lcpsolver/LcpTestFixtures.hpp` if future comparisons need them;
  keep fixture definitions and expected outcomes co-located with the harness.
