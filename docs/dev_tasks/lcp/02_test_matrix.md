# LCP Solver Test Matrix + Fixtures Plan

This document defines the test categories, instance generation, and assertions
needed for apples-to-apples solver comparisons. It pairs with the solver
comparison contract in `01_comparison_contract.md`.

## Core Test Categories

1. Sanity (hand-constructed)
   - Sizes: 1x1, 2x2, 3x3.
   - Construction: fixed matrices/vectors with known solution.
   - Assertions: solution matches expected x within tight tolerance; invariants
     pass.

2. Random SPD (standard)
   - Sizes: {5, 10, 25, 50}.
   - Construction: `A = M^T M + n I`, `x* >= 0`, `b = A x*`.
   - Assertions: invariants pass; solution near `x*` (tight for direct solvers,
     relaxed for iterative).

3. Degenerate / Nearly singular
   - Sizes: {5, 10}.
   - Construction: SPD with tiny eigenvalues or repeated rows/cols; include
     multiple-solution cases.
   - Assertions: invariants pass within relaxed tolerance; detect soft-pass if
     only relaxed threshold satisfied.

4. Infeasible / Inconsistent
   - Sizes: {2, 5}.
   - Construction: contradictory bounds or inconsistent complementarity (e.g.,
     `lo > hi` or `b` that forces `w` negative at fixed bounds).
   - Expected behavior: solver reports `InvalidProblem` or `Failed`, or
     `MaxIterations`. If solver returns `Success`, invariants must still pass.
   - Assertions: no crash; outputs finite.

5. Box constraints stress
   - Sizes: {5, 10, 25}.
   - Construction: mixed bounds (finite, +/-inf, fixed variables); active lower
     and upper bounds in same instance.
   - Assertions: invariants pass; active-bound cases hit expected bounds.

6. Friction-index cases
   - Sizes: contact blocks with 1, 4, 8, 16 groups (3 vars per group).
   - Construction: grouped `findex` patterns:
     - no friction indices (all -1)
     - single group (all tangents reference one normal)
     - many small groups (each tangential pair references its own normal)
     - pathological grouping (sparse references, non-contiguous blocks)
   - Assertions: effective bounds via `findex` are satisfied; invariants pass.

7. Scaling / Conditioning
   - Sizes: {5, 10}.
   - Construction: scale `A` and `b` by large/small factors (1e-6, 1e6),
     include mixed-scale diagonal dominance.
   - Assertions: invariants pass under scaled tolerances.

8. Large (dense baseline, sparse-ish if supported)
   - Sizes: {100, 250} (dense), optional sparse-ish blocks.
   - Construction: block-diagonal SPD with known `x*`.
   - Assertions: invariants pass; performance sanity for benchmarks.

## Fixture/Generator Plan

Deterministic fixtures (checked into tests):

- Standard/boxed LCP: at least 3 fixed problems with known solutions.
- Boxed LCP + friction index: at least 3 fixed problems with known solutions
  that exercise `findex` coupling and mixed bounds.

Generated fixtures (seeded):

- Random SPD and friction-index problems using fixed seeds per size.
- Degenerate/scaled variants created from base problems.
- Harness + fixtures live under `tests/common/lcpsolver/` and are shared by
  unit tests and benchmarks to keep comparisons aligned.

## Assertions Summary

For each instance:

- Validate solver result with the shared harness:
  - bounds feasibility, complementarity, residual norms, finiteness.
- For sanity problems: compare `x` against expected solution.
- For randomized problems: compare against reference solver or known `x*`.
- For infeasible problems: expect non-success or explicit invalid-problem
  status; if success is reported, invariants must still hold.
