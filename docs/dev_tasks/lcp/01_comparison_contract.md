# LCP Solver Comparison Contract

This contract defines the inputs, outputs, invariants, and stopping criteria
used for apples-to-apples solver comparisons. It is solver-agnostic and maps to
the existing `dart::math::LcpProblem` / `LcpOptions` types.

## Required Input Types

All comparisons use `LcpProblem` with DART's `w = Ax - b` convention.

1) Standard LCP
   - `lo = 0`, `hi = +inf`, `findex = -1`.
2) Boxed LCP
   - Finite or infinite `lo/hi`, `findex = -1`.
3) Boxed LCP + friction index
   - `findex[i] = j` couples variable `i` to normal variable `j`.
   - `hi[i]` stores the finite friction coefficient `mu`.
   - Effective bounds are `|x[i]| <= |mu| * |x[j]|` as computed by
     `computeEffectiveBounds()` in `LcpValidation.hpp`.

## Required Outputs

Solvers must provide:
- `x` (solution vector).
- `LcpResult` fields: status, iterations, residual, complementarity.

The harness computes:
- `w = A * x - b`.
- Effective bounds (`loEff`, `hiEff`) via `computeEffectiveBounds()`.
- Natural residual infinity norm.
- Complementarity infinity norm.
- Bound violation (max distance outside `[loEff, hiEff]`).
- Wall time (benchmarks only).

## Invariants (Checked By Harness)

All invariants are evaluated against `loEff/hiEff` for friction-index problems.

- Bounds feasibility: `loEff - tol <= x <= hiEff + tol`.
- Complementarity:
  - At lower bound: `w >= -tol`.
  - At upper bound: `w <= +tol`.
  - Interior: `|w| <= tol`.
  - Fixed variable (lo==hi): `|w| <= tol`.
- Natural residual infinity norm <= tol.
- `x` and `w` must be finite (no NaN/Inf).

## Stopping Criteria (Normalized)

Comparisons use a shared `LcpOptions` profile; solvers that do not honor a field
may ignore it, but the harness still enforces the external invariants.

Baseline comparison profile (tests):
- `maxIterations`: set per test category (see test matrix).
- `absoluteTolerance = 1e-6`
- `relativeTolerance = 1e-4`
- `complementarityTolerance = 1e-6`
- `relaxation = 1.0`
- `warmStart = false`
- `earlyTermination = false`
- `validateSolution = false` (harness does validation consistently)

Benchmark profile:
- Same tolerances as above, `validateSolution = false`.
- Optional fixed wall-time budget for runaway cases (harness-level timeout).

Iteration normalization:
- Iterative solvers must respect `maxIterations`.
- Direct/pivoting solvers report `iterations = 1` (or 0 for empty problems).
- If a solver reaches `maxIterations` but still satisfies invariants, the
  harness reports `MaxIterations` + `pass_within_tolerance`.

## Tolerance Policy and Scaling

Define a scaled tolerance for validation:

```
scale = max(1, ||b||_inf, ||A||_inf * ||x||_inf)
tol = max(absoluteTolerance, relativeTolerance * scale)
compTol = max(complementarityTolerance, relativeTolerance * scale)
```

Use `tol` for bounds feasibility and natural residual; use `compTol` for
complementarity checks.

Ill-conditioned or degenerate cases:
- Keep the same tolerance formula but record a "relaxed" secondary threshold
  (e.g., 10x `tol`) for reporting. If only the relaxed threshold is met, mark
  as "soft pass" and flag the case for review.

## Determinism Expectations

- With identical inputs, options, and seeds, solvers must produce results that
  match within the contract tolerances.
- Randomized solvers must expose a fixed seed or be run with randomized ordering
  disabled for comparison tests.
