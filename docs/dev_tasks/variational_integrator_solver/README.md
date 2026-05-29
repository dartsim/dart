# Variational Integrator Solver — Dev Task

Implementation tracking for PLAN-082 (linear-time variational integrator).
Plan: [`../../plans/082-variational-integrator-solver.md`](../../plans/082-variational-integrator-solver.md).
Design: [`../../design/simulation_variational_integrator.md`](../../design/simulation_variational_integrator.md).

## Current Status

- [~] **Phase A1 — Fixed-base MVP (dense solve, correctness-first)** ← core done + verified
  - [x] SE(3) discrete-mechanics kernels (`detail/variational/discrete_mechanics_math.hpp`):
        `dexp⁻¹`/`dexpInvTranspose` **delegate to the exact
        `SE3::LeftJacobianInverse`**, plus `dAdT`/`dAdInvT`/`adInvRLinear`/`se3Exp`/
        `se3Log`; cross-checked against `dart::math::lie_group` by a parity test.
  - [x] DRNEA residual recursion (forward average-velocity sweep + backward
        discrete-momentum/impulse sweep + per-joint `SᵀF − Q`).
  - [x] RIQN outer loop with the **O(n) articulated-body-inertia inverse-mass
        solve** (`computeMultibodyInverseMassProduct`; no dense factorization),
        `max_iter`, convergence report.
  - [x] Articulated total-mechanical-energy helper (`computeMultibodyMechanicalEnergy`).
  - [x] `MultibodyVariationalIntegrationStage` + registered in CMake; lib + full
        experimental suite build green (24/24).
  - [x] Tests: prismatic free-fall (accel = −9.81 to 1e-9), pendulum first-step
        accel, RIQN convergence, energy conserved over 1e5 steps, lie_group parity.
  - [~] Two-step previous-configuration State — in-memory + bootstrap done;
    **serialization (version bump + bootstrap-done flag) pending**.
  - [x] Integration-family **selector** (`World::setMultibodyIntegrationMethod`)
        so the VI runs on the default `step()` path by method name (pipeline
        substitution, not stage-append); covered by `SelectableThroughWorldStep`.
  - [~] Determinism — run-to-run bit-identical rollout through `world.step()`
    verified (`DeterministicAcrossRuns`); the **save/load** round-trip still
    needs the VI-state serialization above.
  - [ ] Defined non-convergence **error** (currently reports `converged=false`).
- [x] **Phase A2 — O(n) impulse-based ABI** — RIQN uses the articulated-body
      inverse-mass solve (matches the dense `M⁻¹` to 1e-9,
      `ArticulatedInverseMassMatchesDenseSolve`). `bm_variational_integration`
      confirms the inverse-mass kernel is O(n) (BigO `763·N`, 3% RMS) and surfaced +
      fixed an O(n²) `buildVarTree` parent lookup. **Finding**: per-iteration work is
      O(n) and the integrator scales linearly + converges in a few iterations for
      realistic DOF counts (≤~32), but the RIQN iteration count rises sharply for
      long chains (≥~64 links) at small `dt` — the known `Δt·M⁻¹` quasi-Newton
      limitation. The IG3 (semi-implicit, forward-dynamics) initial guess is used
      and helps continuous rollouts, but does **not** resolve the cliff: it is a
      convergence-_rate_ issue (the fixed `Δt·M⁻¹` preconditioner), not a
      starting-point one. Deeper mitigations (relative/scaled tolerance,
      line-search/Anderson acceleration, or the exact recursive-Jacobian
      preconditioner from the paper's Appendix) are tracked follow-ups.
- [ ] Phase B1 — Floating base (manifold-correct RIQN retraction)
- [ ] Phase B2 — Holonomic constraints (loop closures)
- [ ] Phase C — Contact & friction (DEFERRED, separate go/no-go; see plan sidecar)

## Goal

A correct, tested variational-integrator stage in the experimental `World`,
selectable by the `variational integrator` method name, proving symplectic
energy behavior on a passive chain before optimizing to O(n).

## Non-Goals (early phases)

- O(n) scaling (Phase A1 uses a dense solve; A2 adds ABI).
- Floating base, loop closures, contact/friction (later phases).
- Variable time step; GPU/batched execution.

## Key Decisions (rationale in the design doc)

- **Dense-solve placeholder first** (A1), ABI second (A2): get correctness +
  symplectic-behavior evidence before the hardest kernel. The reviewers
  confirmed the experimental World has **no ABA** today (dense `M.ldlt()`).
- **Gravity is forcing-side** (`Fᵏ`), not a Lagrangian potential — matches the
  reference impl; avoids double-counting.
- **Port the author's reference kernels** (`dexp_inv`, `dexp_inv_transpose`,
  `logMap`, `dAdT`/`dAdInvT`) from `/tmp/dart-vi-research/wafr2016`
  (`dart/math/Geometry.cpp`); reuse `dart::math::lie_group` SE3/SO3 where types
  align.
- **Reuse** `DynamicsTree`/spatial algebra from `multibody_dynamics.cpp`; DRNEA
  is a new recursion (continuous RNE is not reusable for the discrete residual).
- **Manifold correctness** (ball/free) deferred to Phase B1; Phase A1 is
  fixed-base revolute/prismatic (Euclidean), matching the reference impl's
  validated envelope.

## Immediate Next Steps

Phase A1 + A2 (core) are complete and verified. Remaining, in priority order
(see `RESUME.md` for the full handoff):

1. A1 finish: serialize `MultibodyVariationalState` (binary-format version bump +
   bootstrap-done flag) + a save/load determinism round-trip test; optionally a
   documented non-convergence error.
2. A2 large-chain convergence (follow-up): relative/scaled tolerance,
   line-search/Anderson acceleration, or the exact recursive-Jacobian
   preconditioner (IG3 alone does not resolve the long-chain iteration cliff).
3. Phase B1: floating base (manifold-correct RIQN retraction).
4. Phase B2: holonomic constraints (loop closures).
5. Phase C: contact/friction (deferred, go/no-go; see the plan sidecar).

Code is the source of truth; keep this file lean and current.
