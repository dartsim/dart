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
  - [ ] Determinism / save-load round-trip test (needs the serialization above).
  - [ ] Defined non-convergence **error** (currently reports `converged=false`).
- [~] **Phase A2 — O(n) impulse-based ABI** — RIQN now uses the articulated-body
  inverse-mass solve (matches the dense `M⁻¹` to 1e-9 in
  `ArticulatedInverseMassMatchesDenseSolve`); the per-step-cost-vs-DOF scaling
  benchmark is the remaining A2 evidence item.
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

1. Add the SE(3) Lie-group calculus kernels (ported + unit-tested vs finite
   differences) in the experimental `common`/`detail` math area.
2. Implement DRNEA residual over `DynamicsTree`; unit-test the residual is ~0 at
   the semi-implicit-consistent configuration for a 1-DOF pendulum.
3. Implement the RIQN loop (dense placeholder) + the stage + selector; wire and
   build.
4. Add the energy helper + the analytic/energy/convergence/determinism tests.

Code is the source of truth; keep this file lean and current.
