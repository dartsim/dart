# Variational Integrator Solver — Dev Task

Implementation tracking for PLAN-082 (linear-time variational integrator).
Plan: [`../../plans/082-variational-integrator-solver.md`](../../plans/082-variational-integrator-solver.md).
Design: [`../../design/simulation_variational_integrator.md`](../../design/simulation_variational_integrator.md).
Paper experiment replication + measured results:
[`paper-experiment-replication.md`](paper-experiment-replication.md).

## Current Status

- [x] **Phase A1 — Fixed-base MVP (dense solve, correctness-first)** — done + verified
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
  - [x] Two-step previous-configuration State — in-memory + bootstrap +
        **serialization**: `MultibodyVariationalState` is a registered State
        component (binary format bumped to v3); save/load round-trips the
        `bootstrapped` flag + `previousDeltaTransform`/`previousMomentum`.
  - [x] Integration-family **selector** (`World::setMultibodyIntegrationMethod`)
        so the VI runs on the default `step()` path by method name (pipeline
        substitution, not stage-append); covered by `SelectableThroughWorldStep`.
  - [x] Determinism & serialization — run-to-run bit-identical rollout through
        `world.step()` (`DeterministicAcrossRuns`) and a binary save/load
        round-trip that resumes the trajectory bit-identically without
        re-bootstrapping history (`StateSerializationRoundTripsTrajectory`).
  - [x] Defined non-convergence **error** — `integrateMultibodyVariational`
        throws `InvalidOperationException` (residual/iterations/tolerance) instead
        of a silent best-effort step (`NonConvergenceRaisesDocumentedError`); mean
        RIQN iterations ≤8 and every step converges
        (`RiqnMeanIterationsWithinBudget`).
  - [x] **Headline symplectic gate** + **momentum**: `PassiveChainEnergyHasNo-
SecularDrift` (10-link chain, 1e5 steps, bounded band + ~0 drift slope,
        ≥50× better than semi-implicit Euler which drifts to ~39%) and
        `FloatingBaseConservesMomentum` (linear + world angular momentum to
        solver precision).
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
- [x] **Phase B1 — Floating base** — Floating + Spherical joints with
      manifold-correct SE(3)/SO(3) RIQN retraction (`jointRetract`/
      `jointLogDifference`: `R_new = R·exp(δω)`, `p_new = p + R·δp`). Verified by
      `FloatingBaseFreeFall` (linear accel −g, no spurious rotation, to 1e-9) and
      `FloatingBaseTorqueFreeConservesEnergy` (tumbling asymmetric-inertia body
      conserves energy over 2e4 steps). Resolves the manifold-retraction gap the
      reference impl left as open TODOs.
- [x] **Phase B2 — Holonomic constraints (loop closures)** — done + verified,
      end-to-end through the public API. The constrained-VI algorithm enforces
      holonomic closures by an impulse-based Newton projection onto `g(q)=0` reusing
      the O(n) ABI (`λ = (J M⁻¹ Jᵀ)⁻¹(−g)`, `Δq = M⁻¹ Jᵀ λ`); the constraint
      Jacobian is finite-difference-verified (`ConstraintJacobianMatchesFinite-
Difference`). The World's `LoopClosure` components are wired into the
      variational stage (`bindVariationalLoopClosure`, the single source of truth for
      both the stage and the `Solve` validation), so a `Solve` Point closure runs
      through `world.step()` under the variational method
      (`LoopClosureSolvedThroughWorldStep`), while the semi-implicit path still
      rejects `Solve` and unsupported closures (Rigid/Distance families, rigid-body
      or cross-multibody endpoints) raise documented errors. **Scope today:** Point
      closures with link/world endpoints on a single multibody. Distance (needs a
      public rest-length field) and Rigid (needs an orientation residual) through the
      public model are tracked follow-ups; the internal `VariationalLoopConstraint`
      Distance path is already verified by `MaintainsDistanceLoopClosure`.
- [x] **Phase C — Contact & friction: go/no-go = NO-GO** (deferred). Recorded in
      the plan sidecar; neither entry gate met (no contact-query-at-trial-config
      redesign; no C2 spike).

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

All committed PLAN-082 phases (A1, A2, B1, B2) and their acceptance gates are
complete and verified; Phase C is a recorded NO-GO. Remaining items are
follow-ups beyond the original commitment, in priority order (see `RESUME.md`):

1. **B2 public-model follow-ups**: support Distance closures through the public
   API (needs a rest-length the `comps::LoopClosure` model does not yet carry)
   and Rigid closures (needs an orientation residual the solver does not yet
   implement). Both are rejected today with documented errors; the internal
   Distance path is already verified.
2. **A2 large-chain convergence** (research follow-up): relative/scaled tolerance,
   line-search/Anderson acceleration, or the exact recursive-Jacobian
   preconditioner (IG3 alone does not resolve the long-chain iteration cliff).
3. **Phase C**: contact/friction (deferred, go/no-go; see the plan sidecar).

Code is the source of truth; keep this file lean and current.
