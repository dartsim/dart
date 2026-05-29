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
      confirms the inverse-mass kernel is O(n) (`806·N`, 13% RMS) and surfaced +
      fixed an O(n²) `buildVarTree` parent lookup; the full RIQN step is also O(n)
      (`3958·N`, 11% RMS) to 64 links. **Long-chain convergence (resolved):** the
      fixed `Δt·M⁻¹` quasi-Newton rate degraded for long chains (undamped 64-link
      max **456** iters; 100 links failed within 500). Depth-`m` Anderson
      acceleration (Euclidean coordinates only; spherical/floating keep the plain
      step) plus a per-coordinate `√n`-scaled tolerance bound it (64-link max
      **19**, 100-link max **205**); a 64-link chain now converges within the
      default budget (`LongChainConvergesWithinDefaultBudget`). IG3 alone did not
      help (a starting-point fix, not a rate fix); a monotone line search hurt
      (it suppressed the beneficial non-monotone steps). The exact recursive-
      Jacobian preconditioner (paper Appendix) remains an option for ≥100-link
      chains.
- [x] **Phase B1 — Floating base** — Floating + Spherical joints with
      manifold-correct SE(3)/SO(3) RIQN retraction (`jointRetract`/
      `jointLogDifference`: `R_new = R·exp(δω)`, `p_new = p + R·δp`). Verified by
      `FloatingBaseFreeFall` (linear accel −g, no spurious rotation, to 1e-9) and
      `FloatingBaseTorqueFreeConservesEnergy` (tumbling asymmetric-inertia body
      conserves energy over 2e4 steps). Resolves the manifold-retraction gap the
      reference impl left as open TODOs.
- [x] **Phase B2 — Holonomic constraints (loop closures)** — done + verified,
      end-to-end through the public API for **all three families** (Point,
      Distance, Rigid). The constrained-VI algorithm enforces closures by an
      impulse-based Newton projection onto `g(q)=0` reusing the O(n) ABI
      (`λ = (J M⁻¹ Jᵀ)⁻¹(−g)`, `Δq = M⁻¹ Jᵀ λ`). The World's `LoopClosure`
      components are wired into the variational stage (`bindVariationalLoopClosure`,
      the single source of truth for both the stage and the `Solve` validation):
      Point (3-row offset), Distance (1-row separation, with a public
      `LoopClosureSpec.distance` target), and Rigid (6-row position + world-frame
      orientation residual) closures all run through `world.step()` under the
      variational method, while the semi-implicit path still rejects `Solve` and
      cross-multibody / rigid-body endpoints raise documented errors. The
      constraint Jacobians (including the Rigid orientation Jacobian) are
      finite-difference-verified (`ConstraintJacobianMatchesFiniteDifference`,
      `RigidConstraintJacobianMatchesFiniteDifference`); closures hold through
      `world.step()` (`LoopClosure{,Distance,Rigid}SolvedThroughWorldStep`).
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
complete and verified, the paper's experiments are replicated
([`paper-experiment-replication.md`](paper-experiment-replication.md)), and the
prior follow-ups are done: the dartpy integration-family selector, the A2
long-chain convergence fix (Anderson + `√n` tolerance), and Point/Distance/Rigid
loop closures through the public API. Phase C is a recorded NO-GO. What remains:

1. **Phase C**: contact/friction (deferred, go/no-go; see the plan sidecar).
2. **A2 extreme chains (≥100 links)**: the exact recursive-Jacobian
   preconditioner (paper Appendix) for chains that still need a budget above the
   100-iteration default.
3. **Dedup**: the local spatial-algebra/kinematic-tree helpers in
   `variational_integration.cpp` duplicate a subset of `multibody_dynamics.cpp`;
   hoist into a shared internal header.

Code is the source of truth; keep this file lean and current.
