# Variational Integrator Solver — Dev Task

Implementation tracking for PLAN-082 (linear-time variational integrator).
Plan: [`../../plans/082-variational-integrator-solver.md`](../../plans/082-variational-integrator-solver.md).
Design: [`../../design/simulation_variational_integrator.md`](../../design/simulation_variational_integrator.md).
Paper experiment replication + measured results:
[`paper-experiment-replication.md`](paper-experiment-replication.md).

## North Star

The durable target this work builds toward — a **production-grade, linear-time
variational integrator**: DART's symplectic, structure-preserving alternative to
semi-implicit Euler for multibody systems, faithful to Lee/Liu/Park/Srinivasa
(WAFR 2016, arXiv:1609.02898) and extended to DART's needs. It lives entirely
behind the experimental World's method-name facade
(`World::setMultibodyOptions({.integrationFamily = "variational integrator"})`),
never exposing solver / stage / component / backend types, and adds **zero
runtime overhead** when not selected (the default path stays semi-implicit
Euler).

A "done" north star means all of the following. Checked items are implemented
and verified **today**; unchecked items are the [gaps below](#gaps-from-current-progress-road-to-the-north-star):

- [x] **Symplectic, O(n), fixed + floating base** — DRNEA residual + RIQN
      root-find + Featherstone ABI inverse-mass; energy-bounded with no secular
      drift over 1e5 steps; manifold-correct SE(3)/SO(3) retraction. _(A1/A2/B1)_
- [x] **Holonomic constraints** — Point / Distance / Rigid loop closures as
      impulse-based Newton projections reusing the O(n) ABI. _(B2)_
- [x] **Public surface + visual proof** — dartpy integration-family selector;
      scalable `MultibodyOptions` config; GUI demo scenes for visual verification
      (passive chain, floating tumbler).
- [ ] **Contact & friction** — the deferred Phase C: compliant/penalty →
      augmented-Lagrangian bounded force → (optional) IPC barrier, all as forces
      in the forced DEL residual so symplectic structure + O(n) survive.
      **C2 compliant ground contact has landed** (a real, configurable
      link-point-vs-analytic-ground query — `makeVariationalGroundContactHook`);
      C1 friction, C3 augmented-Lagrangian, and link-vs-link remain
      _(the largest remaining gap)_.
- [x] **Scales to extreme chains** — the exact recursive-Jacobian preconditioner
      (paper Appendix) lands ~3 iterations independent of length, verified to 128
      links; supported bounds in [`supported-envelope.md`](supported-envelope.md).
- [x] **Manifold-aware convergence acceleration** — tangent-space Anderson for
      spherical/floating chains (Euclidean preconditioner + manifold Anderson).
- [ ] **Graduation** from `experimental` to a supported solver (variable `Δt`,
      GPU/batched execution are open stretch goals).

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
  - [x] Integration-family **selector** (`World::setMultibodyOptions`, a
        `MultibodyOptions` value object)
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
- [x] **Phase C — Contact & friction: C2 compliant ground contact landed.**
      Progression in the plan sidecar: NO-GO (2026-05-28) → gate-2 spike GO →
      **C2 real compliant ground contact** (`makeVariationalGroundContactHook`,
      2026-05-30). C1 friction, C3 augmented-Lagrangian, and link-vs-link contact
      (the full gate-1 broad-phase workstream) remain.

## GUI Demos (visual verification)

Two py-demos scenes exercise the VI through the viewer (e.g.
`pixi run py-demos -- --scene sx_variational_chain`):

- **`sx_variational_chain`** — a 5-link passive revolute chain released from
  horizontal; the symplectic VI keeps it swinging with no secular energy loss
  (contrast the dissipative semi-implicit `sx_articulated`).
- **`sx_variational_tumbler`** — a torque-free floating asymmetric body in zero
  gravity, tumbling without energy/momentum drift (the Phase B1 floating-base
  path).

Both select the VI via `World.multibody_options`; the headless cycle smoke
(`python/tests/integration/test_demos_cycle.py`) builds and steps them. More
scenes (a loop-closure scene now that B2 is done; a contact scene once Phase C
lands) are north-star surface.

## Original Phase-A1 Goal (achieved)

A correct, tested variational-integrator stage in the experimental `World`,
selectable by the `variational integrator` method name, proving symplectic
energy behavior on a passive chain before optimizing to O(n). The
[North Star](#north-star) now captures the full vision beyond this MVP.

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

## Gaps From Current Progress (Road to the North Star)

All committed PLAN-082 phases (A1, A2, B1, B2), every acceptance gate, the paper
experiment replication
([`paper-experiment-replication.md`](paper-experiment-replication.md)), the
scalable `MultibodyOptions` config, the zero-default-overhead guarantee, and the
dartpy + GUI surface are complete and verified. The roadmap follow-ups also
landed: the **exact recursive-Jacobian preconditioner** (~3 iters to 128 links),
**manifold tangent-space Anderson**, internal spatial-algebra **dedup**
(`detail/multibody_spatial_algebra.hpp`), the **loop-closure GUI scene**, and the
**supported-envelope** + **performance** characterization docs. The remaining
gaps to the [north star](#north-star), in priority order:

1. **Phase C — contact & friction (the largest gap).** The VI is contact-free in
   production today. Unblocking needs a _contact-query-at-trial-configuration_
   redesign (cheap distance/gradient at an arbitrary trial `qᵏ⁺¹` inside the RIQN
   loop; today `World::collide()` rebuilds the whole collision world once per
   step with no such query) — **scoped in gate 1** of the
   [contact roadmap](../../plans/082-variational-integrator-solver/contact-roadmap.md).
   A compliant-contact robustness **spike cleared gate 2 (GO** for the
   compliant/AL rungs, `k ≲ 1e4·mg` — see [`supported-envelope.md`](supported-envelope.md));
   an opt-in in-loop `VariationalContactHook` exists (default-off byte-for-byte
   identical). **C2 compliant ground contact has now landed** — a real,
   configurable link-point-vs-analytic-ground query
   (`makeVariationalGroundContactHook`: an analytic half-space + body-fixed
   contact points, VBD/XPBD quadratic penalty, reduced-coordinate glue), verified
   to rest at `mg/k` and to hold a swinging revolute tip off the plane. Remaining:
   **C1** lagged friction, **C3** augmented-Lagrangian bounded force, and
   **link-vs-link contact** (the full gate-1 persistent-broad-phase workstream,
   owned with PLAN-081); barrier (C4) last — stiff curvature mis-scales the
   `Δt·M⁻¹` quasi-Newton.
2. **Graduation to a supported solver.** The [graduation
   checklist](graduation-criteria.md) now has the convergence-at-scale and
   performance-characterization criteria met (this round); the open items are
   contact-or-an-explicit-contact-free-envelope and the API freeze + deprecation
   policy. Variable time step and GPU/batched execution stay explicit
   non-blockers (stretch).
3. **Manifold preconditioner for very long _floating_ chains.** The exact
   recursive-Jacobian preconditioner is Euclidean-only; spherical/floating chains
   use the manifold Anderson (verified to 20 links). A Lie-group extension of the
   preconditioner would scale floating chains the way the Euclidean one does.
4. **Contact demo scene.** A GUI contact scene once Phase C lands (the
   loop-closure scene is done).

Code is the source of truth; keep this file lean and current.
