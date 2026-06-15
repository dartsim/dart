# Variational Integrator Solver — Dev Task

Implementation tracking for PLAN-084 (linear-time variational integrator).
Plan: [`../../plans/084-variational-integrator-solver.md`](../../plans/084-variational-integrator-solver.md).
Design: [`../../design/simulation_variational_integrator.md`](../../design/simulation_variational_integrator.md).
Paper experiment replication + measured results:
[`paper-experiment-replication.md`](paper-experiment-replication.md).

## North Star

The durable target this work builds toward — a **production-grade, linear-time
variational integrator**: DART's symplectic, structure-preserving alternative to
semi-implicit Euler for multibody systems, faithful to Lee/Liu/Park/Srinivasa
(WAFR 2016, arXiv:1609.02898) and extended to DART's needs. It lives entirely
behind the DART 7 World's method-name facade
(`WorldOptions::multibodyOptions` at construction or
`World::setMultibodyOptions({.integrationFamily = MultibodyIntegrationFamily::Variational})`),
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
      **C1/C2/C3 are all reachable from `World::step()`** via
      `Multibody::setGroundContact`: lagged regularized-Coulomb friction +
      compliant penalty by default, and the **C3** augmented-Lagrangian drift-free
      rung opt-in via `dualUpdateCadence` (`0` = C2; `N>0` advances the duals
      every `N` steps, persisted in `VariationalContactDualState`). A
      **link-vs-link** sphere-sphere slice (`makeVariationalLinkSphereContactHook`)
      landed; **arbitrary link geometry** (the rigid-IPC-stack adapter) remains
      _(the largest remaining gap)_.
- [x] **Scales to extreme chains** — the exact recursive-Jacobian preconditioner
      (paper Appendix) lands ~3 iterations independent of length, verified to 128
      links; supported bounds in [`supported-envelope.md`](supported-envelope.md).
- [x] **Manifold-aware convergence acceleration** — tangent-space Anderson for
      spherical/floating chains (Euclidean preconditioner + manifold Anderson).
- [ ] **Graduation** from `experimental` to a supported solver — all
      [graduation criteria](graduation-criteria.md) are now met/declared (ready
      to propose); the `experimental → supported` flip needs the maintainer's
      `PLAN-` entry + adversarial review. Variable `Δt`, GPU/batched execution are
      explicit non-blockers (stretch).

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
  - [x] Integration-family **selector** (`WorldOptions::multibodyOptions`,
        `World::setMultibodyOptions`, and the `MultibodyOptions` value object)
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
- [x] **Phase C — Contact & friction: C1-C3 landed** (compliant contact, lagged
      friction, augmented Lagrangian). Progression in the plan sidecar: NO-GO
      (2026-05-28) → gate-2 spike GO → **C1-C3 contact** (compliant +
      regularized-Coulomb friction + AL drift-free contact with Kelvin-Voigt
      damping; `makeVariationalGroundContactHook` /
      `VariationalGroundContactSolver`, 2026-05-30). Link-vs-link contact (the
      full gate-1 broad-phase workstream) remains.

## GUI Demos (visual verification)

Several py-demos scenes exercise the VI through the viewer (e.g.
`pixi run py-demos -- --scene sx_variational_contact`):

- **`sx_variational_chain`** — a 5-link passive revolute chain released from
  horizontal; the symplectic VI keeps it swinging with no secular energy loss
  (contrast the dissipative semi-implicit `sx_articulated`).
- **`sx_variational_tumbler`** — a torque-free floating asymmetric body in zero
  gravity, tumbling without energy/momentum drift (the Phase B1 floating-base
  path).
- **`sx_loop_closure`** — a planar arm with a DISTANCE loop closure held to
  machine precision while it swings (Phase B2).
- **`sx_variational_contact`** — a pendulum tip caught by compliant ground
  contact + friction, configured through the World surface
  (`Multibody.set_ground_contact`; Phase C rungs C1/C2).

All select the VI via `World.multibody_options`; the headless cycle smoke
(`python/tests/integration/test_demos_cycle.py`) builds and steps them.

## Original Phase-A1 Goal (achieved)

A correct, tested variational-integrator stage in the DART 7 `World`,
selectable by the `variational integrator` method name, proving symplectic
energy behavior on a passive chain before optimizing to O(n). The
[North Star](#north-star) now captures the full vision beyond this MVP.

## Non-Goals (early phases)

- O(n) scaling (Phase A1 uses a dense solve; A2 adds ABI).
- Floating base, loop closures, contact/friction (later phases).
- Variable time step; GPU/batched execution.

## Key Decisions (rationale in the design doc)

- **Dense-solve placeholder first** (A1), ABI second (A2): get correctness +
  symplectic-behavior evidence before the hardest kernel. At Phase A1 start,
  reviewers confirmed the DART 7 World had no ABI path yet (only the dense
  `M.ldlt()` oracle); Phase A2 then landed the O(n) inverse-mass product.
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

All committed PLAN-084 phases (A1, A2, B1, B2), every acceptance gate, the paper
experiment replication
([`paper-experiment-replication.md`](paper-experiment-replication.md)), the
scalable `MultibodyOptions` config, the zero-default-overhead guarantee, and the
dartpy + GUI surface are complete and verified. The roadmap follow-ups also
landed: the **exact recursive-Jacobian preconditioner** (~3 iters to 128 links),
**manifold tangent-space Anderson**, internal spatial-algebra **dedup**
(`detail/multibody_spatial_algebra.hpp`), the **loop-closure GUI scene**, and the
**supported-envelope** + **performance** characterization docs. The remaining
gaps to the [north star](#north-star), in priority order:

1. **Arbitrary-geometry contact & friction (the largest remaining gap).** C1-C3
   contact have landed for the supported envelope; what remains is the broader
   _contact-query-at-trial-configuration_ redesign for arbitrary link geometry
   (cheap distance/gradient at an arbitrary trial `qᵏ⁺¹` inside the RIQN loop;
   today `World::collide()` rebuilds the whole collision world once per step
   with no such query) — **scoped in gate 1** of the
   [contact roadmap](../../plans/084-variational-integrator-solver/contact-roadmap.md).
   The implemented ground slice is a real, configurable query
   (`makeVariationalGroundContactHook`:
   analytic half-space + body-fixed points, VBD/XPBD quadratic penalty,
   reduced-coordinate glue), **lagged regularized-Coulomb friction**, and
   **augmented-Lagrangian** drift-free contact (`VariationalGroundContactSolver`:
   per-contact dual + Kelvin-Voigt damping). Verified: rest at `mg/k`, a swinging
   revolute tip held off the plane, a sliding block decelerated by friction, and
   the AL slider centered at `d ≈ 0` (vs the penalty `−mg/k`). **Link-vs-link
   contact has a first slice**: `makeVariationalLinkSphereContactHook` does
   compliant **sphere-sphere self-contact** between links (verified — a link
   sliding into a fixed base sphere is stopped, no pass-through). C1–C3 are all
   reachable from `World::step()` (C3 opt-in via `dualUpdateCadence`). Remaining:
   **arbitrary link geometry** — the rigid IPC stack's `primitive_distance.hpp`
   already gives reusable analytic `(d, ∂d/∂q)` kernels, but the dominant missing
   piece is **rigid/articulated candidate generation** (its `candidate_set.hpp` is
   mesh-vertex specific) plus a warm-started per-step query — a ~2–3-week
   PLAN-scale workstream coordinated with PLAN-081, not an in-task slice (see the
   [contact roadmap](../../plans/084-variational-integrator-solver/contact-roadmap.md)).
   The **C4 IPC barrier** is intentionally **out of scope** — this task stops at
   C3 (stiff barrier curvature mis-scales the `Δt·M⁻¹` quasi-Newton; it is the
   _optional_ last rung).
2. **Graduation to a supported solver.** The [graduation
   checklist](graduation-criteria.md) is now **all met/declared** — including
   Phase C contact (the contact criterion) and the API-freeze surface — so the VI
   is **ready to propose for graduation**. The flip itself is **structural, not a
   toggle**: `experimental` is encoded in the namespace / directory /
   `DART_SIMULATION_API` macro / CMake gate, with no per-family stability flag,
   and the VI shares the module with the `World` and every other solver — so
   graduating means a maintainer-scale whole-module promotion or a VI extraction
   refactor (see the graduation-mechanics section of
   [`graduation-criteria.md`](graduation-criteria.md)), plus the `PLAN-` entry and
   adversarial review. Variable time
   step and GPU/batched execution stay explicit non-blockers (stretch).
3. **Manifold preconditioner for very long _floating_ chains.** The exact
   recursive-Jacobian preconditioner is Euclidean-only; spherical/floating chains
   use the manifold Anderson (verified to 20 links). A Lie-group extension of the
   preconditioner would scale floating chains the way the Euclidean one does.
4. **Contact demo scene — landed.** `sx_variational_contact` (a VI pendulum tip
   caught by the compliant ground plane) joins the chain / tumbler / loop-closure
   scenes in the demos registry and the demos-cycle test.

Code is the source of truth; keep this file lean and current.
