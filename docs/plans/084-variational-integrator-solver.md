# PLAN-084: Linear-Time Variational Integrator (Discrete-Mechanics Solver)

- Operating state: `PLAN-084` in [`dashboard.md`](dashboard.md)
- Outcome: the DART 7 `World` offers a **variational integrator** as a
  selectable integration family for multibody systems — symplectic,
  near-energy-conserving (for smooth conservative forcing at fixed `Δt`), and
  linear-time — implementing Lee/Liu/Park/Srinivasa (WAFR 2016,
  arXiv:1609.02898) behind the existing method-name facade, without exposing
  solver, stage, component, or backend types.
- Current evidence:
  - Durable design + math rationale and the honest new-vs-reused split:
    [`../design/simulation_variational_integrator.md`](../design/simulation_variational_integrator.md).
  - Phases A1/A2/B1/B2 landed behind the `MultibodyOptions` method-family
    selector (`WorldOptions::multibodyOptions`, `World::setMultibodyOptions`,
    and dartpy `multibody_options`), with no public solver/stage/component leak.
  - The O(n) inverse-mass path is implemented as
    `computeMultibodyInverseMassProduct`; the dense mass-matrix path remains an
    oracle/baseline, not the selected linear-time VI path.
  - Phase C C1-C3 landed for the scoped envelope: link-point-vs-analytic-ground
    contact, lagged friction, augmented-Lagrangian centering, serialized dual
    state, and a link-vs-link sphere-sphere hook. Arbitrary link geometry and
    the optional C4 hard barrier remain future work.
  - Durable implementation evidence lives in the PLAN-084 sidecar directory:
    [`084-variational-integrator-solver/paper-experiment-replication.md`](084-variational-integrator-solver/paper-experiment-replication.md),
    [`084-variational-integrator-solver/supported-envelope.md`](084-variational-integrator-solver/supported-envelope.md),
    [`084-variational-integrator-solver/performance.md`](084-variational-integrator-solver/performance.md),
    and
    [`084-variational-integrator-solver/graduation-criteria.md`](084-variational-integrator-solver/graduation-criteria.md).

## Owner Docs

- Architecture/math rationale:
  [`../design/simulation_variational_integrator.md`](../design/simulation_variational_integrator.md)
- Solver architecture this plugs into:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
- Public facade / capability matrix:
  [`../design/simulation_cpp_api.md`](../design/simulation_cpp_api.md)
- Contact/friction/inequality-constraint roadmap:
  [`084-variational-integrator-solver/contact-roadmap.md`](084-variational-integrator-solver/contact-roadmap.md)
- Research catalog entry: `lee-vi-2016` in
  [`../readthedocs/papers.md`](../readthedocs/papers.md)
- Paper experiment replication:
  [`084-variational-integrator-solver/paper-experiment-replication.md`](084-variational-integrator-solver/paper-experiment-replication.md)
- Supported envelope:
  [`084-variational-integrator-solver/supported-envelope.md`](084-variational-integrator-solver/supported-envelope.md)
- Performance characterization:
  [`084-variational-integrator-solver/performance.md`](084-variational-integrator-solver/performance.md)
- Graduation criteria:
  [`084-variational-integrator-solver/graduation-criteria.md`](084-variational-integrator-solver/graduation-criteria.md)

## Relationship To Other Plans

- **PLAN-080 (rigid-body solver)** owns floating-base articulated dynamics and
  is the natural owner of the shared O(n) impulse-ABI; PLAN-084 Phase A2/B1
  coordinate with it rather than forking that work.
- **PLAN-081 (deformable IPC)** owns the barrier/CCD/tangent kernels that
  PLAN-084's deferred contact phase would adapt.
- The VI is additive and opt-in; the default integration family stays
  semi-implicit Euler.

## Workstreams (sequenced)

Each phase lands as small verifiable slices; this plan and its sidecars own the
durable state after the implementation dev task retires.

1. **Phase A1 — Fixed-base MVP, correctness-first (implemented).**
   Integration-family selector (pipeline substitution, not stage append); SE(3)
   `exp`/`log`, `dlog_V`/`dexp⁻¹` + transpose, `dAdT`; the DRNEA residual
   recursion; the RIQN outer loop with a **dense `M.ldlt().solve(Δt·e)`
   placeholder**, `max_iter`, and a defined non-convergence error; two-step
   previous-configuration State (+ serialization version bump, bootstrap-done
   flag); articulated total-mechanical-energy helper. Fixed-base open chains
   (revolute/prismatic), fixed `Δt`, passive + actuated. **No O(n) claim.**
2. **Phase A2 — O(n) impulse-based ABI (implemented).** Implement and verify a Featherstone
   articulated-body-inertia recursion (modeled on the reference
   `computeImpulseForwardDynamics`); replace the dense inverse-mass apply with
   the O(n) `Δt·M⁻¹·e`. Unlocks the linear-time claim and the scaling gate.
3. **Phase B1 — Floating base (implemented).** Extend DRNEA/RIQN/ABI to a floating root,
   including manifold-correct velocity reconstruction and RIQN retraction for
   `SE(3)`/`SO(3)` joints (net-new; the reference impl has open TODOs here).
4. **Phase B2 — Holonomic constraints (loop closures, implemented).** A constraint Jacobian
   `∂g/∂q` from the existing body Jacobians; an impulse-based constraint-force
   solve folded into RIQN (the paper's Sec. 5 sketch); make the loop-closure
   `Solve` dynamics policy supported (currently rejected at step time) for the
   variational stage only.
5. **Phase C — Contact & friction (C1-C3 implemented; broader geometry
   deferred).** Per the contact-roadmap sidecar: the supported envelope now has
   ground point contact, lagged friction, AL centering, and a sphere-sphere link
   slice. The remaining PLAN-scale work is arbitrary link-geometry candidate
   generation/query-at-trial-configuration and the optional C4 hard barrier.
6. **Surface & docs (incremental).** dartpy selector binding + import coverage;
   a headless example/GUI smoke once a demonstrative scene exists; capability
   matrix and changelog updates per phase.

## Acceptance Criteria

Gates are binary and evidence-backed; thresholds are initial and tuned with
recorded measurements, mirroring PLAN-080/081 rigor.

- **Analytic correctness (A1):** one VI step of a single-DOF revolute pendulum
  and a prismatic free-fall matches the closed-form acceleration/velocity/
  position within `1e-9` (cf. existing `test_world.cpp` analytic gates).
- **Symplectic behavior (A1, headline):** for a passive ≥10-link chain over
  `N ≥ 1e5` fixed-`Δt` steps, total mechanical energy stays within a bounded
  band `|E(t)−E(0)|/|E(0)| ≤ 1%` **with no secular drift** (fitted energy-vs-time
  slope ≈ 0 within noise), while semi-implicit Euler on the same scene drifts
  monotonically beyond that band. The no-secular-drift slope test is the
  falsifiable gate.
- **Momentum (A1):** linear/angular momentum of a force-free floating test (or
  conserved component for the fixed-base case) conserved to `1e-9` per step.
- **Convergence (A1):** RIQN residual decreases monotonically within a step;
  mean iterations `≤ 8` and max `≤ max_iter` on the gate scene, reported via
  solver diagnostics and asserted in test; non-convergence raises the documented
  error (no silent NaN/fallback).
- **Determinism & serialization (A1):** repeated runs from identical state and
  options are bit-identical; binary save/load round-trips the trajectory and
  does **not** re-bootstrap history.
- **Linear-time scaling (A2 only):** measured per-step cost vs DOF over
  `n ∈ {10,…,100}` has a log-log slope `≤ 1.2` (sub-quadratic), recorded as
  Google Benchmark JSON; the dense A1 path is the O(n³) comparison baseline.
- **Floating base (B1):** analytic free-fall/free-rotation of a floating base
  matches closed form; manifold updates verified against finite differences.
- **Constraints (B2):** a closed-loop (e.g., four-bar) holds its closure
  residual below tolerance over a long horizon; constraint Jacobian verified by
  finite differences; the semi-implicit path still correctly rejects `Solve`.
- **Facade (all phases):** `pixi run check-api-boundaries` stays green; no
  public exposure of `Solver`/stage/`PhysicsDomain`/component/backend types or
  ECS storage; selection is by documented method-family name only; unsupported
  requests return documented errors.
- **Gates per PR:** `pixi run lint`, `pixi run build`, focused
  `tests/unit/simulation/` tests, `test-py` when bindings change,
  benchmark JSON for performance claims, changelog entry.

## Revision Triggers

- The Phase A2 ABI spike changes the feasibility or cost of the O(n) claim.
- PLAN-080 delivers floating base or a shared ABI, re-sequencing B1/A2.
- The contact-query redesign (Phase C prerequisite) is scoped or lands.
- The solver-architecture lifecycle (`finalize`, substep windowing) lands,
  changing the State/selection seam.
- A phase completes, splits, or is reprioritized against other DART 7
  clean-break work.

## Current State

The former implementation dev task has landed on `main`: the selector, O(n)
inverse-mass path, floating/spherical support, loop closures, scoped C1-C3
contact/friction, dartpy/demo surface, supported envelope, performance
characterization, and paper experiment replication are complete. The sidecars in
[`084-variational-integrator-solver/`](084-variational-integrator-solver/) are
the durable evidence for those claims.

Remaining work is deliberately outside the retired implementation task:

- **Graduation proposal:** open a maintainer-owned `PLAN-` entry that links the
  graduation criteria, supported envelope, and performance evidence, then run
  the adversarial review before any `experimental -> supported` structural
  promotion.
- **Arbitrary link-geometry contact:** start a separate PLAN-scale adapter
  workstream coordinated with PLAN-081/PLAN-082 to evaluate signed distance and
  distance gradients at trial configurations inside the VI RIQN loop.
- **Stretch numerics:** a manifold extension of the exact recursive-Jacobian
  preconditioner for very long spherical/floating chains. Variable `dt`,
  GPU/batched execution, and the optional C4 hard IPC barrier remain non-blocking
  future work.

## Next Step

Continue from the durable PLAN-084 evidence: either open the maintainer-owned
graduation proposal that links
[`graduation-criteria.md`](084-variational-integrator-solver/graduation-criteria.md),
[`supported-envelope.md`](084-variational-integrator-solver/supported-envelope.md),
and [`performance.md`](084-variational-integrator-solver/performance.md), or
start the separate arbitrary-geometry contact adapter workstream coordinated
with the rigid IPC / deformable IPC geometry stack. Do not rerun the old Phase
A2 de-risking sequence as if the ABI/selector were still missing.
