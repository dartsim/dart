# PLAN-082: Linear-Time Variational Integrator (Discrete-Mechanics Solver)

- Operating state: `PLAN-082` in [`dashboard.md`](dashboard.md)
- Outcome: the DART 7 `World` offers a **variational integrator** as a
  selectable integration family for multibody systems — symplectic,
  near-energy-conserving (for smooth conservative forcing at fixed `Δt`), and
  linear-time — implementing Lee/Liu/Park/Srinivasa (WAFR 2016,
  arXiv:1609.02898) behind the existing method-name facade, without exposing
  solver, stage, component, or backend types.
- Current evidence:
  - Durable design + math rationale and the honest new-vs-reused split:
    [`../design/simulation_variational_integrator.md`](../design/simulation_variational_integrator.md).
  - The current DART 7 multibody path is **semi-implicit Euler** with a **dense**
    mass-matrix solve (`dart/simulation/compute/multibody_dynamics.cpp`
    `computeMassAndBias` + `ldlt`); there is **no ABA/ABI** and **no
    integrator-selection mechanism** today (adversarial review, 2026-05).
  - Reusable O(n) spatial-algebra toolkit exists (`adjoint`, `motionCross`,
    `spatialInertia`, `rotationExp/Log` (SO3), `jointSubspaceInJointFrame`,
    `DynamicsTree`/`buildDynamicsTree`); twist/inertia conventions match the
    paper (verified).
  - Two-step-history precedent in-tree: `DeformableNodeState.previousPositions`
    (`compute/world_step_stage.cpp`).
  - Reference implementation (classic DART) and the paper's validation
    experiments: `github.com/jslee02/wafr2016` (`dm_DRNEA_*`,
    `computeImpulseForwardDynamics`, `experiments/`).

## Owner Docs

- Architecture/math rationale:
  [`../design/simulation_variational_integrator.md`](../design/simulation_variational_integrator.md)
- Solver architecture this plugs into:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
- Public facade / capability matrix:
  [`../design/simulation_cpp_api.md`](../design/simulation_cpp_api.md)
- Contact/friction/inequality-constraint roadmap (deferred, go/no-go gated):
  [`082-variational-integrator-solver/contact-roadmap.md`](082-variational-integrator-solver/contact-roadmap.md)
- Research catalog entry: `lee-vi-2016` in
  [`../readthedocs/papers.md`](../readthedocs/papers.md)
- Active implementation tracking: `docs/dev_tasks/variational_integrator_solver/`
  (created when Phase A1 implementation starts).

## Relationship To Other Plans

- **PLAN-080 (rigid-body solver)** owns floating-base articulated dynamics and
  is the natural owner of the shared O(n) impulse-ABI; PLAN-082 Phase A2/B1
  coordinate with it rather than forking that work.
- **PLAN-081 (deformable IPC)** owns the barrier/CCD/tangent kernels that
  PLAN-082's deferred contact phase would adapt.
- The VI is additive and opt-in; the default integration family stays
  semi-implicit Euler.

## Workstreams (sequenced)

Each phase lands as small verifiable slices; see the dev-task roadmap (once
created) for slice-level detail.

1. **Phase A1 — Fixed-base MVP, correctness-first (dense solve).**
   Integration-family selector (pipeline substitution, not stage append); SE(3)
   `exp`/`log`, `dlog_V`/`dexp⁻¹` + transpose, `dAdT`; the DRNEA residual
   recursion; the RIQN outer loop with a **dense `M.ldlt().solve(Δt·e)`
   placeholder**, `max_iter`, and a defined non-convergence error; two-step
   previous-configuration State (+ serialization version bump, bootstrap-done
   flag); articulated total-mechanical-energy helper. Fixed-base open chains
   (revolute/prismatic), fixed `Δt`, passive + actuated. **No O(n) claim.**
2. **Phase A2 — O(n) impulse-based ABI.** Implement and verify a Featherstone
   articulated-body-inertia recursion (modeled on the reference
   `computeImpulseForwardDynamics`); replace the dense inverse-mass apply with
   the O(n) `Δt·M⁻¹·e`. Unlocks the linear-time claim and the scaling gate.
3. **Phase B1 — Floating base.** Extend DRNEA/RIQN/ABI to a floating root,
   including manifold-correct velocity reconstruction and RIQN retraction for
   `SE(3)`/`SO(3)` joints (net-new; the reference impl has open TODOs here).
4. **Phase B2 — Holonomic constraints (loop closures).** A constraint Jacobian
   `∂g/∂q` from the existing body Jacobians; an impulse-based constraint-force
   solve folded into RIQN (the paper's Sec. 5 sketch); make the loop-closure
   `Solve` dynamics policy supported (currently rejected at step time) for the
   variational stage only.
5. **Phase C — Contact & friction (DEFERRED, separate go/no-go).** Per the
   contact-roadmap sidecar; blocked on a contact-query-at-trial-configuration
   redesign. Not part of the initial commitment.
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
  `tests/unit/simulation/experimental/` tests, `test-py` when bindings change,
  benchmark JSON for performance claims, changelog entry.

## Revision Triggers

- The Phase A2 ABI spike changes the feasibility or cost of the O(n) claim.
- PLAN-080 delivers floating base or a shared ABI, re-sequencing B1/A2.
- The contact-query redesign (Phase C prerequisite) is scoped or lands.
- The solver-architecture lifecycle (`finalize`, substep windowing) lands,
  changing the State/selection seam.
- A phase completes, splits, or is reprioritized against other DART 7
  clean-break work.

## Next Step

Run the Phase A2 de-risking spike **first** (a standalone, verified O(n)
impulse-ABI over `DynamicsTree`, benchmarked `Δt·M⁻¹·e` vs DOF) to confirm the
linear-time premise, then start Phase A1 under a new
`docs/dev_tasks/variational_integrator_solver/` folder. If ABI proves
infeasible to commit, rescope to the dense placeholder and drop the linear-time
claim before writing VI code.
