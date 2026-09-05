# PLAN-080: Rigid-Body Dynamics Solver

- Operating state: `PLAN-080` in [dashboard.md](dashboard.md).
- Outcome: dependable free-rigid and articulated dynamics behind a simple
  DART-owned facade, with interchangeable compatible methods.
- Current evidence: the original rigid MVP shipped in PR #2705. The current
  source additionally contains native collision, articulated loading and
  dynamics, unified rigid/articulated constraints, and SI/IPC/VBD/AVBD rigid
  selections. These are implemented slices, not completion of the
  [M1-M3 readiness gates](040-dart7-release-hardening.md).
- Gap evidence: [architecture assessment](../design/dart7_architecture_assessment.md)
  and [solver architecture](../design/simulation_solver_architecture.md).
  `dart/simulation/compute/rigid_body_integration_stage.cpp` currently applies
  inverse inertia to torque without the general gyroscopic term; the SI
  contact stage uses a tangential friction pyramid. Existing tests cover
  selected forces, contacts, and energy, not the complete M1 manifest.

## Ownership And Sequence

This plan owns model/state and rigid physics. PLAN-030 owns runtime/kernel
selection and complete CUDA execution, PLAN-041 owns public checkpoint
workflows, PLAN-122 owns allocation qualification, and PLAN-040 coordinates
readiness. Existing implementation detail remains in
`docs/dev_tasks/rigid_body_dynamics_solver/` while that task is active.
Its README and RESUME route implementation through the prerequisites below;
older contact-polish sequences remain historical backlog, not a bypass.

The CPU reference comes first so physical errors are separable from backend
errors. All M1 examples still require a complete CUDA implementation before
M1 closes. DART 6 is an optional differential reference from `release-6.*`,
not the physics oracle. Existing promotion checker requirements remain in
force until WP-040.2 explicitly migrates them.

## M1 Work Packets

Packets are unclaimed future work; use the
[orchestration lifecycle](../ai/orchestration.md). Acceptance uses the fixed
RB-01 through RB-07 manifest from WP-040.1, never tolerances fitted to output.

### WP-080.1 Complete Rigid Model And State Ownership

- Objective/value: prove reusable, cache-friendly data ownership before physics
  and device paths harden around incomplete state.
- Scope: `comps/`, `detail/world_storage.hpp`, rigid batch storage, bake/reset,
  full rigid pose/velocity and control blocks, and their focused tests.
- Architecture impact: rigid model/state ownership, physical update semantics and
  continuation; update the assessment and supported/allocation rows.
- Non-goals: a public generic ECS, all-domain storage, or implementing FEM.
- Assumptions/open decisions: align contiguous/strided views, alignment and
  index widths with WP-030.3; immutable model and mutable state remain distinct.
  Sharing one model must not replicate its mass/inertia/geometry per state.
- Acceptance evidence: one model serves two independent states; interleaving,
  changing inputs, resetting/restoring one state, and rebuilding the model do
  not corrupt the other. Compare with isolated runs; reject mismatched model
  fingerprints and stale handles after clear/repopulate. All free-rigid
  orientation/angular-velocity state is enumerable and serializable. Record
  configuration-storage, tangent/velocity and control dimensions/offsets
  independently; never assume `nq == nv` or state size `2 * ndof`. Quaternion
  rigid state has seven pose scalars and six velocity scalars while the
  configuration manifold has six DOFs. Test those dimensions, manifold-aware
  orientation updates and dimension-preserving snapshot round trips. A tiny
  variable-length block/layout test challenges index and schema assumptions
  without adding a second physical solver. Views have documented ownership,
  lifetime, host/device residency, and mutation rules.
- Gates: focused model/state, handle, bake/reset and serialization tests;
  `pixi run check-api-boundaries`; relevant PLAN-122 allocation regression gates.
- Dependencies: accepted WP-040.1 and WP-030.3.

### WP-080.2 General Rigid Dynamics And Force Semantics

- Objective/value: establish an independent CPU physical reference for every
  unconstrained M1 example, including asymmetric rotation.
- Scope: rigid integration, inertia transforms, manifold-aware orientation
  updates, force/torque frame conventions, one-shot/persistent input lifecycle,
  public C++/Python scene builders, and RB-01/02/03 tests.
- Architecture impact: rigid model/state ownership, physical update semantics and
  continuation; update the assessment and supported/allocation rows.
- Non-goals: exact finite-step energy conservation, high-order integrator
  portfolios, contact, or articulated refactoring.
- Assumptions/open decisions: semi-implicit translation; choose and document
  a consistent rotational update using body angular momentum or the equivalent
  Euler equation `I ωdot + ω × (I ω) = τ` with correct frame transport.
- Acceptance evidence: analytic linear and principal-axis cases pass;
  asymmetric torque-free motion agrees with an independently converged
  reference. World angular momentum, quaternion norm, energy error, and
  timestep convergence meet the frozen budgets. Offset forces produce the
  expected moment arm; zero inputs and reset preserve the declared lifecycle.
- Gates: RB-01/02/03 tests, focused existing integration and dartpy tests,
  `pixi run check-api-boundaries`, and assessed simulation evidence.
- Dependencies: accepted WP-080.1.

### WP-080.3 M1 Contact And Material Contract

- Objective/value: make ground interaction correct and reproducible before
  scaling to body-body contact graphs.
- Scope: native sphere/box-plane queries, stable manifold ordering, SI normal
  impulses, circular Coulomb tangential projection, restitution/stabilization,
  warm-start continuation, and RB-04 through RB-07 C++/Python scenes.
- Architecture impact: rigid model/state ownership, physical update semantics and
  continuation; update the assessment and supported/allocation rows.
- Non-goals: arbitrary-mesh CCD, stacking performance, a new IPC/VBD family,
  or changing a named research variant without its own evidence.
- Assumptions/open decisions: use the material rules and discrete speed/shape
  envelope frozen in WP-040.1. M1 uses one friction coefficient; static/dynamic
  coefficients and richer material laws require separate intake.
- Acceptance evidence: normals and contact points match geometry references;
  oblique/face/edge/corner impulses, restitution, resting support, and friction
  thresholds meet the manifest. Rotate the tangent basis and sliding direction
  to expose pyramid anisotropy. Track flight, physical impulse, friction, and
  positional-correction energy separately. Contact ordering and warm-start
  invalidation are deterministic in the declared same-backend mode. Unsupported
  configurations and capacity overflow fail explicitly before partial updates.
- Gates: RB-04/05/06/07 tests and existing contact/cross-family regressions;
  `pixi run check-api-boundaries`; assessed headless visual evidence.
- Dependencies: accepted WP-080.2. WP-030.5 consumes this CPU contract.

## M2 And M3 Workstreams

These are prioritized follow-ups, not executable packets until the named
milestone manifest is accepted and scoped packets are added here.

1. **M2 collision and capacity:** dynamic body-body sphere/box contacts and
   100-body plane/box scenes, then triangulated bowl contacts. Admit mesh
   behavior explicitly; exercise contact growth, overflow, manifold churn,
   rest, reset, and snapshot continuation before performance tuning.
2. **M2 optimization:** profile broadphase, narrowphase, islands, solve,
   integration, and transfers; compare brute-force small-scene and native
   AABB-tree references before selecting an accelerator broadphase. Compare
   batched independent worlds separately from one interacting world. PLAN-030
   owns graph grouping and scheduling measurements at matched accuracy.
3. **M3 articulated qualification:** analytical pendulum and chain dynamics,
   joint limits/actuators/loop closures, and rigid-articulated contact. Audit
   which advertised O(n) operators are actually used; measure scaling and
   compare independent inverse/forward dynamics identities.
4. **M3 usable research workflow:** import representative models, inspect
   diagnostics/observations, control/reset, checkpoint, and compare supported
   method variants through the same public scene. Remaining loader diagnostics,
   visual/material import, mimic relations and actuator gaps get evidence-led
   packets rather than blanket inherited feature-completeness claims.

## Acceptance And Revision Rules

M1 packets do not replace the complete targets of PLAN-081/082/083/084/104.
The [solver intake](solver-family-intake.md) applies before another method or
major component. Existing boxed-LCP utilities and native collision are reuse
candidates; their completion does not make new physical envelopes correct.

Every closing packet updates the architecture assessment, supported capability
matrix, and allocation rows it changes. Reopen affected evidence when state
layout, contact law, solver variant, execution ordering, precision, or snapshot
schema changes. Public policy values may express method/device preferences;
solver objects, coupling machinery, ECS, and runtime types stay private.
