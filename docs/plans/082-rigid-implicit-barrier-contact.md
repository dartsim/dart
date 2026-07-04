# PLAN-082: Rigid Implicit-Barrier Contact Solver

- Operating state: `PLAN-082` in [`dashboard.md`](dashboard.md)
- Outcome: the DART 7 `World` supports a DART-owned rigid-body
  implicit-barrier contact method that extends incremental potential contact to
  rigid bodies, runs alongside the current rigid and deformable DART 7
  domains, and covers the reference paper's fixtures, tests, benchmarks,
  comparison baselines, CPU/GPU evidence, and visual examples without exposing
  solver registries, ECS storage, or backend resources through the public
  facade.
- Current evidence:
  - The reference implementation is `ipc-sim/rigid-ipc`, audited at commit
    `23b6ba6fbf8434056444ae106356fd2209136988`.
  - The paper and project page describe an implicit time-stepping method for
    rigid-body contact and friction that models curved trajectories, uses
    conservative CCD with minimum separation, extends incremental potential
    contact to reduced coordinates, and introduces a rigid-body simulation
    benchmark.
  - DART's current DART 7 rigid path has semi-implicit velocity/position
    stages, native collision queries, sequential rigid contact impulses, partial
    articulated contact response, and a deformable IPC-inspired first slice, but
    it does not yet have rigid IPC barriers, curved-trajectory CCD, projected
    Newton, lagged friction, fixture import, or same-domain method selection.
  - The row-level upstream inventory lives in
    [`082-rigid-implicit-barrier-contact/rigid_ipc_fixture_manifest.json`](082-rigid-implicit-barrier-contact/rigid_ipc_fixture_manifest.json)
    and currently tracks 798 upstream rows: 300 fixture JSON paths, 405 CCD test
    data files, 8 test source families, 8 benchmark scripts, and 77 comparison
    baseline files.
  - The first internal fixture reader covers mesh-path rigid bodies, pose,
    scale, velocity, force, fixed DOFs, static/kinematic mode metadata, gravity,
    friction, restitution, barrier distance, solver tolerance, and explicit
    unsupported-field diagnostics.
  - The first internal fixture replay path populates a DART 7 `World`
    with timestep, gravity, rigid body state, loads, and material coefficients
    while preserving mesh path, scale, fixed-DOF, density, group, and kinematic
    metadata. It now also preserves fixture torque plus inline
    vertices/edges/polygons. Supported OBJ, OFF, rigid-ipc MSH, binary STL,
    ASCII STL, and polygonal inline geometry now attach as DART 7 mesh
    collision shapes backed by native collision queries; missing and
    unsupported mesh assets remain explicit replay metadata. This still does
    not claim solver behavior. The same fixture model now imports the upstream
    IPC comparison `.txt` shape-row subset for mesh poses, scale, density,
    initial velocities, prescribed velocities, Neumann body forces, friction,
    barrier distance, friction iterations, and tolerance metadata, with
    MSH-backed replay coverage. Path-loaded fixture/script imports remember
    their source directory so upstream relative mesh paths replay without
    caller-supplied asset roots. Comparison-script material Young/Poisson
    values plus energy model, warm-start, self-collision, and gravity-disable
    metadata are represented as fixture metadata. A runtime replay regression
    now steps a fixture-populated `World` through the default
    sequential-impulse policy to keep the importer separated from implicit
    solver-family selection; opt-in IPC stepping is selected through the
    World-level rigid solver policy.
  - The first direct CCD data reader covers upstream edge-edge, edge-vertex,
    and face-vertex JSON rows as pose and primitive records, with an internal
    replay path that dispatches loaded rows to the matching curved CCD query.
    The first expected-TOI evaluator regressions cover upstream-style
    edge-vertex, edge-edge, and face-vertex rows, while the first audited root
    rows (`tests/data/ccd-test-000..003.json`) now have hermetic
    parser/topology and full-step miss regressions. The audited kinematic
    rows (`tests/data/kinematic/ccd-test-000..012.json`) are covered by the
    zero-time-hit guard regression, and the tracked wrecking-ball rows
    (`tests/data/wrecking-ball/ccd-test-000..385.json`) are covered by a
    conservative-TOI truncation regression. Internal residual helpers now cover the
    edge-vertex, edge-edge, and face-vertex parameter-space equations used by
    interval-root rigid CCD, and first parameter-box subdivision queries recover
    those expected contacts within the reference TOI tolerance. Corpus-scale
    reference evaluator parity is still pending.
  - The first internal rigid curved-trajectory ACCD query covers 3D
    face-vertex, edge-edge, and point-edge primitives over linearly interpolated
    rotation vectors. Focused rotational tests show endpoint-linear primitive
    CCD can miss mid-step rigid contact when the endpoints return to their
    initial positions, and the first minimum-separation regressions cover the
    barrier activation-distance path. Remaining CCD gaps include codimensional
    coverage, rigorous interval arithmetic, accepted reference tolerances, and
    direct-row evaluator parity.
  - The first internal rigid barrier scaffold evaluates face-vertex and
    edge-edge primitive barrier terms by transforming local rigid primitives
    along an interpolated rigid pose and then reusing DART's current IPC C2
    clamped-log world-primitive kernel. Focused tests prove parity with the
    shared kernel for translated and rotated rigid poses. A first two-body
    reduced-coordinate chain-rule layer now maps face-vertex and edge-edge
    world-primitive barrier gradients/Hessians into 6-DOF rigid-pose
    coordinates, with local PSD projection and finite-difference coverage. The
    same local barrier scaffold now includes edge-vertex and vertex-vertex
    terms by adding shared point-edge and point-point kernels over the existing
    distance derivatives plus rigid pose-sampled/reduced wrappers. It now also
    assembles cross-body surface constraints into active scene-level row
    records, global dynamic-body gradients, and sparse PSD-projected Hessians,
    and computes the first conservative surface line-search feasibility bound
    over matching start/end surfaces for vertex-vertex, edge-vertex, edge-edge,
    and face-vertex rows. Initial separation violations and indeterminate CCD
    exhaustion are treated as unsafe zero-step results. A first internal
    projected-Newton step helper now consumes the assembled gradient and sparse
    PSD-projected Hessian with diagonal regularization, descent-step
    statistics, and optional line-search bound scaling/blocking. A first
    barrier-only projected-Newton loop now
    recomputes assembly over copied rigid surfaces, updates dynamic surface
    poses, records convergence/progress statistics, and uses the conservative
    line-search bound to scale or block candidate steps. First internal
    generalized dynamics objective terms now add per-body diagonal quadratic
    pose weights and generalized force/torque vectors to that same global
    6-DOF system. A first physical construction helper now maps pose,
    generalized velocity, mass, diagonal inertia, generalized force/torque, and
    timestep into those objective terms. A first opt-in runtime world-step stage
    now extracts mesh/box/sphere free rigid-body state, runs the
    projected-Newton IPC solve, and writes solved poses/velocities back. The
    default DART 7 `World` pipeline now has same-domain rigid solver
    selection, keeping sequential impulse as the default while allowing callers
    to opt into the rigid IPC free-rigid dynamics stage without exposing solver
    registries. The opt-in stage now also exposes durable solve status, last
    step norm, last line-search bound, and aggregate conservative CCD
    line-search counters. A first runtime activated-contact regression now
    verifies mesh barriers move a dynamic body away from a static surface while
    reporting nonzero line-search diagnostics. The runtime extractor now also
    rejects malformed mesh topology, non-finite mesh vertices, and invalid box
    extents before barrier assembly or CCD, and the stage now skips
    non-converged solve results instead of writing partial poses back silently.
    The first lagged smoothed Coulomb friction potentials now cover
    vertex-vertex, edge-vertex, edge-edge, and face-vertex world-coordinate
    terms, with reduced rigid-coordinate coverage for vertex-vertex and
    edge-vertex terms. The projected-Newton objective now assembles first
    lagged friction rows from active lagged barrier constraints and reports
    runtime active-friction diagnostics. The internal solve now supports bounded
    outer lagged-friction passes, zero-iteration friction disable, and refreshed
    momentum-balance/pass diagnostics. Remaining geometry corpus coverage,
    runtime fixture behavior, production convergence criteria, robust contact
    behavior across corpus scenes, and production-ready default activation
    criteria remain open.

## Owner Docs

- Architecture rationale:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
- Rigid-body implementation tracker:
  [`../dev_tasks/rigid_body_dynamics_solver/`](../dev_tasks/rigid_body_dynamics_solver/)
- Active rigid IPC implementation tracker:
  [`../dev_tasks/rigid_ipc_solver/`](../dev_tasks/rigid_ipc_solver/)
- Unified Newton-barrier family:
  [`083-unified-newton-barrier-multibody.md`](083-unified-newton-barrier-multibody.md)
  owns the cross-variant IPC/ABD consolidation plan. PLAN-082 remains the rigid
  IPC variant owner and correctness oracle until an affine/stiff-body path beats
  it on matched fixtures.
- Simultaneous-impact intake and go/no-go sidecar:
  [`082-rigid-implicit-barrier-contact/simultaneous-impact-intake.md`](082-rigid-implicit-barrier-contact/simultaneous-impact-intake.md)
- Research catalog:
  [`../readthedocs/papers.md`](../readthedocs/papers.md)

## Workstreams

1. **Upstream corpus and paper gap control** - Keep the manifest generated from
   the audited upstream commit and validate that it has zero unclassified rows.
   A manifest row is complete only when it names a DART artifact, command,
   expected invariant, and evidence matching the upstream fixture, test,
   benchmark, or comparison responsibility.
2. **Rigid mesh/import state** - Add a DART-owned importer for the rigid-ipc
   fixture subset: mesh path, scale, pose, velocity, force, fixed DOF mask,
   dynamic/static/kinematic mode, timestep, gravity, restitution, friction,
   barrier distance, and solver tolerance. Keep importer diagnostics explicit
   for unsupported fields.
3. **Curved-trajectory CCD** - Implement conservative rigid primitive CCD for
   point-edge, edge-edge, face-vertex, and codimensional cases over rigid
   curved trajectories. Reuse DART's native primitive CCD foundations where
   possible, but add the reduced-coordinate trajectory and minimum-separation
   semantics needed by rigid IPC.
4. **Barrier/contact objective** - Add rigid incremental-potential terms for
   inertia, contact barriers, fixed coordinates, scripted kinematics, forces,
   restitution policy, and diagnostics. Implement clamped barrier derivatives,
   Hessian PSD projection, sparse assembly, and line-search feasibility checks
   before promoting scene-level claims.
5. **Projected Newton and friction** - Add projected Newton with deterministic
   sparse solve/fallback behavior, then lagged smoothed Coulomb friction with
   tangent bases, normal-force lagging, static/dynamic transition diagnostics,
   and convergence tests.
6. **Same-domain method selection** - Let the DART 7 `World` host more
   than one rigid-body method family internally. The easy path should keep the
   current default behavior, while tests can select the implicit-barrier method
   through DART-owned policy names without exposing solver registries or
   external project names.
7. **Benchmarks, visuals, and CPU/GPU evidence** - Port all manifest rows into
   focused tests, C++ benchmarks, fixture replay examples, comparison baselines,
   profiling packets, and headless Filament evidence. GPU work remains private
   and benchmark-gated until the algorithm has a representative workload.
8. **Unified Newton-barrier direction** - Follow PLAN-083 for shared barrier,
   CCD, projected-Newton, friction, diagnostics, CPU/GPU benchmark, and corpus
   verification primitives. Rigid IPC remains the exact/reduced-coordinate
   reference path; ABD or other stiff-body variants must prove matched
   correctness and performance before replacing any rigid IPC responsibility.
9. **Simultaneous-impact intake** - Keep the explicit event-level
   simultaneous-impact papers as a benchmark/go-no-go sidecar rather than a new
   solver commitment. Promote an impact operator only if solver-neutral DART
   scenes show a gap not covered by sequential impulse, boxed LCP, rigid IPC, or
   AVBD-style finite-time contact.

## Acceptance Criteria

- `scripts/check_rigid_ipc_fixture_manifest.py` passes with and without an
  upstream checkout at the audited commit.
- Each manifest row moves from `planned` only when DART has matching code,
  tests or benchmarks, command evidence, and visual/profiling artifacts where
  required.
- The implementation has focused tests for curved-trajectory CCD, minimum
  separation, barrier derivatives/Hessians, line search, projected Newton,
  friction, fixture import diagnostics, restart/serialization, and mixed
  rigid/deformable stepping.
- The DART 7 public facade stays DART-owned and backend-neutral:
  no public solver registry, coupler registry, ECS component, upstream-project
  solver selector, device, stream, or memory-pool type.
- Full method parity is not claimed until the paper fixtures, upstream unit
  tests, benchmark scripts, comparison baselines, visual examples, CPU
  performance packets, and GPU-gated packets are all covered or explicitly
  classified as manual/not-applicable with maintainer-visible rationale.
- Performance completion requires benchmark packets comparing against the
  current DART rigid contact path, the audited reference implementation, and
  the paper-reported scene families; regressions or slower rows need an
  explicit accepted tradeoff rather than a parity claim.
- Any shared primitive promoted by PLAN-083 has cross-variant tests proving
  rigid IPC behavior remains unchanged or that the new behavior is explicitly
  accepted against the audited fixture manifest.
- Simultaneous-impact promotion requires the sidecar's literature matrix,
  solver-neutral corpus, IPC/AVBD comparison, and public-boundary review before
  any event-level operator becomes a PLAN-082 workstream.

## Revision Triggers

- The audited upstream commit changes.
- The rigid IPC fixture schema requires public API beyond current
  DART 7 facade rules.
- Rigid and deformable IPC work diverges in a way that duplicates barrier,
  CCD, Newton, friction, diagnostics, or benchmark infrastructure.
- Hosted or local benchmark evidence shows the implicit-barrier path cannot be
  competitive without a different CPU/GPU data layout.
- Solver-neutral simultaneous-impact scenes show restitution, ordering
  uncertainty, energy, termination, or break-away gaps that rigid IPC and
  AVBD-style finite-time contact do not cover.
- A maintainer decides a manifest family should be manual, out of scope, or
  promoted earlier than this plan's sequencing.
- PLAN-083 changes the shared Newton-barrier primitive contract or promotes ABD
  from evaluation to replacement for a rigid IPC responsibility.

## Progress log

Relocated from the dashboard on 2026-07-03; newest first.

Continue the active dev task from fixture replay, comparison
script ingestion, curved-trajectory CCD/residual/subdivision slices, local
rigid barrier derivatives, scene-level sparse barrier assembly, and
conservative line-search feasibility, and barrier/dynamics Newton solve
scaffolding, first physical dynamics-term construction, opt-in runtime rigid
IPC stage, same-domain `World` rigid solver selection, and runtime sphere
triangulation, durable stage diagnostics, and the first activated-contact
runtime regression plus vertex-vertex line-search CCD, invalid runtime
geometry rejection, explicit non-converged-result skipping, and
primitive-family friction potentials, first lagged friction assembly, and
bounded outer lagged-friction passes into remaining geometry corpus coverage,
runtime fixture behavior, production convergence criteria, production-ready
default activation criteria, mixed-domain coupling, rigorous interval
arithmetic, direct CCD evaluator parity, remaining comparison script
commands, and full fixture/test/benchmark/visual parity. Keep the
simultaneous-impact intake as a PLAN-082 sidecar until solver-neutral scenes
prove a restitution/order-uncertainty gap not already covered by rigid IPC or
AVBD-style finite-time contact. Coordinate any shared primitive extraction or
ABD replacement decision through PLAN-083 before changing the rigid IPC
correctness-oracle role.
