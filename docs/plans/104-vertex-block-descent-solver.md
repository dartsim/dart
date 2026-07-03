# PLAN-104: Vertex Block Descent Solver

- Operating state: `PLAN-104` in [`dashboard.md`](dashboard.md)
- Outcome: the experimental `World` can step deformable bodies with a
  DART-owned Vertex Block Descent (VBD) solver — a per-vertex, graph-colored
  block coordinate descent on the variational implicit-Euler objective — that
  matches the reference implementations for correctness and beats the reference
  and/or paper performance numbers on CPU and GPU, without exposing solver
  registries, backend/project names, ECS storage, or execution resources
  through the public facade.
- Current evidence:
  - PR #2705 provides the rigid-body solver surface, staged `WorldStepPipeline`,
    and the multi-solver architecture this plan extends.
  - PR #2711 added experimental deformable body dynamics: the public
    `DeformableBody` handle, the `comps::Deformable*` ECS components, the
    inertial-target/gravity/damping setup, and `DeformableDynamicsStage`, which
    minimizes the variational implicit-Euler objective with a global
    mass-preconditioned gradient descent plus Armijo line search.
  - PLAN-081's IPC slices added the `detail/deformable_contact` distance,
    barrier, CCD, and tangent kernels that the VBD contact phase will reuse.
  - `docs/design/simulation_solver_architecture.md` defines the solver/coupler
    direction, domain-driven assignment, and model/state/control/contact
    separation that VBD plugs into as a second deformable solver.

## Owner Docs

- Architecture rationale:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
- Public facade:
  [`../design/simulation_experimental_cpp_api.md`](../design/simulation_experimental_cpp_api.md),
  [`../design/simulation_experimental_python_api.md`](../design/simulation_experimental_python_api.md)
- Research references: [`../readthedocs/papers.md`](../readthedocs/papers.md)
  (`chen-2024-vbd`, `tinyvbd`, `gaia`, `ogc-2025`, `avbd-2025`).
- VBD paper/reference gap audit:
  [`104-vertex-block-descent-solver/vbd-paper-gap-audit.md`](104-vertex-block-descent-solver/vbd-paper-gap-audit.md)
  owns the method scope, the component-by-component gap, the elastic-energy
  targets, and the reference performance numbers to beat.
- OGC paper/source gap audit:
  [`104-vertex-block-descent-solver/ogc-gap-audit.md`](104-vertex-block-descent-solver/ogc-gap-audit.md)
  owns the Offset Geometric Contact research and implementation sequence for
  codimensional VBD contact.
- AVBD paper/reference gap audit:
  [`104-vertex-block-descent-solver/avbd-paper-gap-audit.md`](104-vertex-block-descent-solver/avbd-paper-gap-audit.md)
  owns the augmented-Lagrangian hard-constraint extension, demo corpus,
  CPU/GPU parity target, and paper/reference performance numbers to beat.
- AVBD demo/benchmark corpus matrix:
  [`104-vertex-block-descent-solver/avbd-demo-corpus.md`](104-vertex-block-descent-solver/avbd-demo-corpus.md)
  owns the row-by-row source-demo, paper-scene, website/video, and benchmark
  packet tracking needed before any parity claim.

## Current Implementation Evidence

The temporary `docs/dev_tasks/vbd_deformable_solver/` tracker was retired after
the post-merge self-collision follow-up slice because the durable method audit
and remaining work now live here. PR #2781 landed the DART-owned VBD CPU+CUDA
solver path: per-vertex block kernels, graph coloring, colored Gauss-Seidel
block descent, Stable Neo-Hookean tetrahedra, Chebyshev/Rayleigh acceleration,
World solver selection, static ground contact + Coulomb friction, CPU baseline
benchmarks, CUDA mass-spring/tetrahedral rollouts, and the first GUI showcases.
PR #2801 extends that landed path so the World VBD solver honors shared FEM
tetrahedral material kernels, handles static sphere/box obstacle barriers,
adds lagged VT/EE surface self-collision penalties, and provides the TinyVBD
tilted-strand plus contact showcase py-demos.

Remaining durable work is deliberately narrower than the retired task tracker:
self-contact tangential friction, committed benchmark/profiling JSON, paper
tetrahedral scene reproduction, OGC source/code audit plus CPU proof-of-contact,
Phase 8 SoA plus Gaia CPU comparison, and same-GPU RTX-4090 Table 1
reproduction.

Maintainer direction now extends this same solver-family plan to Augmented VBD
(`avbd-2025`). AVBD is not a replacement for the VBD foundation above; it is
the hard-constraint, friction, rigid/articulated, finite-stiffness-ramp, and
CPU/GPU performance continuation of the VBD path. The active multi-session
implementation tracker is
[`../dev_tasks/avbd_solver/`](../dev_tasks/avbd_solver/). The first local
implementation slice adds a tested internal scalar-row utility for AVBD
regularization, warm starting, hard-row clamping, dual updates, and
finite-stiffness ramping. Follow-on local slices add deterministic scalar-row
keys/inventory for warm-started `lambda`/`k` state, standalone CPU
half-space contact-normal, hard point-attachment, and finite-stiffness spring
row drivers, plus a standalone finite-stiffness tetrahedral material row
driver, a bounded half-space friction-tangent row primitive, and self-contact
normal rows for point-triangle / edge-edge primitive directions and AVBD
hard-row stamping. Self-contact friction tangent rows reuse lagged
point-triangle / edge-edge tangent stencils in the combined mass-spring row
driver, with supported World generation for serial self-contact scenes plus
pairwise static/dynamic switching and circular-cone projection.
The supported mass-spring World envelope carries
contact-normal, friction-tangent, self-contact-normal, attachment, and spring
finite-stiffness families in one serial AVBD row solve. The supported
frictionless pure-tetrahedral World envelope now carries finite-stiffness
material rows with a dimensionless Lamé multiplier and separate tet-row
diagnostics, while still using the existing lagged VBD self-contact penalty
when hard self-contact rows are not requested. That pure-tet envelope can now
also combine finite-stiffness material rows with AVBD self-contact normal rows
and matching bounded self-contact friction tangent rows when requested.
The supported mass-spring friction tangent pairs now use the lagged tangential
dual to switch between static sticking and dynamic sliding and project paired
forces to the circular Coulomb cone. Static box obstacle row keys now include
face/edge/corner feature IDs so rows warm-start across small same-feature
penetrations but reset when contact moves to another box manifold. Persisting
static half-space friction rows also project their decayed tangential dual into
the current tangent basis when smooth obstacle normals change, and persisting
self-contact friction rows project their generalized tangential dual into the
current 12D tangent stencil.
The first private rigid-body foundation adds a 6-DOF block accumulator,
world-frame quaternion tangent updates, an inertia term, a 6x6 block solve, and
a scalar rigid point-attachment row plus two-body point-pair row stamping with
focused tests. Point-pair rows now also carry a scalar offset and private
constructors for rigid contact-normal rows and bounded contact-friction tangent
rows, plus a paired friction helper that switches between static sticking and
dynamic sliding while projecting the force to a circular Coulomb cone. A
private serial rigid row driver now sweeps point attachments,
contact-normal point pairs, and paired friction tangent rows; paired friction
row updates now reuse the regularized tangent constraint vector for both cone
projection and stiffness growth instead of evaluating the same tangent
constraints twice, and shared-anchor tangent pairs reuse the first transformed
anchor pair in the generic force helper, direct block assembly, and row-state
update path. The first private
rigid contact-manifold builder now maps active contact points with stable
endpoint feature IDs into warm-started normal and paired tangent rows. A private
World-contact snapshot helper now maps rigid-body `World::collide()` contacts
into those manifold-point inputs and runs them through the private serial rigid
row solve plus dynamic rigid-body ECS writeback through a combined private
wrapper in focused coverage. The internal `RigidAvbdContactConfig`
contact-stage path now activates supported free rigid-body contacts as a
velocity projection, with focused dynamic/dynamic and static/dynamic
contact-stage coverage in
`World.RigidBodyContactStageAvbdProjectsDynamicDynamicContactVelocity` and
`World.RigidBodyContactStageAvbdProjectsStaticDynamicContactVelocity`,
single-config dynamic/dynamic opt-in coverage in
`World.RigidBodyContactStageAvbdProjectsDynamicPairWithSingleConfig`,
default `World::step()` dynamic/dynamic coverage in
`World.RigidBodyContactStageAvbdDynamicDynamicRunsThroughDefaultWorldStep`,
default `World::step()` single-config dynamic/dynamic coverage in
`World.RigidBodyContactStageAvbdDynamicPairWithSingleConfigRunsThroughDefaultWorldStep`,
contact-to-position pipeline coverage in
`World.RigidBodyContactStageAvbdFeedsRigidBodyPositionStage`,
default `World::step()` schedule coverage in
`World.RigidBodyContactStageAvbdRunsThroughDefaultWorldStep`,
static-owned static/dynamic opt-in coverage in
`World.RigidBodyContactStageAvbdProjectsStaticOwnedContactConfig`,
default `World::step()` static-owned static/dynamic opt-in coverage in
`World.RigidBodyContactStageAvbdStaticOwnedRunsThroughDefaultWorldStep`,
stored static-body velocity ignore coverage in
`World.RigidBodyContactStageAvbdIgnoresStoredStaticVelocity`,
kinematic-as-prescribed endpoint coverage in
`World.RigidBodyContactStageAvbdTreatsKinematicBodyAsStaticObstacle`,
default `World::step()` kinematic-prescribed endpoint coverage in
`World.RigidBodyContactStageAvbdKinematicRunsThroughDefaultWorldStep`,
enabled-peer/disabled-peer opt-in coverage in
`World.RigidBodyContactStageAvbdProjectsEnabledPeerWithDisabledConfig`,
default `World::step()` enabled-peer/disabled-peer coverage in
`World.RigidBodyContactStageAvbdEnabledPeerWithDisabledConfigRunsThroughDefaultWorldStep`,
configured multi-contact coverage in
`World.RigidBodyContactStageAvbdProjectsMultipleConfiguredContacts`, plus
default `World::step()` configured multi-contact coverage in
`World.RigidBodyContactStageAvbdMultipleConfiguredContactsRunThroughDefaultWorldStep`,
mixed-config all-or-nothing fallback coverage in
`World.RigidBodyContactStageAvbdFallsBackForUnconfiguredContactSet`,
default `World::step()` mixed-config all-or-nothing fallback coverage in
`World.RigidBodyContactStageAvbdMixedConfigFallsBackThroughDefaultWorldStep`,
default `World::step()` disabled-config fallback coverage in
`World.RigidBodyContactStageAvbdDisabledConfigFallsBackThroughDefaultWorldStep`,
disabled-config opt-out fallback coverage in
`World.RigidBodyContactStageAvbdDisabledConfigFallsBack`, and
static/dynamic and dynamic/dynamic warm-started friction slip-reduction
coverage in `World.RigidBodyContactStageAvbdWarmStartedFrictionReducesSlide`
and
`World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionReducesSlip`,
static-owned warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionReducesSlide`,
kinematic-owned warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionReducesSlide`,
enabled-peer/disabled-peer warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionReducesSlide`,
simultaneous multi-contact warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionReducesSlide`,
live box-box manifold warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionReducesSlide`,
live dynamic/dynamic box-manifold warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionReducesSlip`,
stacked static/dynamic plus dynamic/dynamic box-manifold warm-started friction
coverage in
`World.RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionReducesSlip`,
multi-top stacked box-manifold warm-started friction contact-stage coverage in
`World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionReducesSlip`,
plus default `World::step()` warm-started friction schedule coverage in
`World.RigidBodyContactStageAvbdWarmStartedFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
and
`World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionRunsThroughDefaultWorldStep`,
while
unsupported envelopes still fall back.
Private dynamic/rigid contact identity helpers now pack contact feature
kind/index IDs, canonicalize two-endpoint row keys, and create normal/friction
row descriptors. Rigid snapshots now derive box face/edge/corner, cylinder
side/cap/rim, and capsule side/top-cap/bottom-cap endpoint features for
narrower private warm-start keys; known and unknown-index contacts now map world
points through body and shape transforms before feature coding, including
explicit endpoint-A/B compound shape-index coverage that uses narrow-phase
shape-local contact points, actual `World::collide()`
sphere/cylinder/capsule/plane/mesh primitive-feature coverage including live
mesh vertex endpoint-feature evidence, unique unknown-index plane compound contacts that
derive shape-scoped face IDs, and triangle-mesh contacts that derive
face/edge/vertex feature IDs. Persisting private rigid contact
friction rows now retain their previous tangent directions and project the
warm-started
paired dual into the current tangent basis when the contact normal rotates, with
both private row-builder coverage and live sphere/plane `World::collide()`
normal-rotation evidence. The
private rigid row path also has
point-joint linear, angular, and combined builders for fixed-anchor
translation/orientation rows, with step-start
previous values seeded for AVBD alpha regularization. Those private point-joint
rows can now be appended to the World rigid snapshot/solve/apply wrapper and
combined step helper from world-space point-joint inputs, and a private
fixed-joint ECS extractor can feed the step helper for rigid-body-linked joint
entities. The internal contact-stage AVBD opt-in can project those fixed-joint
rows with or without active contacts. The private point-joint builders now
accept per-axis linear and angular masks, keeping all-axis fixed-joint rows as
the default. Named private revolute and prismatic point-joint configs now build
arbitrary joint-axis bases, leave one rotational or translational axis free, and
preserve axes/masks through World point-joint input and solve coverage.
Simulation-entry current-pose initialization and extraction also cover private
rigid-body ECS revolute/prismatic joint entities, and non-topology
multibody-link fixed/revolute/prismatic point-joint entities can now generate hard AVBD
configs from the simulation-entry current pose while tree-topology joints remain
skipped. Public DART 7 `World`
facades now expose rigid-body revolute and prismatic joints for free rigid
bodies through C++ and dartpy, with generated stubs, focused C++/Python tests,
and a categorized `sx_rigid_limited_joints` py-demo. Public free-rigid
break-force coverage also now includes binary save/load regressions for broken
fixed/revolute/prismatic/spherical joints' restored contact-filtering
lifecycle, same-multibody/world-link articulated fixed/revolute/prismatic/
spherical broken-state binary round-trips with one-DOF motor-state
preservation, and design-mode binary save/load now preserves public free-rigid
and articulated AVBD point-joint start/linear/angular stiffness facade state
before private row rebuild, with direct C++/dartpy coverage rejecting same-link,
cross-multibody, and cross-world articulated point-joint endpoints. The first
AVBD
rigid-constraint
`py-demos` scene, `avbd_rigid_fixed_joint_contact`, shows the public
fixed-joint/contact slice for users. Public multibody joint extraction and
articulated World facade wiring are now narrowly started by same-multibody
link-link and world-link fixed/revolute/prismatic point-joint facades, with
focused public revolute/prismatic bounded velocity-actuator,
same-multibody/world-anchored command-update, and break/reset lifecycle
coverage including same-multibody/world-anchored public one-DOF motor
break/skip and reset/re-engagement with focused dartpy stepping coverage for
same-multibody/world-link revolute and prismatic explicit-anchor cases,
including post-reset endpoint/axis-shape assertions for non-cardinal
same-multibody pair and world-link revolute/prismatic reset coverage plus
C++/dartpy non-cardinal off-origin finite-effort cap coverage
spanning same-multibody/world-link revolute and prismatic facades plus dartpy
binary round-trip coverage for all same-multibody/world-link
fixed/spherical/one-DOF design-mode rebuilds and broken state, plus
one-DOF motor
re-engagement, plus focused
movable-movable same-multibody link-pair motor
projection, direct/private and current-pose movable-pair revolute plus
non-cardinal current-pose revolute/prismatic motor-axis coverage, public
same-multibody/world-anchored articulated revolute/prismatic
floating-endpoint plus selected off-origin-anchor facade non-cardinal
motor-axis coverage, selected same-multibody/world-link save/load
non-cardinal axis-basis persistence, selected one-DOF broken-state
save/load/reset non-cardinal axis-basis persistence, selected direct
same-multibody/world-link one-DOF break/skip/reset non-cardinal axis-basis
coverage plus direct private parent-endpoint one-DOF reset re-engagement,
direct private child-/parent-endpoint one-DOF command-update non-cardinal
axis-basis coverage, direct private child-/parent-endpoint finite-effort
non-cardinal axis-basis coverage,
current-pose
fixed/prismatic break/reset coverage, private current-pose spherical break/reset
coverage for linear-only rows, current-pose movable-pair revolute/prismatic
finite-limit coverage, public same-multibody/world-link one-DOF
non-cardinal off-origin finite-limit coverage, public same-multibody movable-pair
revolute/prismatic non-cardinal motor-axis and finite-limit coverage, public
same-multibody movable-pair one-DOF broken-state save/load/reset coverage, and
private generated current-pose movable-pair fixed all-axis, one-DOF, and
spherical linear-row broken-state save/load/reset coverage, and
revolute/prismatic dartpy break/reset stepping plus
C++ off-origin revolute/prismatic anchor break/reset coverage, off-origin anchor
projection,
same-multibody fixed break/reset, and
world-fixed break/reset; fixed facades now also have same-multibody link-link
and world-link save/load coverage that rebuilds the private AVBD all-axis rows,
including selected dartpy design-mode rebuild evidence, with movable-movable
fixed broken-state save/load/reset and dartpy stepping now covered,
while spherical facades now also have focused
same-multibody/world-link break/skip/reset coverage for their linear-only rows
while orientation remains free plus same-multibody link-link and
world-link save/load coverage that rebuilds the private AVBD linear rows,
including selected dartpy design-mode rebuild evidence; a
narrow `avbd_articulated_high_ratio_chain` py-demo,
`BM_AvbdArticulatedHighRatioChainStep` dashboard row, and tracked
[`avbd-articulated-high-ratio-chain-packet.json`](104-vertex-block-descent-solver/avbd-articulated-high-ratio-chain-packet.json)
now exercise a five-link variational chain with a 200:1 heavy tip, but broader
articulated entrypoints and default paper-scale scene coverage are not solved
yet. `PaperScaleHighRatioChainStaysFiniteAndResets` now adds a focused
50-link/50,000:1 finite/reset stability smoke through configured
`World::step()` solve-budget fields, and
`BM_AvbdPaperScaleHighRatioChainStep` adds a matching paper-scale CPU
dashboard row with tracked
[`avbd-paper-scale-high-ratio-chain-packet.json`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-chain-packet.json)
visual/benchmark evidence, without closing same-hardware paper-number or GPU
parity gates. `BM_AvbdPaperScaleHighRatioChainIterationSweep` now adds the
first dashboard-selected max-iteration sweep for that same fixture over
25/50/100/200 iterations, with finite replay counters, tracked
benchmark/stability-plot evidence in
[`avbd-paper-scale-high-ratio-iteration-sweep-packet.json`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-packet.json),
and a rendered
[`avbd-paper-scale-high-ratio-iteration-sweep-plot.svg`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-plot.svg);
it still needs same-hardware paper-number comparison and GPU parity before
counting as a completed parameter sweep.
Public free-rigid-body revolute/prismatic velocity actuators now also extract
to private AVBD angular/linear motor rows. Both paths have categorized
`avbd_rigid_revolute_motor` / `avbd_rigid_prismatic_motor` py-demos,
end-to-end dashboard benchmark rows for the public motor paths, and tracked
[`avbd-rigid-revolute-motor-packet.json`](104-vertex-block-descent-solver/avbd-rigid-revolute-motor-packet.json)
and
[`avbd-rigid-prismatic-motor-packet.json`](104-vertex-block-descent-solver/avbd-rigid-prismatic-motor-packet.json)
visual/benchmark evidence.
Solver-identity relabel (PLAN-091 WP-091.1): no `avbd-demo2d`/`avbd-demo3d`
benchmark or py-demo scene emplaces the internal AVBD rigid-contact opt-in
config (`comps::RigidAvbdContactConfig`), because AVBD contact is not
facade-selectable, so every rigid contact in the source-row scenes below ran
DART's default sequential-impulse contact path. The native-runner timing
ratios for contact scenes are whole-pipeline `World::step` comparisons, not
AVBD-contact-solver comparisons: the pure-contact rows (2D Dynamic Friction,
Static Friction, Pyramid, Cards, Stack, and Stack Ratio; 3D Ground, Dynamic
Friction, Static Friction, Pyramid, Stack, and Stack Ratio) timed no AVBD rows
at all; the joint-plus-contact rows (2D Fracture, Soft Body, Joint Grid, and
Net; 3D Soft Body, Bridge, and Breakable) timed AVBD
point-joint/motor/spring rows while their ordinary contacts ran sequential
impulse; and incidental link-link contacts in the chain rows (2D Rod, Rope,
Heavy Rope, and Hanging Rope; 3D Rope and Heavy Rope) also ran sequential
impulse. This relabel changes no committed packet bytes and neither closes nor
reopens any PLAN-104 completion gate; new AVBD evidence packets must
machine-record `resolved_solver_identity` at AVBD packet schema version 2,
enforced by `pixi run check-avbd-packets`.
The `avbd_empty_baseline` py-demo and `BM_AvbdEmptyWorldStep` row now provide a
first runnable baseline for the 2D/3D source-demo empty rows, with source
revision/default metadata and a `sceneEmpty` zero-count reference invariant.
The tracked
[`avbd-empty-baseline-packet.json`](104-vertex-block-descent-solver/avbd-empty-baseline-packet.json)
adds the matching headless visual-capture hashes and Google Benchmark row.
The `avbd_demo2d_ground` py-demo now ports the static `avbd-demo2d` Ground
source row, matching the reference revision, source scene index, one static
slab, one rigid body, one collision shape, no joints, and no dynamic bodies
with focused Python source-row coverage and a `BM_AvbdDemo2dGroundStep`
dashboard row. The tracked
[`avbd-demo2d-ground-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-ground-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
after skipping static-only contact queries and no-op rigid dynamics stages in
static-only worlds, later clean frame-cache graph execution, and the clean
no-work default step pipeline with a cheap scratch reset, the refreshed packet
records the DART public World Ground row at 16.3 ns median CPU time per step
versus 24.5 ns for the native static source runner on this host. The narrow CPU
reference-comparison gate is closed at about 1.51x faster than native, while
broad ground-contact, GPU, and paper-number gates remain open.
The `avbd_demo2d_motor` py-demo now ports the first one-DOF motor
`avbd-demo2d` source row, matching the reference Motor row's revision, source scene
index, timestep/gravity/iteration defaults, target speed, and effort bound with
focused Python source-row coverage and a `BM_AvbdDemo2dMotorStep` dashboard row;
the tracked
[`avbd-demo2d-motor-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-motor-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing.
The rigid contact stage skips contact queries for worlds with no collision
geometry in prepare/execute, but the refreshed packet still records the DART
public World row at 8.87 us median CPU time per step versus 1.435 us for the
native source runner on this host, so the CPU reference-comparison gate remains
open.
The `avbd_demo2d_hanging_rope` py-demo now ports the `avbd-demo2d` Hanging Rope
source row with source revision/scene metadata, 49 regular links, one 10 m
endpoint block, 49 linear-only public spherical point joints, focused Python
source-row coverage, and a `BM_AvbdDemo2dHangingRopeStep` dashboard row. The
tracked
[`avbd-demo2d-hanging-rope-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-hanging-rope-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Hanging Rope row at 0.392 ms median CPU time
per step versus 50.1 us for the native source runner on this host, so the CPU
reference-comparison gate remains open.
The `avbd_demo2d_fracture` py-demo now ports the `avbd-demo2d` Fracture source
row with source revision/scene metadata, 11 chain links, two dynamic supports,
15 falling blocks, 10 public breakable fixed joints, focused Python source-row
coverage, focused break/reset lifecycle coverage proving the source-row fixed
joints fracture, reset at a high break force, stay unbroken, and reduce their
anchor residuals again, and a `BM_AvbdDemo2dFractureStep` dashboard row. The
tracked
[`avbd-demo2d-fracture-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-fracture-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
after a refreshed same-source timing run, it records the DART public World
Fracture row at 51.2 us median CPU time per step versus 61.4 us for the native
source runner on this host, closing only this narrow CPU reference-comparison
gate. Follow-up contact-stage cleanup replaces the duplicate prepare-time
collision query with collision-shape-count constraint prewarm; execute-time
contact discovery remains authoritative. Follow-up source-row alignment now
suppresses live public rigid-body joint endpoint pairs in collision queries
using reusable query-cache scratch, matching the native solver's
constrained-pair broadphase rule while letting broken AVBD joints become
collidable again; the query-filter path no longer needs `Name` views, skips
empty ignored-pair lookups, and folds the contact-stage preflight into one
registry scan. Focused coverage verifies warmed filtered queries do not
allocate. The Fracture source-row metadata and packet writer now record the 10
joint-connected collision pairs.
The `avbd_demo2d_dynamic_friction` py-demo now ports the `avbd-demo2d` Dynamic
Friction source row with source revision/scene metadata, 11 sliding boxes, a
static ground, source friction coefficients from 5.0 down to 0.0, focused
Python source-row coverage, and a `BM_AvbdDemo2dDynamicFrictionStep` dashboard
row. The tracked
[`avbd-demo2d-dynamic-friction-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-dynamic-friction-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Dynamic Friction row at 5.73 us median CPU
time per step versus 10.49 us for the native source runner on this host,
closing that narrow source row while leaving broad contact-manifold friction
persistence, stacking/friction sweeps, GPU, and paper-number gates open.
The companion `BM_AvbdDemo2dFrictionCoefficientSweep` benchmark now sweeps the
same source-shaped scene over maximum Coulomb friction values 0, 0.5, 1, 2.5,
and 5. The tracked
[`avbd-friction-coefficient-sweep-packet.json`](104-vertex-block-descent-solver/avbd-friction-coefficient-sweep-packet.json)
validates the sweep rows, embeds same-source native timing for each coefficient,
embeds per-coefficient visual capture hashes, and records that after skipping
all-zero Coulomb contact-friction row blocks DART is faster for max friction
0.5, 1.0, 2.5, and 5.0 but still slower for the frictionless max friction 0
case on this host.
The rendered
[`avbd-friction-coefficient-sweep-plot.svg`](104-vertex-block-descent-solver/avbd-friction-coefficient-sweep-plot.svg)
plots DART and native source CPU step time versus maximum friction. This closes
only the source/reference timing-evidence and per-coefficient visual-capture
gaps for this friction-coefficient comparison; it is not a full CPU-win, GPU,
or paper-number claim.
The `avbd_demo2d_static_friction` py-demo now ports the `avbd-demo2d` Static
Friction source row with source revision/scene metadata, one rotated static
ground slab, 11 rotated dynamic boxes, uniform source friction 1.0, focused
Python source-row coverage, and a `BM_AvbdDemo2dStaticFrictionStep` dashboard
row. The tracked
[`avbd-demo2d-static-friction-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-static-friction-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Static Friction row at 5.44 us median CPU time
per step versus 14.56 us for the native source runner on this host, closing
that narrow source row while leaving broad contact-manifold friction
persistence, stacking/friction sweeps, GPU, and paper-number gates open.
The `avbd_demo2d_pyramid` py-demo now ports the `avbd-demo2d` Pyramid source
row with source revision/scene metadata, a static ground, 210 dynamic boxes in
the source pyramid layout, 211 collision shapes, focused Python source-row
coverage, and a `BM_AvbdDemo2dPyramidStep` dashboard row. The tracked
[`avbd-demo2d-pyramid-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-pyramid-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing; it
records the DART public World Pyramid row at 0.251 ms median CPU time per step
versus 2.47 ms for the 10,000-step native source runner on this host, closing
that narrow source row while leaving broad rigid stacking, GPU, and paper-number
gates open.
The `avbd_demo2d_stack` py-demo now ports the `avbd-demo2d` Stack source row
with source revision/scene metadata, 20 vertical dynamic boxes over static
ground, 21 collision shapes, focused Python source-row coverage, and a
`BM_AvbdDemo2dStackStep` dashboard row. The tracked
[`avbd-demo2d-stack-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-stack-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Stack row at 10.37 us median CPU time per step
versus 22.46 us for the native source runner on this host, closing that narrow
source row while leaving broad rigid stacking, GPU, and paper-number gates
open.
The `avbd_demo2d_stack_ratio` py-demo now ports the `avbd-demo2d` Stack Ratio
source row with source revision/scene metadata, six geometric-size dynamic
boxes over static ground, 7 collision shapes, focused Python source-row
coverage, and a `BM_AvbdDemo2dStackRatioStep` dashboard row. The tracked
[`avbd-demo2d-stack-ratio-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-stack-ratio-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing; it
records the DART public World Stack Ratio row at 3.62 us median CPU time per
step versus 8.03 us for the native source runner on this host, closing that
narrow source row while leaving broad rigid stacking, high-ratio stability, GPU,
and paper-number gates open.
The `avbd_demo2d_rod` py-demo now ports the `avbd-demo2d` Rod source row with
source revision/scene metadata, 20 rigid links, one static anchor link, 19
public all-axis fixed joints, 20 collision shapes, focused Python source-row
coverage, and a `BM_AvbdDemo2dRodStep` dashboard row. The tracked
[`avbd-demo2d-rod-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-rod-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Rod row at 0.253 ms median CPU time per step
versus 26.4 us for the native source runner on this host, keeping that CPU-win
gate open while leaving broad rod/high-ratio, GPU, and paper-number gates
open.
The `avbd_demo2d_soft_body` py-demo now ports the `avbd-demo2d` Soft Body
source row with source revision/scene metadata, one static ground slab, two
15x5 dynamic rigid-box lattices, 260 finite-stiffness all-axis fixed joints,
224 diagonal ignored collision pairs, focused Python source-row coverage, and
a `BM_AvbdDemo2dSoftBodyStep` dashboard row. The tracked
[`avbd-demo2d-soft-body-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-soft-body-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Soft Body row at 3.58 ms median CPU time per
step versus 0.568 ms for the native source runner on this host. Later friction
tangent-pair world-anchor reuse improved the local contact-heavy solve path but
has not refreshed this packet, keeping that CPU-win gate open while leaving the
GPU and paper-number gates open.
The `avbd_demo2d_joint_grid` py-demo now ports the `avbd-demo2d` Joint Grid
source row with source revision/scene metadata, 625 rigid boxes, two static
top-corner anchors, 1200 public all-axis fixed joints, 625 collision shapes,
focused Python source-row coverage, and a `BM_AvbdDemo2dJointGridStep`
dashboard row. The tracked
[`avbd-demo2d-joint-grid-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-joint-grid-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Joint Grid row at 15.1 ms median CPU time per
step versus 7.33 ms for the native source runner on this host after reusing
per-body row-index scratch, caching snapshot body indices, reusing row-assembly
world-anchor computations through force/Hessian/Jacobian stamping, and caching
all-axis angular-row orientation errors,
keeping that CPU-win gate open while configuring the source diagonal
ignore-collision filter through the public per-pair collision API; broad
joint-grid/high-ratio, GPU, and paper-number gates remain open.
The `avbd_demo2d_cards` py-demo now ports the `avbd-demo2d` Cards source row
with source revision/scene metadata, one static source-shaped ground slab, 40
thin dynamic cards across five levels, 41 collision shapes, focused Python
source-row coverage, and a `BM_AvbdDemo2dCardsStep` dashboard row. The tracked
[`avbd-demo2d-cards-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-cards-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Cards row at 0.517 ms median CPU time per step
versus 96.2 us for the native source runner on this host. Later friction
tangent-pair world-anchor reuse smoke reported about 0.550 ms under high host
load, and later row-state update reuse smoke reported about 0.472 ms under
load average around 7.5. Shared-anchor reuse smoke later reported about
0.507 ms under load average around 4.3. None of these refreshed this packet, so
the CPU-win gate remains open while
leaving broad card-tower/contact-stability, GPU, and paper-number gates open.
The `avbd_demo2d_rope` py-demo now ports the `avbd-demo2d` Rope source row
with source revision/scene metadata, 20 rigid links, 19 linear-only public
spherical point joints, 20 collision shapes, focused Python source-row
coverage, and a `BM_AvbdDemo2dRopeStep` dashboard row. The tracked
[`avbd-demo2d-rope-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-rope-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Rope row at 0.133 ms median CPU time per step
versus 25.6 us for the native source runner on this host, keeping that CPU-win
gate open while leaving broad rope/high-ratio, GPU, and paper-number gates
open.
The `avbd_demo2d_heavy_rope` py-demo now ports the `avbd-demo2d` Heavy Rope
source row with source revision/scene metadata, 19 regular links, a 30 m
endpoint block, 19 linear-only public spherical point joints, 20 collision
shapes, focused Python source-row coverage, and a
`BM_AvbdDemo2dHeavyRopeStep` dashboard row. The tracked
[`avbd-demo2d-heavy-rope-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-heavy-rope-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
after reusing per-body row-index scratch, caching snapshot body indices,
avoiding redundant row-assembly world-anchor transforms, and caching grouped
angular-row orientation errors, with the follow-up distance-spring Hessian path
also reusing precomputed world anchors for point Jacobians and capped
finite-stiffness row-state updates now skipping redundant constraint
recomputation, it records the DART
public World Heavy Rope row at 0.149 ms median CPU time per step versus 25.4 us
for the native source
runner on this host, keeping that CPU-win gate open while leaving broad
high-ratio, GPU, and paper-number gates open.
The `avbd_demo2d_net` py-demo now ports the `avbd-demo2d` Net source row with
source revision/scene metadata, one static ground slab, 40 endpoint-pinned net
links, 50 falling rigid blocks, 39 linear-only public spherical point joints,
91 collision shapes, focused Python source-row coverage, and a
`BM_AvbdDemo2dNetStep` dashboard row. The tracked
[`avbd-demo2d-net-packet.json`](104-vertex-block-descent-solver/avbd-demo2d-net-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
after caching snapshot body indices, reusing row-assembly world-anchor
computations through force/Hessian/Jacobian stamping, and caching grouped
angular-row orientation errors, with the follow-up friction tangent-pair path
also reusing precomputed world anchors and the row-state update path reusing
precomputed tangent constraint values and shared tangent-row anchors, plus capped
finite-stiffness row-state updates now skipping redundant constraint
recomputation, and with local-anchor point-joint extraction now using a stable
full-rank linear basis for spherical point joints, it records
the DART public World Net
row at 0.286 ms median CPU time per step versus 0.232 ms
for the native source runner on this host, keeping that CPU-win gate open while
leaving broader soft/rigid
net coverage, GPU, and paper-number gates open.
The `avbd_demo3d_ground` py-demo now ports the `avbd-demo3d` Ground source row
with source revision/scene metadata, a static floor, falling rigid box, rigid
ground-contact coverage, and a `BM_AvbdDemo3dGroundStep` dashboard row. The
tracked
[`avbd-demo3d-ground-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-ground-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Ground row at 5.49 us median CPU time per
step versus 6.10 us for the native source runner on this host, closing that
narrow source row without closing broad stacking/friction, GPU, or paper-number
gates.
The `avbd_demo3d_dynamic_friction` py-demo now ports the `avbd-demo3d` Dynamic
Friction source row with source revision/scene metadata, 11 sliding rigid
boxes, a static floor, focused Python source-row coverage, and a
`BM_AvbdDemo3dDynamicFrictionStep` dashboard row. The tracked
[`avbd-demo3d-dynamic-friction-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-dynamic-friction-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Dynamic Friction row at 36.85 us median CPU
time per step versus 51.98 us for the native source runner on this host,
closing that narrow source row without closing broad contact-manifold friction
persistence, stacking/friction sweeps, GPU, or paper-number gates.
The `avbd_demo3d_static_friction` py-demo now ports the `avbd-demo3d` Static
Friction source row with source revision/scene metadata, a static floor,
inclined static ramp, 11 sliding boxes, focused Python source-row coverage, and
a `BM_AvbdDemo3dStaticFrictionStep` dashboard row. The tracked
[`avbd-demo3d-static-friction-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-static-friction-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Static Friction row at 48.27 us median CPU
time per step versus 51.97 us for the native source runner on this host,
closing that narrow CPU reference-comparison gate.
The `avbd_demo3d_pyramid` py-demo now ports the `avbd-demo3d` Pyramid source
row with source revision/scene metadata, a static ground, 136 dynamic boxes in
the triangular pile layout, focused Python source-row coverage, and a
`BM_AvbdDemo3dPyramidStep` dashboard row. The tracked
[`avbd-demo3d-pyramid-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-pyramid-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Pyramid row at 0.991 ms median CPU time per
step versus 2.80 ms for the native source runner on this host, closing that
narrow source row without closing broad stacking, GPU, or paper-number gates.
The `avbd_demo3d_rope` py-demo now ports the `avbd-demo3d` Rope source row with
source revision/scene metadata, 20 rigid links, 19 anchored linear-only public
spherical point joints, focused Python source-row coverage, and a
`BM_AvbdDemo3dRopeStep` dashboard row. The tracked
[`avbd-demo3d-rope-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-rope-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
after reusing per-body row-index scratch and caching snapshot body indices, it records
the DART public World Rope row at 0.134 ms median CPU time per step versus
35.9 us for the native source runner on this host, so the CPU
reference-comparison gate remains open.
The `avbd_demo3d_heavy_rope` py-demo now ports the `avbd-demo3d` Heavy Rope
source row with source revision/scene metadata, 19 regular links, a 5 m
endpoint block, 19 anchored linear-only public spherical point joints, focused
Python source-row coverage, and a `BM_AvbdDemo3dHeavyRopeStep` dashboard row.
The tracked
[`avbd-demo3d-heavy-rope-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-heavy-rope-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
after reusing per-body row-index scratch and caching snapshot body indices, it records
the DART public World Heavy Rope row at 0.128 ms median CPU time per step
versus 33.4 us for the native source runner on this host, so the CPU
reference-comparison gate remains open.
The `avbd_demo3d_spring` py-demo now ports the `avbd-demo3d` Spring source row
with source revision/scene metadata, a static ground, static anchor, one dynamic
block, one public free-rigid radial distance spring, focused Python source-row
coverage, and a `BM_AvbdDemo3dSpringStep` dashboard row. The tracked
[`avbd-demo3d-spring-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-spring-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Spring row at 3.58 us median CPU time per step
versus 1.83 us for the native source runner on this host, so the CPU
reference-comparison gate remains open.
The `avbd_demo3d_spring_ratio` py-demo now ports the `avbd-demo3d` Spring Ratio
source row with source revision/scene metadata, eight rigid links, static
endpoints, seven alternating-stiffness public free-rigid radial distance
springs, focused Python source-row coverage, and a
`BM_AvbdDemo3dSpringRatioStep` dashboard row. The tracked
[`avbd-demo3d-spring-ratio-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-spring-ratio-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Spring Ratio row at 29.1 us median CPU time
per step versus 7.85 us for the native source runner on this host, so the CPU
reference-comparison gate remains open.
The `avbd_demo3d_stack` py-demo now ports the `avbd-demo3d` Stack source row
with source revision/scene metadata, 10 vertical dynamic boxes over static
ground, focused Python source-row coverage, and a `BM_AvbdDemo3dStackStep`
dashboard row. The tracked
[`avbd-demo3d-stack-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-stack-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Stack row at 42.1 us median CPU time per step
versus 75.7 us for the native source runner on this host, closing that narrow
source row without closing broad stacking, high-ratio stability, GPU, or
paper-number gates.
The `avbd_demo3d_stack_ratio` py-demo now ports the `avbd-demo3d` Stack Ratio
source row with source revision/scene metadata, four geometric-size dynamic
boxes over static ground, focused Python source-row coverage, and a
`BM_AvbdDemo3dStackRatioStep` dashboard row. The tracked
[`avbd-demo3d-stack-ratio-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-stack-ratio-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Stack Ratio row at 18.1 us median CPU time per
step versus 42.0 us for the native source runner on this host, closing that
narrow source row without closing broad size-ratio stability, GPU, or
paper-number gates.
The `avbd_demo3d_soft_body` py-demo now ports the `avbd-demo3d` Soft Body
source row with source revision/scene metadata, three 4x4x4 dynamic rigid-box
lattices, 432 finite-stiffness all-axis fixed joints (`Klin=1000`,
`Kang=250`), 648 diagonal ignored collision pairs, focused Python source-row
coverage, and a `BM_AvbdDemo3dSoftBodyStep` dashboard row. The tracked
[`avbd-demo3d-soft-body-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-soft-body-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Soft Body row at 3.93 ms median CPU time per
step versus 4.76 ms for the native source runner on this host, closing that
narrow CPU reference-comparison gate while leaving GPU, soft/rigid coupling,
and paper-number gates open.
Follow-up row-inventory work now warm-starts unchanged same-order AVBD scalar
row descriptor lists in place instead of rebuilding a previous-row map every
step, and rigid World contact snapshots now assign contact/joint/spring row
ordinals through reserved endpoint-pair hash counters instead of per-step tree
maps. Append paths now seed the body-index cache when a fallback scan finds a
preseeded snapshot entity, and the rigid row driver also skips per-body
row-index scratch setup for row families absent from the current solve, reuses
unchanged row-index layouts across frames, and routes single-family
point-pair/angular row solves directly instead of copying them into combined
work vectors, while one-new-row point-joint/distance-spring appends skip
endpoint row-counter hash-map setup. Articulated AVBD extraction now also uses
a per-multibody link-index cache in the point-joint extractor,
so same-multibody/world-link AVBD private rows do not rescan structure
membership for every point-joint config. Rigid AVBD point-joint and
distance-spring extraction now also share a projectable-body metadata lookup,
avoiding a second transform lookup after endpoint classification in the
pair-constraint hot path and reusing the already-checked projectable transform,
mass, and static tag when contact snapshots materialize body state. The same
metadata path also skips static-static rigid point-joint and distance-spring
pairs before input or row construction. The contact-stage AVBD path now also
reuses a scratch contact snapshot through an in-place builder, so
contact-enabled rows do not recreate the snapshot containers every step, skips
point-joint/distance-spring extraction when no AVBD pair-constraint configs
exist, and extracts/appends point-joint and distance-spring families
independently when only one family exists. The rigid snapshot solve now also
clears absent row-family inventories directly instead of calling empty
contact/joint/motor/spring builders, and small
point-joint/motor/distance-spring row builders now use stack
descriptor/active-row storage for up to 16 candidate rows instead of allocating
temporary vectors, and capped rigid finite-stiffness point-pair, angular-pair,
and distance-spring rows now skip row-state constraint recomputation once their
effective stiffness is already at the material cap. The split rigid-body velocity stage now also assembles force
batches only for advanceable rigid bodies. Local profile smoke showed the Joint
Grid rigid contact stage moving from about 14.1 ms to about 13.3 ms, and current
benchmark smoke reports Motor median at about 8.43 us, Spring median at about
3.98 us, and Spring Ratio median at about 36.16 us. Refreshed same-command
packets now record about 3.82 us 2D Spring, 31.8 us 2D Spring Ratio, 3.58 us 3D
Spring, and 29.1 us 3D Spring Ratio, but all four rows remain slower than the
native runners. A broader capped finite-row diagnostic over Rod, Joint Grid,
Soft Body, and Spring rows ran under load average around 11.7 and is also
non-packet overhead smoke.
The `avbd_demo3d_bridge` py-demo now ports the `avbd-demo3d` Bridge source row
with source revision/scene metadata, 40 planks, 50 load boxes, 78 paired
linear-only public spherical point joints, focused Python source-row coverage,
and a `BM_AvbdDemo3dBridgeStep` dashboard row. The tracked
[`avbd-demo3d-bridge-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-bridge-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
after reusing per-body row-index scratch and caching snapshot body indices, it records
the DART public World Bridge row at 0.746 ms median CPU time per step versus
1.20 ms for the native source runner on this host, closing that narrow source
row while leaving broad coupled-constraint, GPU, and paper-number gates open.
The `avbd_demo3d_breakable` py-demo now ports the `avbd-demo3d` Breakable
source row with source revision/scene metadata, 19 rigid bodies, 10 breakable
fixed joints, 19 collision shapes, focused Python source-row coverage, focused
break/reset lifecycle coverage proving the source-row fixed joints fracture,
reset at a high break force, stay unbroken, and reduce their anchor residuals
again, and a `BM_AvbdDemo3dBreakableStep` dashboard row. The tracked
[`avbd-demo3d-breakable-packet.json`](104-vertex-block-descent-solver/avbd-demo3d-breakable-packet.json)
adds headless visual capture, DART benchmark JSON, and native source timing;
it records the DART public World Breakable row at 79.5 us median CPU time per
step versus 112.7 us for the native source runner on this host, so the narrow
CPU reference-comparison gate is closed for that source row.
Public articulated revolute velocity motors now also have a categorized
`avbd_articulated_revolute_motor` py-demo covering command updates through the
variational bridge plus tracked
[`avbd-articulated-revolute-motor-packet.json`](104-vertex-block-descent-solver/avbd-articulated-revolute-motor-packet.json)
visual/benchmark evidence, and public articulated prismatic velocity motors
have the matching `avbd_articulated_prismatic_motor` py-demo plus tracked
[`avbd-articulated-prismatic-motor-packet.json`](104-vertex-block-descent-solver/avbd-articulated-prismatic-motor-packet.json)
visual/benchmark evidence. Public articulated motor break/reset is now visible
through `avbd_articulated_motor_breakable_joint` and tracked
[`avbd-articulated-breakable-motor-packet.json`](104-vertex-block-descent-solver/avbd-articulated-breakable-motor-packet.json)
evidence for a same-multibody revolute motor, with focused reset coverage that
reverses the target velocity after re-engagement and verifies weak re-arm breaks
again,
`avbd_articulated_prismatic_pair_motor_breakable_joint` for a same-multibody
prismatic motor with tracked
[`avbd-articulated-prismatic-pair-breakable-motor-packet.json`](104-vertex-block-descent-solver/avbd-articulated-prismatic-pair-breakable-motor-packet.json)
evidence and post-reset reversed-command plus weak re-arm coverage, and
`avbd_articulated_prismatic_motor_breakable_joint` for a world-anchored
prismatic motor with tracked
[`avbd-articulated-world-prismatic-breakable-motor-packet.json`](104-vertex-block-descent-solver/avbd-articulated-world-prismatic-breakable-motor-packet.json)
evidence and post-reset reversed-command plus weak re-arm coverage, plus
`avbd_articulated_world_revolute_motor_breakable_joint` for a world-anchored
revolute motor with tracked
[`avbd-articulated-world-revolute-breakable-motor-packet.json`](104-vertex-block-descent-solver/avbd-articulated-world-revolute-breakable-motor-packet.json)
evidence and post-reset reversed-command plus weak re-arm coverage. The bounded AVBD World
dashboard slice now also tracks those public articulated motor paths through
`BM_AvbdArticulatedRevoluteMotorStep` and
`BM_AvbdArticulatedPrismaticMotorStep`, tracks the active break-force armed
articulated revolute motor path through `BM_AvbdArticulatedBreakableMotorStep`,
tracks the active break-force armed articulated prismatic motor path through
`BM_AvbdArticulatedPrismaticBreakableMotorStep`, tracks the active
world-anchored articulated prismatic motor path through
`BM_AvbdArticulatedWorldPrismaticBreakableMotorStep`, tracks the active
world-anchored articulated revolute motor path through
`BM_AvbdArticulatedWorldRevoluteBreakableMotorStep`, and
[`avbd-breakable-motor-scale-packet.json`](104-vertex-block-descent-solver/avbd-breakable-motor-scale-packet.json)
now validates those four public articulated breakable motor benchmark rows over
1, 8, and 32 motors with finite timing rows and exact `motors` plus
`breakable_motors` counters. This remains benchmark-only scale evidence rather
than a broad motor lifecycle, source-demo, GPU, or paper-number claim. The same
dashboard slice also
tracks the public free-rigid and
articulated breakable fixed point-joint paths through
`BM_AvbdRigidBreakableJointStep` and
`BM_AvbdArticulatedBreakableJointStep`, now backed by tracked
[`avbd-rigid-breakable-joint-packet.json`](104-vertex-block-descent-solver/avbd-rigid-breakable-joint-packet.json)
and
[`avbd-articulated-breakable-joint-packet.json`](104-vertex-block-descent-solver/avbd-articulated-breakable-joint-packet.json)
evidence.
The same dashboard slice also tracks public spherical break/reset rows through
`BM_AvbdRigidSphericalBreakableJointStep`,
`BM_AvbdArticulatedWorldSphericalBreakableJointStep`, and
`BM_AvbdArticulatedSphericalPairBreakableJointStep`, each backed by tracked
packet evidence. The companion
[`avbd-breakable-joint-scale-packet.json`](104-vertex-block-descent-solver/avbd-breakable-joint-scale-packet.json)
now validates those five public fixed/spherical breakable point-joint
benchmark rows over 1, 8, and 32 joints with finite timing rows and exact
`breakable_joints` counters; this remains benchmark-only scale evidence rather
than a broad fracture-corpus, breakable-wall, GPU, or paper-number claim.
Public free-rigid-body AVBD point joints now also expose a narrow break-force
and broken-state lifecycle through C++/dartpy, with solved-row fracture marking,
later extraction skip behavior, and a categorized `avbd_rigid_breakable_joint`
py-demo covering reset/re-engagement of the captured fixed pose plus tracked
visual/benchmark packet evidence, plus an
`avbd_rigid_spherical_breakable_joint` py-demo covering free-rigid spherical
break/reset with orientation left free and tracked packet evidence. The
free-rigid fixed/spherical demo regressions also verify weak re-arm breaks
again after high-force reset. Public
articulated fixed point-joints
now also have a categorized
`avbd_articulated_breakable_joint` py-demo covering the world-link
break/reset lifecycle through the variational bridge with tracked packet
evidence, plus an
`avbd_articulated_fixed_pair_breakable_joint` py-demo covering the
same-multibody fixed break/reset lifecycle with tracked packet evidence; both
fixed articulated demo regressions verify weak re-arm breaks again after
reset.
Focused C++ coverage now also
carries the public spherical point-joint break/skip lifecycle for
same-multibody linear-only rows and world-link reset re-engagement without
adding orientation rows, with matching dartpy stepping coverage, tracked
visual/benchmark packet evidence, and a
categorized `avbd_articulated_spherical_breakable_joint` py-demo for the
world-link spherical break/reset path plus an
`avbd_articulated_spherical_pair_breakable_joint` py-demo for the
same-multibody spherical break/reset path; both spherical demo regressions
verify weak re-arm breaks again after reset. A narrow
`avbd_articulated_high_ratio_chain` py-demo and
`BM_AvbdArticulatedHighRatioChainStep` dashboard row now cover a five-link
articulated variational-chain smoke with a 200:1 heavy tip, and the tracked
[`avbd-articulated-high-ratio-chain-packet.json`](104-vertex-block-descent-solver/avbd-articulated-high-ratio-chain-packet.json)
records focused visual/benchmark evidence without claiming the full
50-body/50,000:1 paper pendulum. The
`PaperScaleHighRatioChainStaysFiniteAndResets` C++ regression separately covers
a 50-link/50,000:1 finite/reset smoke through configured `World::step()`
solve-budget fields, and `BM_AvbdPaperScaleHighRatioChainStep` adds the
matching paper-scale CPU dashboard row with
[`avbd-paper-scale-high-ratio-chain-packet.json`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-chain-packet.json)
visual/benchmark evidence but no same-hardware paper-number claim.
`BM_AvbdPaperScaleHighRatioChainIterationSweep` is now selected by the
dashboard runner over 25/50/100/200 max-iteration budgets for the same
paper-scale fixture, with finite replay counters, tracked benchmark/stability
evidence in
[`avbd-paper-scale-high-ratio-iteration-sweep-packet.json`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-packet.json),
and rendered plot evidence in
[`avbd-paper-scale-high-ratio-iteration-sweep-plot.svg`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-plot.svg);
the same-hardware paper-number comparison and GPU evidence remain open.
The private
endpoint classifier now separates free rigid-body endpoints from multibody
links, with
`BM_AvbdRigidEndpointClassification` and the
focused C++ tests covering the first hard fixed and masked revolute/prismatic
link endpoint bridges into the variational articulated solve path. Breakable
hard multibody-link point-joint configs now also mark their source joint broken
from the projection load in that bridge, including private fixed-row reset and
prismatic velocity-motor row indexing/reset re-engagement regressions. The
`variational_endpoint_loop_closure` py-demo is a related public loop-closure
preview, not coverage for the private AVBD config extractor. Public one-DOF
articulated `Velocity` actuators now also project through the variational
articulated path as coordinate motor targets for revolute/prismatic topology
joints. Private hard revolute and prismatic AVBD point-joint velocity actuators
on articulated endpoints now also project one free-axis angular/linear motor row
in that same variational bridge, with focused child-/parent-endpoint
revolute/prismatic command-update coverage including non-cardinal
child-/parent-endpoint axis bases, tiny positive effort-limit coverage, private
fixed-row reset, and break/reset re-engagement for breakable revolute and
prismatic motor rows; the direct private tiny-limit coverage now also spans
child-/parent-endpoint non-cardinal free-axis bases.
Finite-stiffness private AVBD fixed
point-joint configs on articulated endpoints now also contribute compliant
variational forces through persistent per-axis finite-stiffness row state that
warm-starts and ramps toward the configured cap across steps.
Free-rigid AVBD point-joint rows now also support finite linear and angular
material stiffness through the public point-joint facade and dartpy properties;
those rows use raw finite residuals and ramp row stiffness up to the configured
cap, while infinity preserves the previous hard-constraint behavior.
Explicit fallback coverage keeps unsupported mixed spring-plus-tet,
mass-spring self-contact without the self-contact AVBD flag,
finite-stiffness-only friction scenes, Chebyshev, Rayleigh-damped, parallel,
and unsupported-row requests on the existing VBD path without partial AVBD row
counters. Those slices are still foundation work; hard-contact/friction
completeness beyond current static half-space and lagged self-contact rows,
broader contact-stage dynamic/rigid AVBD activation, articulated multibody
state/solve beyond the new private endpoint classifier, broad fracture-corpus
support, rigid/soft coupling, GPU parity, complete paper/source-demo py-demos,
visual evidence, and broad paper/reference benchmark packets remain open.

## Relationship To PLAN-081

PLAN-081 (IPC) and PLAN-104 (VBD) both target robust deformable dynamics on the
same experimental deformable ECS components and the same variational
implicit-Euler objective. They differ in the inner solver: PLAN-081 pursues an
implicit-barrier projected-Newton-style method; PLAN-104 pursues per-vertex
block coordinate descent. VBD reuses the PLAN-081 `deformable_contact` kernels
for contact/friction rather than duplicating them. OGC starts as a PLAN-104
codimensional-contact sidecar because the paper's fast path is VBD plus local
displacement bounds, but it must compare against PLAN-081 IPC-style barriers
before any supersession claim. The public deformable stage stays
algorithm-neutral; which inner solver or contact model runs is an
internal/explicit-opt-in decision, not a leaked solver registry.

## Workstreams

1. **Per-vertex block kernels** — Internal `detail/deformable_vbd` kernels that,
   for a vertex and its incident springs/tets/contacts, accumulate the local
   force `f_i = -dG/dx_i` and a symmetric positive-definite 3x3 Hessian `H_i`,
   then take one block Newton step `x_i += H_i^{-1} f_i`. Mass-spring first,
   FEM later. The inertia term and PD-projected element blocks keep `H_i` SPD.
2. **Graph coloring** — Greedy vertex coloring over the element-induced
   vertex–vertex adjacency so same-color vertices share no element and can be
   updated in parallel. Deterministic, with conflict-free verification.
3. **Block-descent driver** — A single-body sweep that, per VBD iteration,
   visits colors sequentially and updates each color's vertices, reducing `G`.
   Validated against the existing gradient-descent solver on mass-spring scenes.
4. **FEM hyperelasticity** — Stable Neo-Hookean tetrahedral energy with Lame
   parameters from `DeformableMaterial`, contributing per-vertex PD Hessian
   blocks to the block solve.
5. **Acceleration + damping** — Adaptive inertial/previous-step initialization,
   optional Chebyshev semi-iterative acceleration, and the VBD element damping
   model.
6. **Solver wiring** — Make VBD a DART-owned, selectable inner solver for the
   deformable stage with integration tests; keep the public surface neutral.
7. **Contact + friction** — Vertex-based contact and friction reusing the
   `deformable_contact` distance/barrier/CCD/tangent kernels.
8. **CPU performance** — SoA layout and multithreaded color sweeps; benchmarks
   versus the reference CPU numbers.
9. **GPU performance** — A CUDA VBD backend behind the experimental compute
   boundary; benchmarks versus the reference/paper GPU numbers.
10. **Corpus + visual evidence** — DART-native examples, tests, benchmarks,
    profiling JSON, and headless Filament captures reproducing the paper scenes.
11. **Offset Geometric Contact evaluation** — Follow
    [`ogc-gap-audit.md`](104-vertex-block-descent-solver/ogc-gap-audit.md):
    source/code audit, internal vertex-facet and edge-edge OGC contact
    construction, conservative per-vertex bounds and truncation, VBD
    force/Hessian integration, reduced paper-scene corpus, CPU benchmark packets,
    then private GPU evidence. Keep OGC names internal until those gates pass.

## AVBD Workstreams

1. **Scalar row foundation** — Internal `detail/deformable_vbd` utilities for
   augmented-Lagrangian scalar rows: `C(x) - alpha C(x_t)`, warm-started
   `lambda`/`k`, bounded hard-row force magnitudes, dual write-back, and finite
   stiffness ramping.
2. **Row inventory + storage** — Hard equality, bounded inequality, finite
   stiffness, contact normal, friction tangent, joint, motor, fracture, and
   attachment rows with persistent IDs for warm starting.
3. **CPU deformable AVBD** — Extend the existing VBD deformable path so hard
   attachments, finite-stiffness ramped springs/tets, self-contact, and static
   obstacle contact use AVBD rows rather than pure penalty terms.
4. **CPU rigid/articulated AVBD** — Add 6-DOF rigid blocks, tangent angular
   updates, rigid contact manifolds, ball/revolute/limited joints, motors,
   breakage, and high mass-ratio articulated chains.
5. **Unified soft/rigid coupling** — Solve cloth/deformables, rigid bodies,
   articulated chains, collision constraints, joints, and attachments in one
   AVBD loop.
6. **GPU parity** — Port every row family, candidate-generation path, color
   update, dual/stiffness update, and benchmark scene through the private CUDA
   compute boundary.
7. **Paper/demo corpus** — Reproduce the 2D and 3D online demo scenes, all paper
   figures, the parameter sweeps, and the video/headline scenes in tests,
   benchmark JSON, `py-demos`, and visual evidence.
8. **Performance leadership** — Optimize CPU and GPU until DART beats the
   reference demo repositories and the published paper numbers on every claimed
   case, with hardware and command packets recorded.

## AVBD Current Next Gaps

The free-rigid fixed-joint/contact, free-rigid/articulated one-DOF motor,
breakable fixed point-joint, prismatic/revolute/spherical facade,
articulated breakable-joint, and high-ratio articulated-chain slices are
user-visible but intentionally small.
They do not cover the AVBD source-demo or paper corpus, and the dashboard rows
are narrow CPU public-World evidence only. The next bounded
implementation work should prefer one of these gaps, in order:

1. **Articulated multibody AVBD extraction** — the private extractor now
   classifies free rigid-body endpoints separately from multibody links, and
   explicitly hard fixed, masked revolute/prismatic, and breakable
   multibody-link point-joint configs can bridge into the variational
   articulated solve path without entering the free-rigid 6-DOF snapshot
   writeback. Public one-DOF velocity actuators now project through the
   variational articulated path, and private hard revolute/prismatic AVBD
   point-joint velocity actuators now project one free-axis motor row through
   that same bridge with child-/parent-endpoint revolute/prismatic
   command-update coverage including non-cardinal child-/parent-endpoint axis
   bases,
   tiny positive effort-limit coverage including non-cardinal
   child-/parent-endpoint axis bases, private fixed-row reset, and
   revolute/prismatic break/reset re-engagement for world-link polarity.
   Finite-stiffness
   private AVBD fixed point-joint
   configs now also contribute compliant variational forces through persistent
   stiffness-ramped rows, and non-topology multibody-link
   fixed/revolute/prismatic point-joint entities can now generate hard private
   configs from the simulation-entry current pose. Public same-multibody and
   world-link articulated facades now cover fixed, spherical linear-only
   pinned-anchor rows including explicit link-link/world-link anchors, bounded
   revolute/prismatic velocity,
   same-multibody/world-anchored command updates, and narrow break/reset
   behavior including public one-DOF motor break/skip and re-engagement on
   same-multibody and world-link endpoints, with focused dartpy stepping
   coverage for same-multibody/world-link revolute and prismatic explicit-anchor
   break/skip and reset/re-engagement cases including non-cardinal
   same-multibody pair and world-link revolute/prismatic reset regressions,
   plus movable-movable same-multibody motor projection and revolute/prismatic
   dartpy break/reset stepping plus C++ off-origin revolute/prismatic anchor break/reset
   coverage, explicit local/world
   anchor projection, fixed break/reset, movable-movable fixed broken-state
   save/load/reset, same-multibody link-link plus
   world-link fixed/spherical save/load rebuilding coverage for the private
   all-axis and linear-only rows, including selected dartpy design-mode rebuild
   evidence, and same-multibody/world-link revolute/prismatic motor
   save/load rebuilding coverage for the private hard rows and free-axis motor
   row with selected non-cardinal axis-basis persistence, plus
   same-multibody/world-link fixed/spherical/revolute/prismatic
   broken-state save/load/reset persistence including explicit-anchor,
   movable-movable fixed, selected non-cardinal one-DOF motor rows, and
   selected direct break/skip/reset non-cardinal basis checks plus
   focused C++ binary round-trips preserving one-DOF command/effort-limit motor
   state, plus public
   free-rigid/articulated AVBD
   point-joint stiffness facade binary persistence and direct C++/dartpy
   validation of articulated stiffness defaults, finite setters, and invalid
   setter rejection plus C++/dartpy endpoint-ownership rejection coverage,
   plus a per-multibody link-index cache for articulated AVBD point-joint
   extraction and a shared projectable-body metadata lookup for rigid
   point-joint/distance-spring extraction and contact-snapshot body insertion,
   including static-static rigid pair-constraint row pruning,
   with
   `avbd_articulated_revolute_motor` exposing the public articulated revolute
   command-update path, `avbd_articulated_prismatic_motor` exposing the public
   articulated prismatic command-update path as a narrow py-demo,
   `avbd_articulated_motor_breakable_joint` exposing the same-multibody public
   articulated revolute motor break/reset path as a narrow py-demo plus packet,
   and
   `avbd_articulated_breakable_joint` exposing the public articulated fixed
   point-joint world-link break/reset path as a narrow py-demo plus packet,
   plus
   `avbd_articulated_fixed_pair_breakable_joint` exposing the same-multibody
   fixed break/reset path as a narrow py-demo plus packet, plus
   `avbd_articulated_spherical_breakable_joint` exposing the public
   articulated spherical world-link break/reset path and
   `avbd_articulated_spherical_pair_breakable_joint` exposing the public
   articulated spherical same-multibody break/reset path as narrow py-demos
   plus packets,
   plus `avbd_articulated_high_ratio_chain` and
   `BM_AvbdArticulatedHighRatioChainStep` exposing a narrow five-link 200:1
   high mass-ratio variational-chain smoke scene and dashboard row, with
   [`avbd-articulated-high-ratio-chain-packet.json`](104-vertex-block-descent-solver/avbd-articulated-high-ratio-chain-packet.json)
   recording focused visual/benchmark evidence. The high-ratio scene is only
   paper-gap evidence; the companion `PaperScaleHighRatioChainStaysFiniteAndResets`
   regression covers a 50-link/50,000:1 finite/reset smoke through configured
   `World::step()` solve-budget fields, and
   `BM_AvbdPaperScaleHighRatioChainStep` exposes the matching paper-scale CPU
   dashboard row with a visual/benchmark packet, but this is still not a
   same-hardware comparison. The companion
   `BM_AvbdPaperScaleHighRatioChainIterationSweep` row adds first
   dashboard-selected iteration-budget coverage over 25/50/100/200 max
   iterations with finite replay counters, a tracked benchmark/stability packet,
   and a rendered SVG plot, still without a same-hardware paper-number
   comparison or GPU parity.
   Direct private movable-pair fixed/revolute/prismatic and current-pose
   movable-pair fixed/revolute/prismatic break/reset regressions now verify
   broken rows stay skipped under opposing endpoint forces and re-enter with an
   updated command or persistent all-axis rows. Direct private and current-pose
   spherical movable-pair regressions now prove linear-only rows re-enter after
   reset while relative orientation remains free. Current-pose movable-pair
   revolute/prismatic tiny-limit regressions now also prove generated free-axis
   motor rows honor finite effort caps while captured anchor/hinge/masked rows
   remain active, and restored simulation-mode binary save/load regressions now
   prove those tiny effort caps remain effective after reload. Public
   articulated
   revolute/prismatic floating-endpoint plus selected off-origin-anchor facade
   drive regressions now also use non-cardinal axes and check the generated AVBD
   free-axis basis column before stepping, and public same-multibody
   movable-pair revolute/prismatic facade
   finite-limit regressions now prove capped motors preserve the hard rows
   across two movable endpoints. Public same-multibody movable-pair
   revolute/prismatic broken-state binary save/load/reset regressions now also
   reload broken joints, preserve endpoint/type/axis/actuator command and
   effort-limit state, and prove reset rebuilds the hard rows plus updated
   free-axis motor motion across two movable endpoints. Private generated
   current-pose movable-pair fixed broken-state binary save/load/reset coverage
   now reloads broken all-axis rows, preserves endpoints and captured
   current-pose anchors, and proves reset rebuilds both generated linear and
   angular hard rows. Private generated
   current-pose movable-pair revolute/prismatic broken-state binary
   save/load/reset regressions now also reload broken rows, preserve
   endpoint/type/axis/actuator command, effort-limit state, and captured
   current-pose anchors, and prove reset rebuilds the generated hard rows plus
   updated free-axis motor motion across two movable endpoints. Private
   direct world-link revolute/prismatic broken-state binary save/load/reset
   regressions now also preserve private `AvbdRigidWorldPointJointConfig`
   anchors, bases, masks, endpoint shape, stiffness fields, actuator command,
   and effort-limit state before reset re-engages the hard rows plus updated
   free-axis motor motion, with direct revolute/prismatic coverage now spanning
   both child-link and parent-link endpoint polarity plus in-memory
   parent-endpoint reset re-engagement, and direct private
   world-link fixed/spherical broken-state binary save/load/reset coverage now
   preserves private `AvbdRigidWorldPointJointConfig` anchors, masks, fixed
   child-link target orientation, and spherical parent/child endpoint shape
   before reset re-engages all fixed rows or only the spherical linear anchor
   rows. Private
   generated current-pose movable-pair spherical broken-state binary
   save/load/reset coverage now reloads broken linear rows, preserves endpoints
   and captured current-pose anchors, and proves reset rebuilds only the
   generated linear rows while relative orientation remains free, but broader
   persistent private articulated motor coverage beyond the command-update,
   movable link-pair, tiny-limit, restored tiny-limit save/load, one-DOF
   break/reset, spherical reset, and movable-pair plus direct world-link
   save/load/reset checks remains open.
   Extend that bridge to those row families, then broaden fracture
   lifecycle/corpus coverage beyond the narrow hard point-joint threshold and
   the now-covered private fixed-row/world-fixed reset plus 2D Fracture/3D
   Breakable source-demo fixed-joint break/reset rows, and public articulated
   World facade coverage
   beyond the new same-multibody link-link, world-link, explicit-anchor, and
   spherical linear-only point-joint entrypoints plus same-multibody link-link
   and world-link fixed/spherical save/load rebuilding, including dartpy
   same-multibody/world-link fixed/spherical and one-DOF design-mode rebuild
   evidence, plus
   same-multibody/world-link revolute/prismatic motor save/load rebuilding
   including selected non-cardinal axis-basis persistence and restored
   same-multibody/world-link endpoint-shape assertions plus
   same-multibody/world-link fixed/spherical/revolute/prismatic broken-state
   save/load/reset persistence including explicit-anchor fixed and selected
   non-cardinal one-DOF motor rows with restored effort-limit state. The dartpy
   same-multibody/world-link fixed/spherical and one-DOF binary round-trips,
   direct break/skip/reset non-cardinal basis checks, fixed point-joint
   break/skip/reset endpoint-shape assertions, spherical linear-row
   break/skip/reset endpoint-shape assertions, and explicit-anchor one-DOF
   motor reset endpoint/axis-shape assertions are current evidence rather than
   fresh next targets.
2. **Rigid contact persistence completeness** — broaden narrow-phase endpoint
   feature extraction and row identity so box/sphere/cylinder/capsule/plane/mesh
   contact manifolds persist across realistic rigid stacks and piles, building
   beyond the current known/unknown shape-frame feature mapping, endpoint-A/B
   explicit-shape local-point evidence, actual narrow-phase primitive-feature
   evidence including live mesh-vertex endpoint-feature evidence,
   deterministic same-feature sphere/plane plus sphere/mesh-face,
   sphere/mesh-edge, mesh-vertex replay, and
   mesh-face/mesh-edge/mesh-vertex small-pose persistence plus endpoint-order
   stability, live
   positive/negative cylinder-cap/plane and capsule-cap/plane
   row-order evidence, live cylinder-side/capsule-side and positive/negative
   cylinder-rim row-order evidence, cylinder/capsule cap/side/rim small-pose
   persistence and endpoint-order stability evidence, live
   sphere/plane friction tangent warm-start mapping evidence, live sphere/plane
   normal-rotation friction tangent projection evidence, live box-box
   manifold row-order evidence, live sphere/plane and box-box manifold
   endpoint-order row-identity evidence, live endpoint-swapped box-box friction
   tangent projection evidence, live box-box small-pose row-persistence
   evidence, live box-box manifold friction warm-start persistence evidence,
   the first live stacked static/dynamic and
   dynamic/dynamic box manifold row-persistence and friction warm-start
   persistence evidence, live spanning-top and multi-top box-pile friction
   row-persistence plus contact-order replay and endpoint-swapped tangent
   projection evidence,
   contact-stage/default-step box-pile friction slip-reduction evidence, live
   stacked endpoint-swapped
   friction tangent projection evidence,
   contact-stage kinematic-owned warm-started friction
   coverage with prescribed kinematic tangential velocity ignored as slip input,
   contact-stage enabled-peer/disabled-peer warm-started friction coverage,
   contact-stage simultaneous multi-contact warm-started friction coverage,
   contact-stage box-manifold warm-started friction coverage,
   contact-stage dynamic/dynamic box-manifold warm-started friction coverage,
   contact-stage stacked static/dynamic plus dynamic/dynamic box-manifold
   warm-started friction coverage,
   contact-stage multi-top stacked box-manifold warm-started friction coverage,
   default-step dynamic/dynamic box-manifold warm-started friction coverage,
   default-step stacked static/dynamic plus dynamic/dynamic box-manifold
   warm-started friction coverage,
   default-step multi-top stacked box-manifold warm-started friction coverage,
   and tangent-dual
   projection when contact normals rotate, then
   connect that evidence to the paper's rigid
   stacking/friction scenes.
3. **Paper/source-demo corpus implementation** — the durable
   [`avbd-demo-corpus.md`](104-vertex-block-descent-solver/avbd-demo-corpus.md)
   matrix now owns the 19 `avbd-demo2d` scenes, 14 `avbd-demo3d` scenes, paper
   scenes, website/video scenes, and performance packets. The first baseline row
   now exists for the shared empty-scene smoke, matched source-row metadata, and
   tracked visual/benchmark packet; the `avbd-demo2d`
   Ground/Motor/Hanging Rope/Fracture/Dynamic Friction/Static
   Friction/Pyramid/Cards/Stack/Stack Ratio/Rod/Joint Grid/Rope/Heavy
   Rope/Spring/Spring Ratio/Net
   and `avbd-demo3d` Ground/Dynamic Friction/Static
   Friction/Pyramid/Rope/Heavy Rope/Spring/Spring Ratio/Stack/Stack
   Ratio/Soft Body/Bridge/Breakable rows now
   have matched source-row harnesses. The Dynamic Friction path now also has a
   dashboard friction-coefficient sweep over maximum friction 0, 0.5, 1, 2.5,
   and 5 with a tracked benchmark/native-reference packet and rendered plot.
   All listed rows now
   have tracked
   visual/benchmark/native-reference packets, with the
   narrow 2D Dynamic Friction, 2D Static Friction, 2D
   Pyramid, 2D Stack, 2D Stack Ratio, Fracture, 2D Ground, 3D Ground, 3D Dynamic
   Friction, Pyramid, Stack, Stack Ratio, 3D Soft Body, and Bridge rows recording CPU
   reference wins so far and
   the Motor, Hanging Rope, 2D Cards, 2D Rod, 2D Joint Grid, 2D Rope,
   2D Heavy Rope, 2D Net, 2D/3D Spring, 2D/3D Spring Ratio, 3D Rope, and 3D
   Heavy Rope rows keeping CPU-win gaps
   explicit. Use the matrix to implement or optimize the next source-demo row
   with tests, py-demo/visual smoke, benchmark JSON, CPU reference comparison,
   and GPU parity fields. The Spring and Spring Ratio source rows remain short
   of source parity even though the private rigid radial distance-spring row
   primitive, private serial driver support, World/API extraction, py-demo
   harnesses, benchmark rows, native timing entrypoints, and tracked packets
   now exist. A local follow-up removed small-row builder temporary vector
   allocations for up to 16 point-joint/motor/distance-spring candidates and
   currently smokes at about 8.43 us Motor, 3.98 us Spring, and 36.16 us Spring
   Ratio median step time; refreshed same-command packets now record about
   3.82 us 2D Spring, 31.8 us 2D Spring Ratio, 3.58 us 3D Spring, and 29.1 us
   3D Spring Ratio, and a broader capped finite-row diagnostic
   over Rod, Joint Grid, Soft Body, and Spring rows ran under load average around
   11.7. The checked 2D packets still record DART about
   5.09x slower than the native Spring row and about 4.25x slower than the native
   Spring Ratio row, while the 3D packets record DART about 1.96x and 3.70x
   slower respectively, so they still need CPU performance resolution and GPU parity. The existing deformable
   finite-stiffness rows, hard multibody distance loop closures, and
   articulated per-axis compliant point-joint rows are not source-parity
   substitutes.
4. **GPU row parity plan** — route each landed CPU row family through the
   private CUDA boundary only after the shared CUDA substrate and row inventory
   can preserve warm-started dual/stiffness state deterministically.

## Acceptance Criteria

VBD-parity progress is not complete until the implementation:

- distinguishes each internal kernel slice from a wired, scene-level VBD solver
  in its verification language;
- keeps VBD naming backend-neutral and DART-owned in public signatures/docs;
- proves per-vertex force/Hessian correctness with finite-difference tests, PD
  Hessian guarantees, graph-coloring conflict-freedom, block-descent energy
  decrease, and convergence parity against the existing solver and the
  reference implementations on matched scenes;
- adds FEM hyperelasticity, acceleration, damping, contact, and friction with
  focused tests;
- promotes OGC only after the sidecar proves contact construction,
  conservative bounds, VBD force/Hessian agreement, penetration-free repeated
  steps, documented limitations, benchmark/profiling JSON, and headless visual
  evidence against the same corpus used for IPC/VBD comparison;
- records benchmark/profiling JSON for kernels, the solver, and scenes on both
  CPU and GPU, and demonstrates beating the reference and/or paper numbers
  before any performance-parity claim;
- verifies long-horizon headless Filament captures for GUI examples; and
- keeps `pixi run lint`, `pixi run build`, focused C++ tests, and
  `check-api-boundaries` green for every slice.

AVBD parity additionally requires:

- every algorithm and feature in `avbd-2025`, the project page, videos, and the
  `avbd-demo2d`/`avbd-demo3d` sources to be implemented, including hard
  constraints, bounded inequalities, friction cones, finite-stiffness ramping,
  warm-started dual/stiffness state, alpha regularization, quasi-Newton Hessian
  approximation, 6-DOF rigid bodies, joints, motors, breakable constraints,
  contact persistence, and soft/rigid coupling;
- both CPU and GPU implementations for the solver and all paper/demo benchmark
  families;
- all source demo scenes, paper figures, parameter sweeps, website demos, and
  video/headline scenes represented by DART tests, benchmark packets, py-demos,
  or documented visual evidence; and
- benchmark/profiling JSON proving DART beats the reference demo repositories
  and the paper's published CPU/GPU numbers before any AVBD-complete claim.

## Progress log

Relocated from the dashboard on 2026-07-03; newest first.

The DART-owned VBD CPU+CUDA solver landed on `main` (#2781):
per-vertex block kernels, graph coloring, the colored Gauss-Seidel
block-descent driver, Stable Neo-Hookean tetrahedra, Chebyshev/Rayleigh
acceleration, the implicit-Euler stepper, the opt-in World wiring
(`comps::DeformableVbdConfig` + the `advanceDeformableBody` VBD branch), the
algorithm-neutral public `configureDeformableSolver` API and dartpy binding,
static half-space ground contact + Coulomb friction, the CUDA mass-spring and
tetrahedral rollouts (CUDA-graph capture + mixed precision), the CPU baseline
benchmark, and the first GUI showcases (cloth/net/beam). PR #2801 extends
that landed path by routing VBD tetrahedra through the shared
`deformable_elasticity` FEM kernels, preserving VBD on static sphere/box
obstacle barriers, adding lagged VT/EE surface self-collision penalties, and
adding the TinyVBD tilted-strand plus contact showcase py-demos; it also
retires the temporary `docs/dev_tasks/vbd_deformable_solver/` tracker by
promoting the gap audit into this plan. Remaining VBD closeout work is:
self-contact tangential friction, OGC source/code audit and CPU
proof-of-contact from
[`104-vertex-block-descent-solver/ogc-gap-audit.md`](104-vertex-block-descent-solver/ogc-gap-audit.md),
committed benchmark/profiling JSON for the new scenes, paper tetrahedral scene
reproduction, Phase 8b SoA + Gaia-CPU comparison, and Phase 9 RTX-4090
same-GPU Table 1 reproduction. Maintainer direction now promotes Augmented VBD
(`avbd-2025`) as the next PLAN-104
implementation focus: use
[`104-vertex-block-descent-solver/avbd-paper-gap-audit.md`](104-vertex-block-descent-solver/avbd-paper-gap-audit.md)
plus the active [`../dev_tasks/avbd_solver/`](../dev_tasks/avbd_solver/)
tracker to implement AVBD's hard constraints, bounded contact/friction,
finite-stiffness ramping, 6-DOF rigid/articulated blocks, soft/rigid coupling,
all paper/demo scenes, and CPU/GPU benchmark parity. The local AVBD foundation
now includes the internal scalar-row utility, deterministic row
key/inventory warm-start cache, a standalone CPU half-space contact-normal
block-descent kernel for active mass-spring rows, a scalar hard
point-attachment row kernel/driver, a finite-stiffness spring row
kernel/driver, a combined serial mass-spring AVBD row driver for those row
families plus bounded friction tangents, self-contact normal rows, and
self-contact friction rows, and narrow internal World opt-ins in the
supported static-contact mass-spring, self-contact mass-spring, and
pure-tetrahedral envelopes. Pure-tetrahedral finite-stiffness scenes can now
also opt into AVBD self-contact normal rows and matching self-contact
friction rows in the same serial tet solve, with explicit
fallback coverage for unsupported topology mixes, unrequested self-contact
AVBD rows, Chebyshev, Rayleigh-damped, parallel, and unsupported-row requests;
adjacent friction tangent pairs now use lagged-dual static/dynamic switching
and pairwise circular-cone projection, including supported World generation
for self-contact friction rows. Static box obstacle row keys now distinguish
faces, edges, and corners so same-feature rows warm-start while box-manifold
changes reset normal/friction state; persisting static half-space friction
rows also project their decayed dual into the current tangent basis when smooth
obstacle normals change, and persisting self-contact friction rows project
their generalized tangential dual into the current 12D tangent stencil. The
first private rigid foundation adds a 6-DOF block accumulator, world-frame
quaternion tangent update, inertia term, block solve, scalar rigid
point-attachment row, two-body point-pair row stamping, and private
point-pair contact/friction row constructors plus paired friction-cone
projection, with a private serial row driver for point attachments,
contact-normal point pairs, and paired friction tangent rows. Paired friction
row updates now reuse the regularized tangent constraint vector for both cone
projection and stiffness growth, shared-anchor tangent pairs reuse the first
transformed anchor pair across both tangent rows, and a private
rigid contact-manifold row builder for active contact points. Private
dynamic/rigid contact feature IDs, canonical two-endpoint row keys, and
normal/friction row descriptor helpers have started, and private
World-contact snapshot/solve/writeback helpers now translate rigid-body
`World::collide()` contacts into manifold-point inputs, run them through the
private serial rigid row solve, and write dynamic rigid-body state back to the
ECS through a combined private wrapper in focused tests. The first
contact-stage AVBD activation is now available behind the internal
`RigidAvbdContactConfig`: supported free rigid-body contacts route through
the private 6-DOF AVBD row solve as a velocity-level projection, with focused
dynamic/dynamic and static/dynamic contact-stage coverage in
`World.RigidBodyContactStageAvbdProjectsDynamicDynamicContactVelocity` and
`World.RigidBodyContactStageAvbdProjectsStaticDynamicContactVelocity`,
single-config dynamic/dynamic opt-in coverage in
`World.RigidBodyContactStageAvbdProjectsDynamicPairWithSingleConfig`,
default `World::step()` dynamic/dynamic coverage in
`World.RigidBodyContactStageAvbdDynamicDynamicRunsThroughDefaultWorldStep`,
default `World::step()` single-config dynamic/dynamic coverage in
`World.RigidBodyContactStageAvbdDynamicPairWithSingleConfigRunsThroughDefaultWorldStep`,
contact-to-position pipeline coverage in
`World.RigidBodyContactStageAvbdFeedsRigidBodyPositionStage`,
default `World::step()` schedule coverage in
`World.RigidBodyContactStageAvbdRunsThroughDefaultWorldStep`,
static-owned static/dynamic opt-in coverage in
`World.RigidBodyContactStageAvbdProjectsStaticOwnedContactConfig`,
default `World::step()` static-owned static/dynamic opt-in coverage in
`World.RigidBodyContactStageAvbdStaticOwnedRunsThroughDefaultWorldStep`,
stored static-body velocity ignore coverage in
`World.RigidBodyContactStageAvbdIgnoresStoredStaticVelocity`,
kinematic-as-prescribed endpoint coverage in
`World.RigidBodyContactStageAvbdTreatsKinematicBodyAsStaticObstacle`,
default `World::step()` kinematic-prescribed endpoint coverage in
`World.RigidBodyContactStageAvbdKinematicRunsThroughDefaultWorldStep`,
enabled-peer/disabled-peer opt-in coverage in
`World.RigidBodyContactStageAvbdProjectsEnabledPeerWithDisabledConfig`,
default `World::step()` enabled-peer/disabled-peer coverage in
`World.RigidBodyContactStageAvbdEnabledPeerWithDisabledConfigRunsThroughDefaultWorldStep`,
configured multi-contact coverage in
`World.RigidBodyContactStageAvbdProjectsMultipleConfiguredContacts`, plus
default `World::step()` configured multi-contact coverage in
`World.RigidBodyContactStageAvbdMultipleConfiguredContactsRunThroughDefaultWorldStep`,
mixed-config all-or-nothing fallback coverage in
`World.RigidBodyContactStageAvbdFallsBackForUnconfiguredContactSet`,
default `World::step()` mixed-config all-or-nothing fallback coverage in
`World.RigidBodyContactStageAvbdMixedConfigFallsBackThroughDefaultWorldStep`,
disabled-config opt-out fallback coverage in
`World.RigidBodyContactStageAvbdDisabledConfigFallsBack`, default
`World::step()` disabled-config fallback coverage in
`World.RigidBodyContactStageAvbdDisabledConfigFallsBackThroughDefaultWorldStep`,
and
static/dynamic and dynamic/dynamic warm-started friction slip-reduction
coverage in `World.RigidBodyContactStageAvbdWarmStartedFrictionReducesSlide`
and
`World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionReducesSlip`,
static-owned warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionReducesSlide`,
kinematic-owned warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionReducesSlide`,
enabled-peer/disabled-peer warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionReducesSlide`,
simultaneous multi-contact warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionReducesSlide`,
live box-box manifold warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionReducesSlide`,
live dynamic/dynamic box-manifold warm-started friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionReducesSlip`,
stacked static/dynamic plus dynamic/dynamic box-manifold warm-started
friction coverage in
`World.RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionReducesSlip`,
multi-top stacked box-manifold warm-started friction contact-stage coverage in
`World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionReducesSlip`,
plus default `World::step()` warm-started friction schedule coverage in
`World.RigidBodyContactStageAvbdWarmStartedFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
`World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
and
`World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionRunsThroughDefaultWorldStep`,
while
unsupported envelopes fall back to the existing sequential-impulse path. The
private rigid contact snapshot now derives box face/edge/corner endpoint
feature IDs, scopes row ordinals per canonical endpoint pair, and assigns
same-feature manifold rows from deterministic canonical-local point ordering
for narrower warm-start persistence, including endpoint-order row-identity
evidence, actual `World::collide()` sphere/plane contact-point replay,
sphere/mesh-face, sphere/mesh-edge, mesh-vertex replay, and
mesh-face/mesh-edge/mesh-vertex small-pose persistence plus endpoint-order
stability,
cylinder/capsule cap/side/rim small-pose persistence and endpoint-order
stability, and live box-box manifold box-face feature evidence. The first
wider live box-pile friction row-persistence regression now preserves per-pair
row ordinals, friction coefficients, Coulomb bounds, and warm-started friction
dual state across a small pose nudge; the multi-top companion preserves the
same row state across two lower supports and two independent upper dynamic
bodies; the contact-order replay companion
keeps those row keys and lambdas stable when the live contact vector is
reversed; and the endpoint-order regression keeps those per-pair bounds while
projecting paired tangent duals into the swapped physical tangent basis. The
live contact-stage/default-step pile companions now carry the same two-lower
plus spanning-top stack through `RigidBodyContactStage`, reducing top/lower
tangential slip in both schedules.
Cylinder side/cap/rim and
capsule side/top-cap/bottom-cap endpoint
features extend the same private identity path beyond boxes, and known and
unknown-index contacts now map world points through body and collision-shape
local transforms before feature coding,
while explicit endpoint-A/B compound shape-index feature coding uses
narrow-phase shape-local contact points; actual `World::collide()`
sphere/cylinder/capsule/plane/mesh contacts now cover the primitive-feature
path, including shape-scoped sphere body features, and
single-shape and uniquely containing compound-shape fallback coverage remains
in the rigid snapshot path.
Persisting private rigid contact friction rows now retain their
previous tangent directions and project the warm-started paired dual into the
current tangent basis when the contact normal rotates. The private rigid row
path now has point-joint linear, angular,
and combined row builders for fixed-anchor
translation and orientation constraints, with step-start previous values
seeded for AVBD alpha regularization. Those private point-joint rows can now
be appended to the World rigid snapshot/solve/apply wrapper and combined step
helper from world-space point-joint inputs, and a private fixed-joint ECS
extractor can feed the step helper for rigid-body-linked joint entities. The
internal contact-stage AVBD opt-in can project those fixed-joint rows with or
without active contacts. The private point-joint builders now accept per-axis
linear and angular masks, preserving all-axis fixed-joint behavior while
enabling limited-DOF configs to reuse the same descriptor and warm-start path.
Named private revolute and prismatic point-joint configs now build arbitrary
joint-axis bases, leave one rotational or translational axis free, and
preserve axes/masks through World point-joint input and solve coverage.
Simulation-entry current-pose initialization and extraction also cover
private rigid-body ECS revolute/prismatic joint entities. Public experimental
`World` facades now expose free rigid-body revolute and prismatic joints
through C++ and dartpy, backed by generated stubs, focused C++/Python tests,
and the categorized `sx_rigid_limited_joints` py-demo. The first AVBD
rigid-constraint `py-demos` scene,
`avbd_rigid_fixed_joint_contact`, exposes the fixed-joint/contact slice as a
user-visible showcase. Public free-rigid-body revolute/prismatic velocity
actuators now extract to private bounded AVBD angular/linear motor rows, with
persistent contact-stage motor inventory, categorized
`avbd_rigid_revolute_motor` and `avbd_rigid_prismatic_motor` py-demos,
dashboard benchmark rows for those public motor paths, and tracked
visual/benchmark packets.
Public articulated one-DOF velocity motors now also expose command-update
behavior through the categorized `avbd_articulated_revolute_motor` and
`avbd_articulated_prismatic_motor` py-demos, with tracked visual/benchmark
packets for both paths, while
`avbd_articulated_motor_breakable_joint` exposes the same-multibody public
articulated motor break/reset lifecycle with post-reset reversed-command
plus weak re-arm coverage and a tracked packet,
`avbd_articulated_prismatic_pair_motor_breakable_joint` exposes the
same-multibody public articulated prismatic motor break/reset lifecycle with
post-reset reversed-command plus weak re-arm coverage and a tracked packet,
and
`avbd_articulated_prismatic_motor_breakable_joint` exposes the world-anchored
public articulated prismatic motor break/reset lifecycle with a tracked
packet plus post-reset reversed-command and weak re-arm coverage, while
`avbd_articulated_world_revolute_motor_breakable_joint` exposes the
world-anchored public articulated revolute motor break/reset lifecycle with a
tracked packet and post-reset reversed-command plus weak re-arm coverage. The
bounded AVBD World
dashboard slice now includes matching public articulated
revolute/prismatic/breakable motor step rows, the active prismatic
breakable-motor step row, the active world-prismatic breakable-motor step
row, the active world-revolute breakable-motor step row, plus public
free-rigid/articulated breakable fixed point-joint step rows backed by
tracked visual/benchmark packets, plus public free-rigid/world-link/
same-multibody spherical breakable point-joint rows backed by tracked packets.
Public free-rigid-body point joints also expose a narrow break-force and
broken-state lifecycle, mark solved AVBD point joints broken when row load
reaches the threshold, skip broken joints during later extraction, and expose
fixed/revolute/prismatic/spherical binary save/load broken-state reset
coverage plus same-multibody/world-link articulated fixed/revolute/prismatic/
spherical binary broken-state round-trips preserving one-DOF command/effort-limit
motor state, dartpy same-multibody/world-link fixed/spherical and one-DOF
design-mode rebuild checks, and
free-rigid/articulated design-mode AVBD point-joint stiffness facade binary
persistence plus direct C++/dartpy validation of public articulated stiffness
defaults, finite setters, invalid setter rejection, and C++/dartpy
endpoint-ownership rejection plus the
`avbd_rigid_breakable_joint` py-demo with reset/re-engagement coverage
and tracked packet evidence
plus `avbd_rigid_spherical_breakable_joint` for spherical anchor-only
reset/re-engagement with tracked packet evidence; both free-rigid breakable
demo regressions verify weak re-arm breaks again after high-force reset.
Public articulated fixed
point-joints now also expose the narrow world-link break/reset lifecycle
through the categorized `avbd_articulated_breakable_joint` py-demo with
tracked packet evidence and the
same-multibody break/reset lifecycle through
`avbd_articulated_fixed_pair_breakable_joint` with tracked packet evidence,
both with weak re-arm breakage coverage after reset,
while public articulated
spherical point-joints expose the matching linear-only
anchor break/reset path through
`avbd_articulated_spherical_breakable_joint` for world-link endpoints and
`avbd_articulated_spherical_pair_breakable_joint` for same-multibody
endpoints, both with tracked packet evidence and weak re-arm breakage
coverage after reset. A narrow
`avbd_articulated_high_ratio_chain` py-demo,
`BM_AvbdArticulatedHighRatioChainStep` dashboard row, and
[`avbd-articulated-high-ratio-chain-packet.json`](104-vertex-block-descent-solver/avbd-articulated-high-ratio-chain-packet.json)
now also cover a five-link variational-chain smoke with a 200:1 heavy tip,
and `avbd_paper_scale_high_ratio_chain`,
`PaperScaleHighRatioChainStaysFiniteAndResets`, plus
`BM_AvbdPaperScaleHighRatioChainStep` cover a 50-link/50,000:1 finite/reset
and visual/CPU benchmark smoke through configured `World::step()` solve-budget
fields with
[`avbd-paper-scale-high-ratio-chain-packet.json`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-chain-packet.json)
visual/benchmark evidence, while
`BM_AvbdPaperScaleHighRatioChainIterationSweep` adds dashboard-selected
25/50/100/200 max-iteration sweep coverage for that same paper-scale fixture
with finite replay counters, tracked benchmark/stability evidence in
[`avbd-paper-scale-high-ratio-iteration-sweep-packet.json`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-packet.json),
and a rendered
[`avbd-paper-scale-high-ratio-iteration-sweep-plot.svg`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-plot.svg).
The same-hardware comparison and GPU gates remain open. Solver-identity
relabel (PLAN-091 WP-091.1): no
`avbd-demo2d`/`avbd-demo3d` benchmark or py-demo scene emplaces the internal
AVBD rigid-contact opt-in config (`comps::RigidAvbdContactConfig`), because
AVBD contact is not facade-selectable, so every rigid contact in the
source-row scenes below ran DART's default sequential-impulse contact path.
The native-runner timing ratios for contact scenes are whole-pipeline
`World::step` comparisons, not AVBD-contact-solver comparisons: the
pure-contact rows (2D Dynamic Friction, Static Friction, Pyramid, Cards,
Stack, and Stack Ratio; 3D Ground, Dynamic Friction, Static Friction,
Pyramid, Stack, and Stack Ratio) timed no AVBD rows at all; the
joint-plus-contact rows (2D Fracture, Soft Body, Joint Grid, and Net; 3D
Soft Body, Bridge, and Breakable) timed AVBD point-joint/motor/spring rows
while their ordinary contacts ran sequential impulse; and incidental
link-link contacts in the chain rows (2D Rod, Rope, Heavy Rope, and Hanging
Rope; 3D Rope and Heavy Rope) also ran sequential impulse. This relabel
changes no committed packet bytes and neither closes nor reopens any
PLAN-104 completion gate; new AVBD evidence packets must machine-record
`resolved_solver_identity` at AVBD packet schema version 2, enforced by
`pixi run check-avbd-packets`. Public
empty-scene corpus baseline coverage is now visible through
`avbd_empty_baseline`, a focused Python smoke that checks source revisions,
default source parameters, and the `sceneEmpty` zero-count invariant, and
`BM_AvbdEmptyWorldStep`; the tracked
`avbd-empty-baseline-packet.json` records the corresponding headless
visual-capture hashes and Google Benchmark row. The first non-empty
2D source row is now visible through `avbd_demo2d_ground`, which matches the
`avbd-demo2d` Ground source revision, scene index, one static slab, one rigid
body, one collision shape, no joints, and no dynamic bodies, plus
`BM_AvbdDemo2dGroundStep`. The tracked
`avbd-demo2d-ground-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing; after skipping static-only contact queries,
no-op rigid dynamics stages, clean frame-cache graph execution, and the clean
no-work default step pipeline with a cheap scratch reset, it records DART
about 1.51x faster than the native static Ground runner on this host, closing
that narrow CPU-win gate. The
first one-DOF motor source-demo row is now visible through
`avbd_demo2d_motor`, which matches the
`avbd-demo2d` Motor scene's source revision, scene index, default parameters,
20 rad/s target speed, and 50 N m effort bound, plus
`BM_AvbdDemo2dMotorStep`. The tracked
`avbd-demo2d-motor-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing. After the rigid contact stage skips contact
queries for worlds with no collision geometry in prepare/execute, the packet
still records the DART public World row about 6.18x slower than the native
source runner on this host, so the CPU-win gate remains open. The next
non-empty
`avbd-demo2d` source row is now visible through
`avbd_demo2d_hanging_rope`, which matches the `avbd-demo2d` Hanging Rope
source revision, scene index, 49 regular links, one 10 m endpoint block, 49
linear-only point joints, and 50 collision shapes, plus
`BM_AvbdDemo2dHangingRopeStep`. The tracked
`avbd-demo2d-hanging-rope-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 7.84x slower
than the native Hanging Rope runner on this host, keeping that CPU-win gate
open. The first radial distance-spring source harnesses are now visible
through `avbd_demo2d_spring` and `avbd_demo2d_spring_ratio`, which match the
`avbd-demo2d` Spring and Spring Ratio source rows over public free-rigid
distance springs, plus `BM_AvbdDemo2dSpringStep` and
`BM_AvbdDemo2dSpringRatioStep`. Their tracked
visual/DART-benchmark/native-timing packets now record the spring-connected
ignored-pair counts and DART about 4.02x slower than the native Spring runner
and about 4.48x slower than the native Spring Ratio runner on this host, so
CPU performance resolution and GPU parity remain open. The next
`avbd-demo2d` source row is now visible through
`avbd_demo2d_fracture`, which matches the `avbd-demo2d` Fracture source
revision, scene index, 11 chain links, two dynamic supports, 15 falling
blocks, 10 breakable fixed joints, and 29 collision shapes, plus
`BM_AvbdDemo2dFractureStep`. Focused integration coverage now also verifies
that the source-row fixed joints fracture, reset at a high break force, stay
unbroken, and reduce their anchor residuals again. The tracked
`avbd-demo2d-fracture-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; after a refreshed same-source
timing run, it records DART about 1.20x faster than the native Fracture
runner on this host, closing only that narrow source-row CPU gate. Later
local cleanup replaces the contact stage's duplicate prepare-time collision
query with collision-shape-count constraint prewarm and aligns live
constrained-pair filtering with the native solver, and that refreshed packet
now captures the comparison.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_dynamic_friction`, which matches the `avbd-demo2d` Dynamic
Friction source revision, scene index, 11 sliding boxes, source friction
coefficients from 5.0 down to 0.0, a static ground, and 12 collision shapes,
plus `BM_AvbdDemo2dDynamicFrictionStep`. The tracked
`avbd-demo2d-dynamic-friction-packet.json` adds headless visual capture,
DART benchmark JSON, and native source timing; it records DART about 1.83x
faster than the native Dynamic Friction runner on this host, closing that
narrow CPU-win gate while leaving broad friction scenes and GPU packets open.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_static_friction`, which matches the `avbd-demo2d` Static
Friction source revision, scene index, one rotated static ground slab, 11
rotated dynamic boxes, uniform source friction 1.0, and 12 collision shapes,
plus `BM_AvbdDemo2dStaticFrictionStep`. The tracked
`avbd-demo2d-static-friction-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 2.68x faster
than the native Static Friction runner on this host, closing that narrow
CPU-win gate while leaving broad friction scenes and GPU packets open.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_pyramid`, which matches the `avbd-demo2d` Pyramid source
revision, scene index, a static ground, 210 dynamic boxes in the source
pyramid layout, and 211 collision shapes, plus
`BM_AvbdDemo2dPyramidStep`. The tracked
`avbd-demo2d-pyramid-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing; it records DART about 9.84x faster than the
10,000-step native Pyramid runner on this host, closing that narrow row only.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_stack`, which matches the `avbd-demo2d` Stack source revision,
scene index, 20 vertical dynamic boxes over static ground, and 21 collision
shapes, plus `BM_AvbdDemo2dStackStep`. The tracked
`avbd-demo2d-stack-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing; it records DART about 2.17x faster than the
native Stack runner on this host, closing that narrow row only.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_stack_ratio`, which matches the `avbd-demo2d` Stack Ratio
source revision, scene index, six geometric-size dynamic boxes over static
ground, and 7 collision shapes, plus `BM_AvbdDemo2dStackRatioStep`. The
tracked `avbd-demo2d-stack-ratio-packet.json` adds headless visual capture,
DART benchmark JSON, and native source timing; it records DART about 2.22x
faster than the native Stack Ratio runner on this host, closing that narrow
row only.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_rod`, which matches the `avbd-demo2d` Rod source revision,
scene index, 20 rigid links, one static anchor link, 19 all-axis public fixed
joints, and 20 collision shapes, plus `BM_AvbdDemo2dRodStep`. The tracked
`avbd-demo2d-rod-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing; it records DART about 9.58x slower than the
native Rod runner on this host, so that CPU-win gate remains open.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_soft_body`, which matches the `avbd-demo2d` Soft Body source
revision, scene index, one static ground slab, two 15x5 dynamic rigid-box
lattices, 260 finite-stiffness all-axis public fixed joints, 224 diagonal
ignored collision pairs, and 151 collision shapes, plus
`BM_AvbdDemo2dSoftBodyStep`. The tracked
`avbd-demo2d-soft-body-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 6.30x slower
than the native Soft Body runner on this host, so that CPU-win gate remains
open. Stable same-order AVBD row inventories now warm-start in place and row
ordinal counters now use reserved endpoint-pair hash maps. The rigid row
append paths seed fallback snapshot body-index cache entries, and the rigid
row driver also skips per-body row-index scratch for row families absent in a
solve, keeps unchanged source-row index layouts warm across frames, and
routes single-family point-pair/angular solves without rebuilding combined row
vectors, while one-new-row point-joint/distance-spring appends skip endpoint
row-counter hash-map setup. Friction tangent-pair rows now reuse precomputed
world anchors across force and direction stamping in one pair evaluation. The
contact-stage AVBD path now reuses a
scratch contact snapshot through an in-place builder, skips pair-constraint
extraction when no point-joint/distance-spring configs exist, and
extracts/appends point-joint and distance-spring families independently when
only one family exists. The rigid snapshot solve now also clears absent
row-family inventories directly instead of calling empty
contact/joint/motor/spring builders, small point-joint/motor/distance-spring
row builders use stack descriptor/active-row storage for up to 16 candidate
rows instead of allocating temporary vectors, and the split rigid-body velocity
stage assembles force batches only for advanceable bodies, but the packet
CPU-win gate stays open until refreshed same-command evidence beats the native
runner.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_joint_grid`, which matches the `avbd-demo2d` Joint Grid source
revision, scene index, 625 rigid boxes, two static top-corner anchors, 1200
all-axis public fixed joints, and 625 collision shapes, plus
`BM_AvbdDemo2dJointGridStep`. The tracked
`avbd-demo2d-joint-grid-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 2.06x slower
than the native Joint Grid runner on this host after reusing per-body
row-index scratch, caching snapshot body indices, reusing row-assembly
world-anchor computations through force/Hessian/Jacobian stamping, and
caching all-axis angular-row orientation errors, with the source diagonal
ignore-collision filter configured through DART's public per-pair API.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_cards`, which matches the `avbd-demo2d` Cards source revision,
scene index, one static ground slab, 40 thin dynamic cards across five
levels, and 41 collision shapes, plus `BM_AvbdDemo2dCardsStep`. The tracked
`avbd-demo2d-cards-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing; it records DART about 5.38x slower than the
native Cards runner on this host. Later friction tangent-pair world-anchor
reuse, row-state constraint-vector reuse, and shared-anchor reuse removed
local contact-heavy duplicate work but have not refreshed this packet,
keeping that CPU-win gate open.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_rope`, which matches the `avbd-demo2d` Rope source revision,
scene index, 20 rigid links, 19 linear-only public spherical point joints,
and 20 collision shapes, plus `BM_AvbdDemo2dRopeStep`. The tracked
`avbd-demo2d-rope-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing; it records DART about 5.20x slower than the
native Rope runner on this host, so that CPU-win gate remains open.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_heavy_rope`, which matches the `avbd-demo2d` Heavy Rope source
revision, scene index, 19 regular links, a 30 m endpoint block, 19
linear-only public spherical point joints, and 20 collision shapes, plus
`BM_AvbdDemo2dHeavyRopeStep`. The tracked
`avbd-demo2d-heavy-rope-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 5.85x slower
than the native Heavy Rope runner on this host after reusing per-body
row-index scratch, caching snapshot body indices, avoiding redundant
row-assembly world-anchor transforms, reusing precomputed distance-spring
Hessian/Jacobian world points, skipping capped finite-row constraint
recomputation, and caching grouped angular-row orientation errors, so
that CPU-win gate remains open.
The next `avbd-demo2d` source row is now visible through
`avbd_demo2d_net`, which matches the `avbd-demo2d` Net source revision, scene
index, one static ground slab, 40 endpoint-pinned net links, 50 falling rigid
blocks, 39 linear-only public spherical point joints, and 91 collision shapes,
plus `BM_AvbdDemo2dNetStep`. The tracked
`avbd-demo2d-net-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing; it records DART about 1.23x slower than the
native Net runner on this host after caching snapshot body indices, reusing
row-assembly world-anchor computations through force/Hessian/Jacobian
stamping, reusing friction tangent-pair world anchors, reusing row-state
tangent constraint values, reusing shared tangent-row anchors, skipping capped
finite-row constraint recomputation, and caching grouped angular-row
orientation errors, with local-anchor point-joint extraction now using a
stable full-rank linear basis for spherical point joints, so that CPU-win gate
remains open.
The next 3D source row is now visible through `avbd_demo3d_ground`, which
matches the
`avbd-demo3d` Ground source revision, scene index, a static floor, falling
rigid box, 2 rigid bodies, and 2 collision shapes, plus
`BM_AvbdDemo3dGroundStep`. The tracked `avbd-demo3d-ground-packet.json` adds
headless visual capture, DART benchmark JSON, and native source timing; it
records DART about 1.11x faster than the native Ground runner on this host,
closing that narrow row only. The next non-empty source row after that is
visible through `avbd_demo3d_dynamic_friction`, which matches the
`avbd-demo3d` Dynamic Friction source revision, scene index, 11 sliding rigid
boxes, a static floor, and 12 collision shapes, plus
`BM_AvbdDemo3dDynamicFrictionStep`. The tracked
`avbd-demo3d-dynamic-friction-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 1.41x faster
than the native Dynamic Friction runner on this host, closing that narrow row
only. The next non-empty source row after that is visible through
`avbd_demo3d_static_friction`, which matches the
`avbd-demo3d` Static Friction source revision, scene index, a static floor,
inclined static ramp, 11 sliding rigid boxes, and 13 collision shapes, plus
`BM_AvbdDemo3dStaticFrictionStep`. The tracked
`avbd-demo3d-static-friction-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 1.08x faster
than the native Static Friction runner on this host, closing that narrow
CPU-win gate. The next non-empty source row after that is visible through
`avbd_demo3d_pyramid`, which matches the `avbd-demo3d` Pyramid source
revision, scene index, a static ground, 136 dynamic boxes in the triangular
pile layout, and 137 collision shapes, plus
`BM_AvbdDemo3dPyramidStep`. The tracked `avbd-demo3d-pyramid-packet.json`
adds headless visual capture, DART benchmark JSON, and native source timing;
it records DART about 2.83x faster than the native Pyramid runner on this
host, closing that narrow row only. The next non-empty source row after that
is visible through `avbd_demo3d_rope`, which matches the `avbd-demo3d` Rope
source revision, scene index, 20 rigid links, 19 anchored linear-only point
joints, and 21 collision shapes, plus `BM_AvbdDemo3dRopeStep`. The tracked
`avbd-demo3d-rope-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing; it records DART about 3.75x slower than the
native Rope runner on this host, so that CPU-win gate remains open. The next
non-empty source row after that is visible through
`avbd_demo3d_heavy_rope`, which matches the `avbd-demo3d` Heavy Rope source
revision, scene index, 19 regular links, a 5 m endpoint block, 19 anchored
linear-only point joints, and 21 collision shapes, plus
`BM_AvbdDemo3dHeavyRopeStep`. The tracked
`avbd-demo3d-heavy-rope-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 3.84x slower
than the native Heavy Rope runner on this host, so that CPU-win gate remains
open. The next non-empty source row after that is visible through
`avbd_demo3d_stack`, which matches the `avbd-demo3d` Stack
source revision, scene index, 10 vertical dynamic boxes over static ground,
and 11 collision shapes, plus `BM_AvbdDemo3dStackStep`. The tracked
`avbd-demo3d-stack-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing; it records DART about 1.80x faster than the
native Stack runner on this host, closing that narrow row only. The next
non-empty source row after that is visible through
`avbd_demo3d_stack_ratio`, which matches the `avbd-demo3d` Stack Ratio source
revision, scene index, four geometric-size dynamic boxes over static ground,
and 5 collision shapes, plus `BM_AvbdDemo3dStackRatioStep`. The tracked
`avbd-demo3d-stack-ratio-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 2.32x faster
than the native Stack Ratio runner on this host, closing that narrow row only.
The next non-empty source row after that is visible through
`avbd_demo3d_soft_body`, which matches the `avbd-demo3d` Soft Body source
revision, scene index, three 4x4x4 dynamic rigid-box lattices, 432
finite-stiffness all-axis fixed joints, 648 diagonal ignored collision pairs,
and 193 collision shapes, plus `BM_AvbdDemo3dSoftBodyStep`. The tracked
`avbd-demo3d-soft-body-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 1.21x faster
than the native Soft Body runner on this host, closing that narrow CPU row
only. Stable same-order AVBD row inventories now warm-start in place and row
ordinal counters now use reserved endpoint-pair hash maps. The rigid row
append paths seed fallback snapshot body-index cache entries, and the rigid
row driver also skips per-body row-index scratch for row families absent in a
solve, keeps unchanged source-row index layouts warm across frames, and
routes single-family point-pair/angular solves without rebuilding combined
row vectors, while one-new-row point-joint/distance-spring appends skip
endpoint row-counter hash-map setup. The contact-stage AVBD path now reuses a
scratch contact snapshot through an in-place builder, skips pair-constraint
extraction when no point-joint/distance-spring configs exist, and
extracts/appends point-joint and distance-spring families independently when
only one family exists. The rigid snapshot solve now also clears absent
row-family inventories directly instead of calling empty
contact/joint/motor/spring builders, small point-joint/motor/distance-spring
row builders use stack descriptor/active-row storage for up to 16 candidate
rows instead of allocating temporary vectors, and the split rigid-body velocity
stage assembles force batches only for advanceable bodies, but the packet
CPU-win gate stays open until refreshed same-command evidence beats the native
runner. The next non-empty source row after that is visible through
`avbd_demo3d_bridge`, which matches the `avbd-demo3d` Bridge source revision,
scene index, 40 planks, 50 load boxes, 78 paired linear-only point joints,
and 91 collision shapes, plus `BM_AvbdDemo3dBridgeStep`. The tracked
`avbd-demo3d-bridge-packet.json` adds headless visual capture, DART benchmark
JSON, and native source timing; it records DART about 1.61x faster than the
native Bridge runner on this host, closing that narrow row only. The next
non-empty source row after that is visible through
`avbd_demo3d_breakable`, which matches the
`avbd-demo3d` Breakable source revision, scene index, 19 rigid bodies, 10
breakable fixed joints, and 19 collision shapes, plus
`BM_AvbdDemo3dBreakableStep`. Focused integration coverage now also verifies
that the source-row fixed joints fracture, reset at a high break force, stay
unbroken, and reduce their anchor residuals again. The tracked
`avbd-demo3d-breakable-packet.json` adds headless visual capture, DART
benchmark JSON, and native source timing; it records DART about 1.42x faster
than the native Breakable runner on this host. The next `avbd-demo3d` source
rows are now visible through `avbd_demo3d_spring` and
`avbd_demo3d_spring_ratio`, which match the `avbd-demo3d` Spring and Spring
Ratio source rows over public free-rigid distance springs, plus
`BM_AvbdDemo3dSpringStep` and
`BM_AvbdDemo3dSpringRatioStep`. Their tracked
visual/DART-benchmark/native-timing packets now record the spring-connected
ignored-pair counts and DART about 2.47x slower than the native Spring runner
and about 3.57x slower than the native Spring Ratio runner on this host, so
CPU performance resolution and GPU parity remain open. Other missing corpus
items include Spring and Spring Ratio GPU
gates, the 2D Motor, Hanging Rope, 2D Cards, 2D Rod, 2D Joint Grid,
2D Rope, 2D Heavy Rope, 2D Net, 3D Rope, and 3D Heavy Rope CPU gaps, remaining
CPU reference wins, broad 2D stack variants, and GPU packets remain missing.
multibody/articulated AVBD state is only narrowly wired: the private endpoint
layer distinguishes free
rigid-body endpoints from multibody links, fixed multibody-link point-joint
configs can bridge into the variational articulated solve path, the branch
has a `BM_AvbdRigidEndpointClassification` benchmark row plus focused C++
bridge tests, and public same-multibody/world-link articulated
fixed/revolute/prismatic/spherical facades now feed the current-pose extractor
through C++/dartpy with focused spherical linear-only pinned-anchor behavior
including explicit link-link/world-link anchors,
bounded revolute/prismatic velocity-actuator,
same-multibody/world-anchored tiny effort-limit and command-update coverage,
private revolute/prismatic command-update, fixed-row and revolute/prismatic
break/reset re-engagement,
same-multibody/world-anchored public one-DOF motor break/skip and
reset/re-engagement,
selected direct same-multibody/world-link one-DOF break/skip/reset
non-cardinal basis checks,
including focused dartpy stepping coverage for same-multibody/world-link
revolute and prismatic explicit-anchor cases,
same-multibody/world-anchored public one-DOF motor break/reset
re-engagement, movable-movable same-multibody revolute/prismatic motor
break/reset with explicit off-origin anchors covered in C++/dartpy,
direct/private movable-pair fixed/revolute/prismatic reset plus
direct/private and current-pose spherical linear-row reset, current-pose
movable-pair fixed/revolute/prismatic reset, and non-cardinal motor-axis
coverage, public
same-multibody/world-anchored articulated revolute/prismatic
floating-endpoint plus selected off-origin-anchor facade non-cardinal
motor-axis coverage,
current-pose
fixed/prismatic reset regressions, private current-pose revolute/prismatic
tiny-limit movable-pair coverage, public same-multibody/world-link one-DOF
non-cardinal finite-limit coverage, public same-multibody movable-pair
revolute/prismatic non-cardinal motor-axis and finite-limit coverage, public
same-multibody movable-pair revolute/prismatic broken-state save/load/reset
coverage, private generated current-pose movable-pair fixed and
revolute/prismatic broken-state save/load/reset coverage, private generated current-pose
movable-pair spherical broken-state save/load/reset coverage, direct private
world-link one-DOF broken-state save/load/reset coverage preserving private
`AvbdRigidWorldPointJointConfig` basis/mask/anchor state, private
current-pose spherical reset regression for linear-only rows, and
fixed break/save/load/reset with dartpy stepping coverage, explicit local/world anchor projection including
same-multibody and world-anchored off-origin fixed/revolute/prismatic anchors and
revolute/prismatic motors,
world-fixed break/reset, same-multibody fixed break/reset py-demo coverage,
same-multibody/world-link spherical linear-row break/skip and reset including
focused dartpy stepping coverage and categorized same-multibody/world-link
py-demos, same-multibody link-link and world-link fixed/spherical save/load
rebuilding of the private all-axis and linear rows, including selected dartpy
same-multibody/world-link fixed/spherical and one-DOF design-mode rebuild
checks, same-multibody/world-link
revolute/prismatic motor save/load rebuilding of the private hard rows and
free-axis motor row with selected non-cardinal axis-basis persistence,
same-multibody/world-link
fixed/spherical/revolute/prismatic broken-state save/load/reset persistence
including movable-movable fixed coverage and selected non-cardinal one-DOF
motor rows with focused C++ binary round-trips preserving one-DOF
command/effort-limit motor state plus dartpy same-multibody/world-link
fixed/spherical and one-DOF binary round-trips and direct
break/skip/reset non-cardinal basis checks,
public free-rigid/articulated AVBD point-joint
stiffness facade binary persistence plus direct C++/dartpy validation of
articulated stiffness defaults, finite setters, invalid setter rejection, and
C++/dartpy endpoint-ownership rejection for same-link/cross-multibody/cross-world
articulated point-joint requests,
a narrow five-link 200:1 high mass-ratio
articulated-chain smoke py-demo plus dashboard row, a focused 50-link/50,000:1
finite/reset stability smoke and matching
`BM_AvbdPaperScaleHighRatioChainStep` dashboard row plus benchmark packet
through configured `World::step()` solve-budget fields, the new
`BM_AvbdPaperScaleHighRatioChainIterationSweep` dashboard-selected
25/50/100/200 max-iteration sweep row plus benchmark/stability packet and
rendered SVG plot,
world-anchor coverage, and a
per-multibody link-index cache in the
articulated point-joint extractor so same-multibody/world-link AVBD private
rows no longer rescan structure membership for every point-joint config.
Rigid AVBD point-joint and distance-spring extraction now also share a
projectable-body metadata lookup, avoiding a second transform lookup after
endpoint classification in the pair-constraint hot path and reusing the
already-checked projectable transform, mass, and static tag when contact
snapshots materialize body state. The same metadata path also skips
static-static rigid point-joint and distance-spring pairs before input or row
construction.
The
related `variational_endpoint_loop_closure` py-demo previews public
loop-closure behavior but does not exercise the private AVBD config extractor.
The next local slice should broaden persistent articulated motor/fracture
lifecycle coverage beyond the current revolute/prismatic command-update,
movable link-pair motor/fixed reset projection, direct/private and
current-pose movable-pair fixed/revolute/prismatic break/reset coverage,
current-pose/public same-multibody and world-anchored
floating-endpoint plus selected off-origin-anchor facade non-cardinal axis and
selected save/load and broken-state save/load/reset non-cardinal axis-basis
persistence plus selected direct break/skip/reset non-cardinal axis-basis
checks plus
same-multibody/world-link one-DOF and public movable-pair finite-limit plus
broken-state save/load/reset coverage with non-cardinal axis-basis checks,
private generated current-pose movable-pair fixed, one-DOF, and spherical
broken-state save/load/reset coverage, with narrow breakable benchmark rows,
world-fixed reset, non-cardinal dartpy same-multibody/world-link one-DOF
reset endpoint/axis-shape checks, and dartpy fixed/spherical reset
endpoint-shape checks now treated as covered current evidence,
matched-metadata empty-scene baseline smoke, the 2D Fracture/3D Breakable
source-demo fixed-joint break/reset coverage, the first non-empty
`avbd-demo2d` Ground, Motor, Hanging Rope, Fracture, Dynamic Friction, Static
Friction, Pyramid, Cards, Stack, Stack Ratio, Rod, Joint Grid, Rope, Heavy
Rope, and Net source-row packets, the Spring and Spring Ratio source packets
plus open CPU/GPU gates, the
`avbd-demo3d`
Ground source-row
py-demo, benchmark row, and packet, the `avbd-demo3d` Dynamic Friction
source-row py-demo, benchmark row, and packet, the `avbd-demo3d` Static
Friction source-row py-demo, benchmark row, and packet, the `avbd-demo3d`
Pyramid source-row py-demo, benchmark row, and packet, the `avbd-demo3d`
Rope, Heavy Rope, Spring, and Spring Ratio source-row py-demos, benchmark
rows, and packets, the `avbd-demo3d` Stack, Stack Ratio, and Soft Body source-row py-demos,
benchmark rows, and packets, the `avbd-demo3d` Breakable source-row py-demo,
benchmark row, and packet, and one-DOF
break/reset checks,
expand
articulated facade coverage beyond the new link-link,
world-link, explicit-anchor, spherical linear-only point-joint entrypoints,
same-multibody link-link/world-link fixed/spherical save/load rebuilding
including dartpy same-multibody/world-link fixed/spherical and one-DOF
design-mode rebuild checks, and
same-multibody/world-link revolute/prismatic motor save/load rebuilding
including selected non-cardinal axis-basis persistence and restored
same-multibody/world-link endpoint-shape assertions, and
same-multibody/world-link fixed/spherical/revolute/prismatic broken-state
save/load/reset persistence including explicit-anchor fixed and selected
non-cardinal one-DOF motor rows with restored effort-limit state, with the
dartpy same-multibody/world-link fixed/spherical and one-DOF binary
round-trips, direct break/skip/reset non-cardinal basis checks, fixed and
spherical reset endpoint-shape assertions, and explicit-anchor one-DOF motor
reset endpoint/axis-shape assertions now treated as current evidence,
broaden rigid-contact feature persistence beyond the current
box/sphere/cylinder/capsule/plane/mesh known/unknown shape-frame feature
identity tests, endpoint-A/B explicit-shape local-point evidence, actual
narrow-phase primitive-feature evidence, same-feature sphere/plane replay plus
sphere/mesh-face, sphere/mesh-edge, mesh-vertex replay, and
mesh-face/mesh-edge/mesh-vertex small-pose persistence, cylinder/capsule
cap/side/rim small-pose persistence, live box-box row-order evidence,
endpoint-order row-identity evidence, spanning-top and multi-top box-pile
row-persistence evidence, plus
private rigid tangent-dual projection, or
implement the first missing row from the new
[`avbd-demo-corpus.md`](104-vertex-block-descent-solver/avbd-demo-corpus.md)
matrix, while the plan/dev-task surfaces must also keep the 2D/3D source-demo
corpus, paper/video scenes, CPU/GPU benchmark packets, and performance
leadership gates explicit instead of treating the current free-rigid rows as
AVBD completion.
