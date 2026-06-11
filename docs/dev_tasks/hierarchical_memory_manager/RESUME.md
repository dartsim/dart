# Resume: Hierarchical Memory Manager

## Current Reality (2026-06-10)

PR #2956 is wrapped and should stay frozen except for PR-management fixes. The
active HMM Phase 4/5 continuation is on
`pr/hmm-phase45-follow-up-clean`, based on `origin/main` after PR #2956
landed.

The latest post-merge pipeline/scratch slices add allocator-aware construction
for `BatchedRigidBodyIntegrationStage` and allocator-aware
`WorldStepPipeline` overflow storage. Custom batched stages can now borrow a
`MemoryManager`, route force-batch and parent-before-child frame-order scratch
vectors through the provided free allocator, and release those reserves when the
stage is destroyed. The built-in World pipeline cache also routes custom-stage
overflow spillover through the World free allocator. These use existing custom
stage and frame-coupled fixtures rather than adding new production scenes.

The latest post-merge unified boxed-LCP slice routes the stage-owned rigid
contact problem, unified problem row/block containers, nested multibody block
row storage, and unified solve scratch traversal/fallback/tangent vectors
through the borrowed World free allocator. Focused allocator-contract tests now
verify those containers reserve from and release back to the provided
`MemoryManager` root, while existing unified same-shape and World boxed-LCP
no-growth/no-heap gates still pass. Local verification for this slice: focused
`test_unified_constraint`/`test_world` build and filters, `pixi run lint`,
`pixi run build`, `pixi run test-unit`, and `git diff --check`.

The latest post-merge deformable slice routes
`DeformableContactSolverScratch`'s per-body surface topology/contact-mask
storage and inter-body surface-CCD edge, sweep-item, and sweep-link buffers
through the World free allocator. This extends the earlier contact-candidate and
projected-Newton scratch routing without adding another production scene. A
second narrow deformable slice routes `DeformableVbdScratch` AVBD scalar-row
inventories, descriptor metadata vectors, static-contact feature IDs, and
friction warm-start lookup buffers through the same World free allocator while
leaving solver row arrays on their existing kernel-facing vector contracts.

The previous post-merge slice covers the rigid IPC mixed-domain path that arrived
from `origin/main`: mixed rigid/deformable surface preparation now keeps
allocator-backed BDF2 history, articulation input, mixed-domain surface payload,
candidate, edge, and AABB scratch under the stage's borrowed World free
allocator. The existing rigid IPC custom-stage allocator test now prepares a
mixed rigid/deformable surface shape under the global heap counter and verifies
zero global heap allocation while the relevant reserves grow from the provided
free-list allocator.

Latest allocator-policy work kept the free-list persistent registry policy
unchanged and narrowed frame-backed STL bake storage instead: `FrameStlAllocator`
now sends scalar arrays through the dense 32-byte frame fast path and reserves
cache-line alignment for cache-line-sized or over-aligned element pages. This
matches the one-shot bake/growth role better than cache-line-aligning every
large vector payload. Focused allocator tests passed for
`UNIT_common_frame_allocator` and `UNIT_common_stl_allocator`, and direct pinned
EnTT build/growth probes showed the 256/512/2048 DART frame-bake rows beating
foonathan medians after the dense scalar-array change. The retained-policy
strict evidence is now green in
`.benchmark_results/allocator_entt_frame_dense_retained_cpu12_reps9_merged.json`:
all 12 EnTT no-growth/build comparisons pass against foonathan/memory and
standard baselines, with zero post-prewarm DART allocator calls in no-growth
rows. The base CPU12 run had two noisy baseline rows, so
`.benchmark_results/allocator_entt_frame_dense_retained_replacements_cpu12_reps9.json`
replaces only `BM_EnttRegistryBuild_Foonathan/512` and
`BM_EnttRegistryBuild_Std/256` before the checker is re-run over the merged
artifact. Later free-list color-stride experiments moved wins between foonathan
and standard rows across CPUs and were discarded as overfitting; they are not
part of the current diff.

This follow-up slice closes a real VBD allocation gap exposed by a new
World-base/global-heap gate: enabled VBD self-contact previously only baked
self-contact candidate sweep storage when AVBD self-contact rows were enabled,
but ordinary VBD Chebyshev self-contact also builds that swept candidate set in
the step loop. The current branch moves Chebyshev history vectors into
`DeformableVbdScratch`, bakes VBD self-contact sweep/adjacency capacity for all
enabled VBD surface bodies, and adds an active 5x9 two-layer VBD Chebyshev
self-contact gate.

The latest allocator-root slice routes default deformable projected-Newton
assembly scratch through `DeformableContactSolverScratch`'s borrowed World free
allocator: sparse pattern arrays, triplet assembly, PSD edge/tet/barrier block
batches, and matrix-free block/diagonal storage now use allocator-aware vectors.
The FEM rest-shape cache and lagged friction normal/contact arrays now use the
same allocator-backed scratch component. The existing deformable stage allocator
test now expects projected-Newton vectors to reserve from the provided
allocator, and the existing baked World-base/global-heap deformable gates plus
FEM/friction activity checks still pass without adding new scenes.

The current deformable solver scratch slice routes `DeformableSolverScratch`
vectors through the World free allocator: inertial targets, iterate, gradient,
direction, candidate, previous-step, external-acceleration, and active
fixed/Dirichlet/Neumann mask storage now preserve the allocator supplied when
the scratch component is constructed. The VBD/AVBD helper contracts now accept
those allocator-backed buffers through spans or allocator-agnostic vector
templates, keeping standalone one-shot callers on ordinary vectors while
allowing baked World-stage steps to stay under the World allocator root.

The latest variational contact slice routes `VariationalContactDualState`
persistent dual vectors through the World free allocator. The variational bake
helper now receives the World allocator root, both bake-time and lazy
World-stage creation initialize the component with allocator-backed dual
storage, pre-existing/default-constructed dual-state components are rebound to
that allocator before sizing, and the automatic binary state serializer handles
allocator-aware trivial vectors without breaking the component's aggregate
state-component contract. Focused world reservation/rebuild tests and
variational contact save/load/contact tests pass.

The current continuation extends the same allocator ownership to the
`VariationalContact` property component: point-link-index and local-position
vectors now use `StlAllocator`, public ground-contact setup constructs them
with the World free allocator, and the variational World stage rebinds
loaded/pre-existing configs before baking solver scratch. The existing
contact-heavy variational gate now checks both config vectors and dual vectors
use the World allocator after bake.

The latest continuation extends that allocator ownership to
`MultibodyVariationalState`: its SE(3) previous-transform history and 6D
previous-momentum history now use allocator-aware vectors, bake-time and lazy
World-stage creation construct them from the World free allocator, loaded or
pre-existing state is rebound without losing history, and binary state
serialization accepts allocator-aware transform/momentum vectors.

The current continuation routes finite-stiffness AVBD point-joint
compliant-loop scratch through the same World allocator root. The private
variational scratch component now owns allocator-backed compliant constraints,
axis-row vectors, descriptor staging lists, and scalar-row inventories;
`enterSimulationMode()` pre-sizes/synchronizes that storage for baked point-joint
shapes; and the compliant contact hook reads the baked scratch directly instead
of copying constraints into a default-heap vector.

The latest continuation closes the sibling velocity-actuator projection
allocation path. Variational velocity-actuated revolute/prismatic joints now
contribute projection row counts during bake, the projection loop writes their
target rows directly into the reusable residual/Jacobian storage, and projection
retractions reuse existing scratch instead of building a per-step constraint
vector or returning a temporary per-joint retract vector.

The current continuation narrows variational ground-contact scratch ownership:
pure compliant contact now evaluates from baked `groundContact` plus reusable
contact-evaluation force/forcing vectors and resets the AL solver optional,
avoiding construction of the solver's default-heap dual vector. Positive
`dualUpdateCadence` configurations still construct and reuse
`VariationalGroundContactSolver` so augmented-Lagrangian dual state remains
warm-started and persisted. Focused variational ground-contact tests and World
reservation/rebuild tests cover both sides of that split.

The next follow-up slice adds a focused `WorldRegistry` rebuild-boundary gate
for contact-heavy solver-owned ECS storage. It reuses the existing compliant
variational contact slider setup, verifies baked contact scratch and registry
capacity stability during steps, clears the `World`, checks registry capacities
release to zero while the registry still uses the world free-list allocator, and
then rebuilds the same contact-heavy shape with identical baked storage
capacities and no base allocator activity in the baked step loop.

The current follow-up slice also adds a small scripted deformable boundary gate
for active Dirichlet/Neumann processing. It proves the per-body boundary masks
and external-acceleration buffers stay inside baked deformable solver scratch
with no World-base growth and no global heap allocation after prewarm.

The latest slice moves variational multibody manifold Anderson acceleration
history and least-squares temporaries into `MultibodyVariationalScratch`.
`enterSimulationMode()` pre-sizes the World-stage scratch for the baked
multibody shape, while the public one-shot helper keeps a local scratch
fallback. The existing loop-closure scratch bake test now asserts the Anderson
work storage shape, and the spherical/floating manifold Anderson regressions
still pass.

The current slice moves variational multibody articulated inverse-mass and
exact recursive-Newton solve buffers into baked `MultibodyVariationalScratch`.
The World-stage path now reuses articulated operators, spatial bias/twist
buffers, joint projectors, factored joint blocks, right-hand sides, and result
vectors across RIQN iterations and loop-closure projections; the public
one-shot helper keeps local fallback scratch.

The latest slice extends that variational multibody scratch reuse to per-step
state and residual storage. Generalized position, velocity, applied force,
next-position trial, bootstrap spatial velocities, and forced-DEL residuals
now live in baked `MultibodyVariationalScratch`; residual evaluation writes
into caller-owned storage for each RIQN/line-search trial instead of returning
a fresh vector.

The current slice routes the variational stage's initial-guess inverse-dynamics
bias query through a reusable scratch-backed overload. The public
`computeMultibodyInverseDynamics()` helper remains available for one-shot
callers, but the World-stage path now reuses baked dynamics-tree/RNEA storage
and a baked zero-acceleration vector instead of allocating temporary dynamics
work on same-shape steps.

The latest slice moves dense variational loop-closure projection work into
`MultibodyVariationalScratch`: body Jacobians, residual/Jacobian matrices,
inverse-mass transpose blocks, constraint-mass factorization, lambda RHS, lambda,
and correction storage are pre-sized at bake and reused by the World-stage
projection loop. A follow-up in the same line removes the per-projection
`buildVarTree()` rebuild: the loop now refreshes the existing step tree's
configuration-dependent transforms/Jacobians in place for each candidate and
restores the tree to the base configuration before final residual/history
evaluation. The current slice moves the initial per-step variational tree build
into baked `MultibodyVariationalScratch`, so same-shape steps reuse the tree
link vector, link-index map, per-link child lists, and link-frame subspace
matrices instead of constructing fresh containers. The follow-up gate exposed
and closed the last link-index allocation in that path: same-shape steps now
reuse the existing unordered-map nodes instead of clearing and repopulating the
map. The World baked-step global-heap gate now includes the active loop-closure
chain shape.

The current continuation keeps that variational tree scratch inside the World
allocator hierarchy. `MultibodyVariationalTreeScratch` now accepts a
`MemoryAllocator`, allocates its pimpl through that allocator, and constructs the
tree link vector, per-link child lists, and link-index map with allocator-aware
STL storage. The World bake/stage path sets it to the World free allocator
before tree construction; one-shot helpers keep the default allocator fallback.
The existing loop-closure clear/rebuild gate now asserts the baked tree scratch
uses the World allocator.

The next continuation applies the same allocator contract to the nested
`MultibodyInverseDynamicsScratch` inside `MultibodyVariationalScratch`. The
scratch now accepts a `MemoryAllocator`, allocates its pimpl through that
allocator, and constructs the private `MultibodyDynamicsScratch` vector payloads
from the same root. The variational bake/stage path sets it to the World free
allocator before reservation, and the existing loop-closure clear/rebuild gate
asserts the allocator alongside the tree scratch without adding a new scene.

The following contact-scratch continuation routes baked variational ground
contact points and the augmented-Lagrangian solver's private dual vector through
the same World free allocator. The existing compliant-contact and contact-heavy
World gates now assert those allocator bindings on the baked scratch/solver
objects rather than relying only on capacity and no-growth counters.

The latest rebuild-boundary slice reuses the existing mixed default-deformable
storage scenes as compact, production, contact-family production, and
complementary contact-family production gates for direct-sparse self-contact,
matrix-free self-contact, FEM ground-friction, static-obstacle friction, and
inter-body surface-CCD solver storage. Each gate bakes the scene, checks
same-shape steps do not grow the World base allocator or ECS storage capacities,
clears the `World` back to zero registry capacity, then rebuilds the same shape
with identical baked storage capacities under the world free-list allocator.

The current rebuild-boundary slice extends the same proof to existing AVBD/VBD
scenes without adding new scene definitions. AVBD self-contact friction, VBD
Chebyshev self-contact, and AVBD ground-friction storage now bake, keep ECS
storage capacity stable during same-shape steps, clear to zero registry
capacity, and rebuild with identical storage capacities under the same World
free-list allocator.

The latest rebuild-boundary slice extends the same proof to the existing
variational loop-closure chain. The gate asserts baked
`MultibodyVariationalScratch` tree, step, linear-solve, projection, and
Anderson dimensions for the active point loop closure, verifies same-shape
steps keep registry storage capacity stable with no World-base allocator
activity, clears the `World` to zero registry capacity, and rebuilds the same
shape with identical capacities.

The current rebuild-boundary slice extends the same proof to existing boxed-LCP
multibody contact scenes. Stacked cross-multibody fallback and multi-island
fallback shapes now bake active contact rows, step without World-base allocator
activity or ECS capacity changes, clear the `World` to zero registry capacity,
and rebuild the same shape with identical storage capacities.

The latest rebuild-boundary slice extends the same proof to the existing
kinematic IPC surface-CCD crossing scene. The gate exercises the active
`KinematicBodyStepTrace` writer, verifies same-shape steps keep registry
storage capacity stable without World-base allocator activity, clears the
`World` to zero registry capacity, and rebuilds the same trace storage shape
under the World free-list allocator.

The current rebuild-boundary slice extends the same proof to existing rigid
AVBD contact and fixed-joint scenes. It covers baked `RigidAvbdContactConfig`
storage for contact rows and generated `AvbdRigidWorldPointJointConfig` storage
for fixed-joint projection rows, then steps without World-base allocator
activity or ECS capacity changes, clears the `World` to zero registry capacity,
and rebuilds the same storage shapes.

The latest rebuild-boundary slice extends the same proof to the contact-heavy
variational dual-state setup that previously only had bake/no-growth coverage.
Six sliders with four persistent contact duals each now bake
`VariationalContactDualState` storage, step without World-base allocator
activity or ECS capacity changes, clear the `World` to zero registry capacity,
and rebuild the same dual-state capacities.

The current rebuild-boundary slice extends the same proof to the semi-implicit
one-slider multibody path. The gate covers the all-storage capacity map
populated by `reserveMultibodyDynamicsRegistryStorage()` for the active private
`MultibodyDynamicsScratch`/`PendingMultibodyVelocity` path, steps without
World-base allocator activity or ECS capacity changes, clears the `World` to
zero registry capacity, and rebuilds the same storage shape.

The latest docs slice refreshes `docs/design/hierarchical_allocator.md` from a
stale pre-implementation proposal into the durable design note for the current
experimental DART 7 `World` memory hierarchy. It now records the public
`WorldOptions` allocator knobs, `MemoryManager` ownership model, allocator
lifetime roles, EnTT registry bake/rebuild boundaries, and the direct
world-base/global-heap evidence expected before broader zero-allocation claims.

The current root-routing slice moves the opaque `WorldStorage` object itself,
the private built-in step-pipeline cache, built-in stage-owned scratch/cache
objects, the lazy collision query cache, and the optional replay controller
object onto the World free-list allocator, matching the allocator already used
by the EnTT registry and differentiable-parameter list. The focused world test
checks initial construction, built-in stage scratch construction, lazy
collision-cache construction, lazy replay-controller construction, and
`World::clear()` rebuilds through free-list live-allocation counters, and still
directly probes the `WorldStorage` pointer through
`MemoryManager::hasAllocated()` in debug builds. `World::clear()` now drops the
collision query cache so rebuild boundaries release cached collision query
capacity; replay frame payload vectors and nested stage scratch payload vectors
remain governed by the existing same-shape no-growth/no-heap gates, not by this
allocator-root ownership check.

The current nested-payload slices start with the `RigidBodyVelocityStage`
force batch: allocator-aware stage construction now gives the entity, force,
and torque vectors a `StlAllocator` backed by the World free allocator. The
focused world test constructs that stage with `World::getMemoryManager()` and
checks first `prepare()` under the heap counter to prove this specific nested
payload does not allocate from the global heap. The same follow-up line routes
`RigidBodyContactStage`'s sequential-impulse constraint vector through the
World free allocator and checks a compact contact prepare increases the World
free-list allocation count. AVBD contact scratch internals remain open because
they own separate snapshots, row scratch, and warm-start inventories.

The current kinematics-cache slice adds an allocator-aware
`WorldKinematicsGraph` constructor and has the built-in `KinematicsStage`
construct the cached graph with the World free allocator. This routes the
graph's frame entity-node lookup vector through the World hierarchy. The
focused world test constructs a graph on the stack with
`World::getMemoryManager().getFreeAllocator()`, verifies node construction
bumps the World free-list live allocation count, and verifies destruction
returns the live count to the baseline.

The current compute-graph slice adds allocator-aware `ComputeGraph`
construction and routes owned `ComputeNode` objects plus the node-name lookup
table through the supplied allocator. `WorldKinematicsGraph` now passes its
allocator into `ComputeGraph`, so the built-in kinematics cache owns those
graph allocations under the World hierarchy. A focused compute-graph test
verifies provided-allocator node storage and release on destruction.
A follow-up routes `ComputeGraph`'s edge vector and topological-order cache
through the same allocator. The read-only edge/order accessors now return span
views instead of concrete vector references, keeping those allocator-backed
containers private while preserving the internal range-iteration call sites.

The current rigid IPC scratch slice adds an allocator-aware
`RigidIpcContactStage::Scratch` constructor for the top-level runtime-body,
solver-body, surface, dynamics-term, projected-Newton result, kinematic-trace,
writeback-order, and resting-contact scratch vectors. Built-in stages already
pass the World `MemoryManager`, so those reserves now grow from the World free
allocator; the focused custom-stage test verifies reserve growth and release
against the provided free-list live allocation count.
`RigidIpcProjectedNewtonSolveScratch` now also accepts a `MemoryAllocator`, and
the stage passes its allocator into that nested solve scratch so lagged,
line-search, candidate, accepted, and best-decreasing surface work vectors
borrow the same root. A follow-up routes nested rigid IPC surface vertex and
triangle payload vectors through the same allocator by constructing runtime,
solver, result, and solver-scratch surfaces with allocator-aware payload
storage; the focused custom-stage test now covers those payload reserves and
release against the provided free-list live allocation count. The next rigid
IPC detail-solver slice makes `RigidIpcBarrierAssembly` allocator-aware for its
body-offset, active-constraint, and active-friction-constraint vectors, has
allocator-backed solve results construct that assembly from the provided root,
and keeps result assignments capacity-preserving so repeated solves do not
silently switch those vectors back to the default heap.

The current deformable stage scratch slice adds allocator-aware construction
for `DeformableDynamicsStage::Scratch` top-level barrier and snapshot vectors:
static-ground barriers, sphere/box/capsule static-obstacle barriers,
deformable surface snapshots, static rigid surface-CCD snapshots, and moving
rigid surface-CCD snapshots. The focused custom-stage test builds a compact
scene that touches each vector family against an isolated provided
`MemoryManager`, verifies reserve growth, and verifies release at stage
destruction. A follow-up in the same slice constructs new
`SurfaceContactSnapshot` entries with the same borrowed allocator, so each
snapshot's position, topology, contact-mask, and edge payload vectors now route
through the World free allocator as well. The focused custom-stage test now
requires enough free-list live allocations to cover those nested payload vectors.
The latest follow-up makes deformable `ContactCandidateSet` and
`ContactCandidateSweepScratch` allocator-aware, and constructs the World-stage
`DeformableContactSolverScratch` / `DeformableVbdScratch` candidate buffers
from the World free allocator when those ECS scratch components are created.
This covers existing self-contact and inter-body surface-contact candidate and
sweep buffers without adding another scene; focused contact-candidate
reserve/release coverage and the existing baked World-base/global-heap guards
passed locally.

The current rigid AVBD contact scratch slice adds allocator-aware construction
for `RigidBodyContactStage::AvbdScratch` and routes the private contact
snapshot vectors, row-counter scratch, solve scratch vectors, warm-start
inventories, and point-joint input vector through the borrowed allocator. The
focused custom-stage test uses the existing compact fixed-joint AVBD setup,
verifies first reserve growth against an isolated provided `MemoryManager`, and
verifies release at stage destruction.

The current allocator-correctness slice closes a debug-accounting gap in
`MemoryAllocatorDebugger`: aligned allocations now keep their requested
alignment in the live allocation record, and mismatched aligned deallocation
calls are rejected without forwarding the bad alignment to the wrapped
allocator. The focused unit test also verifies that an aligned allocation is not
released through the unaligned deallocation overload. The same slice makes the
debugger destructor honor its leak-release contract by freeing still-tracked
allocations with the recorded size/alignment after reporting them.

Current allocator debug-accounting evidence passed locally:

```bash
cmake --build build/default/cpp/Release --target UNIT_common_memory_allocator -j$(nproc)
ctest --test-dir build/default/cpp/Release \
  -R '^UNIT_common_memory_allocator$' --output-on-failure
pixi run lint
pixi run build
ctest --test-dir build/default/cpp/Release \
  -R '^UNIT_common_' --output-on-failure
```

Current complementary contact-family rebuild evidence passed locally:

```bash
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.ProductionComplementaryDefaultContactFamilyRegistryStorageRebuildsAfterClear'
pixi run lint
pixi run build
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.ProductionComplementaryDefaultContactFamilyRegistryStorageRebuildsAfterClear'
```

Current AVBD/VBD rebuild-boundary evidence passed locally:

```bash
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.AvbdSelfContactRegistryStorageRebuildsAfterClear:World.VbdChebyshevRegistryStorageRebuildsAfterClear:World.AvbdGroundFrictionRegistryStorageRebuildsAfterClear'
```

Current variational loop-closure rebuild-boundary evidence passed locally:

```bash
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.VariationalLoopClosureRegistryStorageRebuildsAfterClear'
```

Current boxed-LCP rebuild-boundary evidence passed locally:

```bash
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.BoxedLcpMultibodyRegistryStorageRebuildsAfterClear'
```

Current kinematic IPC surface-CCD rebuild-boundary evidence passed locally:

```bash
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.KinematicIpcSurfaceCcdRegistryStorageRebuildsAfterClear'
```

Current rigid AVBD rebuild-boundary evidence passed locally:

```bash
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.RigidAvbdRegistryStorageRebuildsAfterClear'
```

Current variational dual-state rebuild-boundary evidence passed locally:

```bash
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.EnterSimulationModeReservesContactHeavyVariationalDualState:World.VariationalContactDualStateRegistryStorageRebuildsAfterClear'
```

Current semi-implicit multibody rebuild-boundary evidence passed locally:

```bash
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.SemiImplicitMultibodyRegistryStorageRebuildsAfterClear'
```

Current focused evidence passed locally before the checkpoint commit:

```bash
pixi run lint
pixi run build
cmake --build build/default/cpp/Release --target test_variational_integration -j$(nproc)
ctest --test-dir build/default/cpp/Release \
  -R '^test_variational_integration$' --output-on-failure
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.EnterSimulationModeReservesRegistryStorageForMultibodySteps:World.SetMultibodyOptionsReservesVariationalStateAfterBake:World.EnterSimulationModeReservesCompliantVariationalContactScratch:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'
cmake --build build/default/cpp/Release \
  --target test_variational_integration test_world test_unified_constraint \
           test_avbd_constraint test_world_step_schedule \
           test_deformable_psd_backend -j$(nproc)
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^(test_variational_integration|test_world|test_unified_constraint|test_avbd_constraint|test_world_step_schedule|test_deformable_psd_backend)$'
```

Current rebuild-boundary evidence also passed locally:

```bash
pixi run lint
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.EnterSimulationModeReservesCompliantVariationalContactScratch:World.ContactHeavyRegistryStorageRebuildsAfterClear:World.DefaultDeformableRegistryStorageRebuildsAfterClear:World.ProductionDefaultDeformableRegistryStorageRebuildsAfterClear:World.ProductionDefaultContactFamilyRegistryStorageRebuildsAfterClear'
cmake --build build/default/cpp/Release -j$(nproc)
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^(test_world|test_unified_constraint|test_avbd_constraint|test_world_step_schedule|test_deformable_psd_backend)$'
```

Current variational Anderson scratch evidence also passed locally:

```bash
cmake --build build/default/cpp/Release --target test_variational_integration test_world -j$(nproc)
build/default/cpp/Release/bin/test_variational_integration \
  --gtest_filter='VariationalIntegration.LoopClosureConstraintScratchIsBaked:VariationalIntegration.ManifoldAndersonAcceleratesSphericalChain:VariationalIntegration.ManifoldAndersonFloatingSphericalChainStaysCorrect'
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.EnterSimulationModeReservesRegistryStorageForMultibodySteps:World.SetMultibodyOptionsReservesVariationalStateAfterBake:World.EnterSimulationModeReservesCompliantVariationalContactScratch'
pixi run lint
pixi run build
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^test_variational_integration$'
```

Current variational linear-solve scratch evidence also passed locally:

```bash
cmake --build build/default/cpp/Release --target test_variational_integration test_world -j$(nproc)
build/default/cpp/Release/bin/test_variational_integration \
  --gtest_filter='VariationalIntegration.ArticulatedInverseMassMatchesDenseSolve:VariationalIntegration.LongChainConvergesWithinDefaultBudget:VariationalIntegration.LongChainExactPreconditionerConvergesWithinBudget:VariationalIntegration.LoopClosureConstraintScratchIsBaked:VariationalIntegration.ManifoldAndersonAcceleratesSphericalChain:VariationalIntegration.ManifoldAndersonFloatingSphericalChainStaysCorrect'
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.EnterSimulationModeReservesRegistryStorageForMultibodySteps:World.SetMultibodyOptionsReservesVariationalStateAfterBake:World.EnterSimulationModeReservesCompliantVariationalContactScratch'
```

Current variational step-state scratch evidence also passed locally:

```bash
cmake --build build/default/cpp/Release --target test_variational_integration test_world -j$(nproc)
build/default/cpp/Release/bin/test_variational_integration \
  --gtest_filter='VariationalIntegration.ArticulatedInverseMassMatchesDenseSolve:VariationalIntegration.LongChainConvergesWithinDefaultBudget:VariationalIntegration.LongChainExactPreconditionerConvergesWithinBudget:VariationalIntegration.LoopClosureConstraintScratchIsBaked:VariationalIntegration.ManifoldAndersonAcceleratesSphericalChain:VariationalIntegration.ManifoldAndersonFloatingSphericalChainStaysCorrect'
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.EnterSimulationModeReservesRegistryStorageForMultibodySteps:World.SetMultibodyOptionsReservesVariationalStateAfterBake:World.EnterSimulationModeReservesCompliantVariationalContactScratch'
```

Current variational inverse-dynamics scratch evidence also passed locally:

```bash
cmake --build build/default/cpp/Release -j$(nproc)
pixi run lint
pixi run build
cmake --build build/default/cpp/Release --target test_variational_integration test_world test_unified_constraint test_avbd_constraint test_world_step_schedule test_deformable_psd_backend -j$(nproc)
ctest --test-dir build/default/cpp/Release --output-on-failure \
  -R '^(test_variational_integration|test_world|test_unified_constraint|test_avbd_constraint|test_world_step_schedule|test_deformable_psd_backend)$'
```

Current variational projection scratch evidence also passed locally:

```bash
cmake --build build/default/cpp/Release --target test_variational_integration -j$(nproc)
ctest --test-dir build/default/cpp/Release -R '^test_variational_integration$' --output-on-failure
cmake --build build/default/cpp/Release --target test_world -j$(nproc)
build/default/cpp/Release/bin/test_world \
  --gtest_filter='World.EnterSimulationModeReservesRegistryStorageForMultibodySteps:World.SetMultibodyOptionsReservesVariationalStateAfterBake:World.EnterSimulationModeReservesCompliantVariationalContactScratch:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'
```

Current variational projection tree-refresh evidence also passed locally:

```bash
cmake --build build/default/cpp/Release --target test_variational_integration -j$(nproc)
ctest --test-dir build/default/cpp/Release -R '^test_variational_integration$' --output-on-failure
```

Current variational tree scratch evidence also passed locally:

```bash
cmake --build build/default/cpp/Release --target test_variational_integration -j$(nproc)
ctest --test-dir build/default/cpp/Release -R '^test_variational_integration$' --output-on-failure
```

Immediate next step: continue selecting remaining Phase 4/5 gaps from
`README.md` on a follow-up PR branch, not on PR #2956. Push/open the follow-up
only after explicit maintainer approval.

## Current Reality (2026-06-08)

Use this folder's `README.md`, `docs/plans/dashboard.md`, and the current code as
the live status. The branch-local "Current Branch", validation-command, and PR
handoff sections below are historical session notes, not current checkout
instructions. Memory/scalability work should continue through World-owned
allocator and diagnostics contracts (`MemoryManager`, `WorldOptions`,
`WorldMemoryDiagnostics`) rather than exposing EnTT storage, allocator internals,
or backend resources on the public simulation facade.

## Current Continuation (2026-06-09)

`pr/allocator-correctness-gates` (PR #2955) is merged to `main`. Its macOS
Release allocator failure was fixed by making STL allocator copy assignment
explicit, and the replacement macOS Release job passed. PR #2956,
`pr/simulation-scratch-reuse`, is now based on `main` and is the active branch
for continuing HMM Phase 4/5 work.

Recent PR #2956 slices added baked no-growth/no-heap coverage for
multi-kinematic rigid surface CCD, mixed late-active direct-sparse plus
matrix-free deformable self-contact grids, and a mixed dense production
deformable scene combining a notched direct-sparse 13x17 grid with a
matrix-free 13x19 dense rectangular grid. The latest continuation extends the
notched/jittered 13x17 irregular mesh, the 7x17 wide and 17x7 tall rectangular
grids, and the 17x17 larger square production grid to the matrix-free
projected-Newton path, so larger square, non-square, and non-grid topology now
cover both sparse assembly and CG-block scratch. It also extends production
default-solver storage and AVBD self-contact row guards, and adds a production
rectangular inter-body deformable surface-CCD crossing for sweep/candidate
scratch plus a production static-obstacle friction patch for shared
sphere/box/capsule normal-force, normal-direction, and sparse/matrix-free
Hessian scratch. The latest continuation combines sparse static-obstacle
friction and matrix-free self-contact deformables in one baked World memory
root. The current continuation also moves variational multibody contact and
constraint scratch into a cache-only component, bakes loop-closure, hard AVBD
point-joint, compliant ground-contact, and augmented-Lagrangian ground-contact
dual capacity during `enterSimulationMode()`, and adds contact-heavy
variational contact plus loop-closure scratch no-growth gates. It also adds the
compliant variational contact solver-scratch reuse with zero duals, plus the
complementary mixed deformable production gate for matrix-free static-obstacle
friction and direct-sparse irregular self-contact under the same baked World
root. The current continuation closes the stronger compliant-contact global
heap gap for the single-prismatic World-surface path by evaluating
ground-contact force through baked solver scratch and the existing scalar
single-prismatic fast path; the baked multi-slider compliant contact row now
passes the monolithic global-heap gate. The next slice adds a mixed default
contact-family production scene that combines direct static-obstacle friction,
matrix-free self-contact friction, and inter-body surface CCD under one baked
World root, with non-vacuous activity coverage plus World-base and global-heap
no-growth gates. The latest continuation adds the complementary contact-family
production scene: matrix-free static-obstacle friction, direct-sparse irregular
self-contact, and production inter-body surface CCD under the same baked root.
The latest slice also closes the batched SoA rigid-body integration stage's
same-shape allocation surface by moving force, state, model, initial-state, and
parent-before-child frame-order containers into stage scratch and by executing
the single SoA kernel directly instead of rebuilding a one-node compute graph.
It adds a focused heap guard for a prewarmed frame-coupled parent/child rigid
body pair.
Continue from the current `README.md` Immediate Next Steps: broaden remaining
boxed-LCP/contact and deformable production no-growth coverage, and move any
newly exposed step-loop scratch to world-owned backed storage before making a
full zero-allocation claim.

## Last Session Summary

The current allocator correctness slice is active on
`feature/allocator-correctness-gates`. It adds overflow guards for
`MemoryAllocator::allocateAs`, `FrameAllocator`, `FrameStlAllocator`, and
`StlAllocator`; fixes fixed-capacity `FreeListAllocator` aligned-allocation
diagnostic peak accounting; and adds focused common allocator tests for invalid
sizes, overflow, reuse, bounded failure, live/peak allocation counters, and
debug-mode mismatch/double-free handling, plus allocator-root isolation across
independent `MemoryManager` and DART 7 `World` instances.

The first memory-manager slice has landed: DART 7 `World` owns a
`dart::common::MemoryManager`, accepts root allocator/frame-scratch options,
exposes memory diagnostics, and resets frame scratch at step boundaries. The
allocator-quality gate remains active: DART allocators must beat standard C++
allocators and every required foonathan/memory allocator baseline on
DART-relevant workloads before broad hot-loop adoption. Missing foonathan rows,
noisy rows without separated confidence intervals, and slower DART rows keep
the hierarchical-memory-manager task open.

Follow-up allocator work added alignment-aware `MemoryAllocator`/`StlAllocator`
paths for over-aligned objects and allocator-aware EnTT registries, fixed
free-list split alignment and overflow edge cases, allowed EnTT view internals
to default-construct `StlAllocator` under Clang, added `FixedPoolAllocator` for
fixed-size slot workloads, and added fixed-capacity `FreeListAllocator` policy
wiring through `MemoryManager::Options` and DART 7 `WorldOptions`.
Fixed-capacity free-list arenas can satisfy over-aligned `PoolAllocator` chunks
from reserved bytes without growing from the base allocator.

The memory-debugger correctness slice exposes structured live-byte, peak-byte,
and allocation-count queries from the allocator debug path and from the
manager-owned free-list/pool allocators. `MemoryManager::DebugDiagnostics` and
DART 7 `WorldMemoryDiagnostics` include typed borrowed allocator use, so
diagnostics cover callers that borrow `getFreeListAllocator()` or
`getPoolAllocator()` directly.

The registry/no-growth slices wire the DART 7 World's internal EnTT
registry, component storage, differentiable-parameter list, and first
World-owned ECS scratch paths through the World memory hierarchy. They add
base-allocator no-growth guards for baked kinematic IPC rigid-body, multibody
variational, and single-deformable step loops, plus inline default step-pipeline
storage. The legacy graph-backed `RigidBodyIntegrationStage` reuses stage-owned
rigid-body entity and dependency-node scratch instead of allocating those lists
per execute. `World::clear()` now recreates the internal allocator-backed
`WorldStorage`, releasing registry capacities and debug-tracked registry
allocations at rebuild boundaries while preserving the World memory hierarchy.
These are not the final global zero-allocation proof.

The global heap guard branches pre-bake the default step stage bundle and
kinematics graph cache at `enterSimulationMode()`, reuse rigid IPC kinematic
scratch storage, route rigid IPC accepted/rejected writeback through that
stage-owned entity-order scratch instead of local traversal vectors, and reuse
stage-owned per-body contact-power and stationary-flag scratch for the rigid IPC
resting-contact no-op predicate. The rigid IPC projected-Newton loop also reuses
solve-local surface buffers across line-search and sufficient-decrease
backtracking candidates, and repeated solve-internal barrier assembly and
line-search calls reuse surface-pair/triplet scratch, including the
lagged-friction barrier pass. The rigid IPC contact stage now calls the
projected-Newton solve through caller-owned result/scratch storage so per-solve
surface candidate buffers persist across steps. They add
global `operator new` guards proving baked kinematic IPC rigid-body,
box-obstacle, rigid-body resting-contact, non-cross articulated resting-contact,
and same-DOF sequential cross-articulated link-contact steps do not allocate
from the global heap. The current base-allocator no-growth guard now covers the
same contact-heavy rigid-body, non-cross articulated, and same-DOF
cross-articulated paths after contact prewarm. Mixed/different-DOF, stacked,
and coupled multi-row cross-articulated boxed-LCP fallback scenes now have World
base-allocator no-growth gates and first baked-step global heap no-allocation
gates, with unified constraint scratch primed during `enterSimulationMode()`.
Five-multibody, eight-multibody, 12-multibody, 16-multibody, 24-multibody, and
32-multibody stacked boxed-LCP fallback scenes plus disconnected multi-island
mixed rigid/articulated contact scenes now extend those gates beyond the
original small contact sets. The latest mixed stress boxed-LCP guard combines
the 32-multibody stacked fallback and stress multi-island shape under one baked
World root with 60+ initial contacts.
Public return-by-value boxed-LCP unified convenience wrappers are API
allocation-boundary surfaces; the production boxed-LCP stage uses in-place
unified assembly and solve scratch. Still-larger production contact sets and
remaining solver-owned scratch remain open.

The EnTT benchmark slice (`bench/entt-registry-allocator`, PR #2890) adds
comparative EnTT registry/component-storage rows against foonathan/memory and
standard-registry baselines. It distinguishes no-growth/prewarmed churn from
build/growth, caches component storage handles, reports DART counters, and
keeps EnTT rows opt-in. Current follow-up policy uses free-list-backed
`StlAllocator` storage for persistent no-growth churn, matching the production
`WorldRegistry` allocator role. It treats build/growth as a separate one-shot
storage-construction role: DART uses `StlAllocator` over a resettable
`FrameAllocator` bake arena, while foonathan/memory uses `memory_stack`
marker/unwind storage.
PR #2890 has merged to
`main`; keep its benchmark evidence as the baseline for future allocator-policy
loops.

The broader no-allocation slice, PR #2899, has merged to `main`. It extends the
merged #2888 guard to baked rigid-body and non-cross articulated resting-contact
scenes by
reusing collision-query/contact result storage, default rigid-body
velocity/contact stage scratch, and semi-implicit multibody
dynamics/contact/staged-velocity scratch.

The current boxed-LCP scratch slice starts reusing
`UnifiedConstraintStage`-owned assembly containers and unified problem storage
for cross-body articulated contact paths. The unified constraint assembler no
longer allocates per-step row-direction, rigid/articulated row-end, or shared
rigid-body inertia lookup containers while filling shared/cross-row coupling.
The stage path now assembles per-multibody link-contact rows through persistent
`MultibodyDynamicsScratch` instead of the public return-by-value assembler, and
cross-multibody row completion reuses the same scratch for other-link point
Jacobians and joint-space denominator work instead of allocating local
lookup/context/Jacobian temporaries. It reduces transient container churn but
does not claim the boxed-LCP solve is globally allocation-free. The Dantzig
boxed-LCP solver now accepts caller-owned reusable scratch, has a matrix/vector
overload that avoids `LcpProblem` copies for already assembled systems, and has
a same-shape no-heap regression; `UnifiedConstraintStage` owns and reuses that
unified solve scratch, which also carries island remapping/sub-problem buffers,
normal-only fallback buffers, and fallback tangent accumulators. Same-shape
no-heap coverage now includes unified island solves. The unified assembler now
reuses same-shape link-block row storage without per-row Eigen matrix-vector
temporaries; same-shape no-heap coverage now also includes mixed rigid plus
borrowed-link unified assembly. The boxed-LCP stage borrows per-multibody
contact problems from persistent `MultibodyDynamicsScratch` instead of copying
them into staging containers first. The rigid-contact assembler now also has an
in-place scratch
overload, so same-shape fallback steps reuse the stage-owned rigid contact
problem instead of building a by-value temporary each step.
`UnifiedConstraintStage::prepare()` now assembles and solves the initial
boxed-LCP contact shape at `enterSimulationMode()` without writing impulses
back to World state, so the current fallback scenes' first active step reuses
that capacity. Public multibody link-contact assembly now has reusable scratch
storage, and the in-place unified assembler can borrow that problem without
same-shape heap growth. Multi-island unified solves now reserve traversal
scratch and initialize the local row remap once per solve instead of once per
island, and no-growth guards cover a disconnected mixed rigid/articulated
contact scene. Convenience return-by-value unified problem wrappers are still
open.

The deformable scratch slice moved step-local obstacle barrier lists,
deformable surface snapshots, and static/moving rigid surface-CCD snapshots
into `DeformableDynamicsStage` scratch. `DeformableDynamicsStage::prepare()`
now primes those reusable containers plus per-body surface-contact candidate
buffers, the wider swept-AABB line-search CCD candidate envelope needed by
non-square self-contact grids, and inter-body/rigid surface-CCD sweep buffers
during `enterSimulationMode()`. The baked global-heap guard now includes a deformable
surface scene with a static rigid surface-CCD obstacle, covering snapshot
reuse, plus first-baked-step active VBD static rigid surface-CCD point crossing.
VBD topology and static-contact scratch are primed during
`enterSimulationMode()`. Scripted deformable boundary processing now reuses
per-body Dirichlet/Neumann count masks instead of allocating local per-node
vectors each step. Default projected-Newton deformable scratch now keeps its
RHS, sparse Hessian assembly, PSD block batches, sparse-pattern cache, and
solution storage on reusable per-body scratch for the covered mass-spring path;
the first-baked-step global heap guard also covers default static rigid
surface-CCD point crossing. FEM rest-shape caches are primed during
`enterSimulationMode()`, and the guard covers a one-tetrahedron FEM
projected-Newton path. A follow-up default-solver gate now adds a
multi-tetrahedron FEM block on a ground-friction barrier to cover FEM rest
shape, Hessian-block, and multi-node ground-friction storage in both the World
base-allocator and global heap baked-step guards. Compact and production mixed
storage gates combine direct-sparse self-contact, matrix-free self-contact, and
FEM ground-friction bodies in one baked default-solver World. The latest FEM
storage gate combines direct and matrix-free 4x4x4-node ground-friction blocks
under the same baked root, covering both FEM projected-Newton solver storage
families.
Projected-Newton self-contact barrier scratch is sized from bake-primed contact
candidates, and the guard covers the two-triangle no-friction self-contact
path. Surface-contact candidate and sweep buffers now get topology-scaled
bake-time reserve capacity, and the guard covers a
multi-triangle frictional self-contact patch, a 5x5 two-layer frictional
self-contact grid, a 7x7 two-layer large grid, a 9x9 two-layer production grid,
an 11x11 two-layer extended production grid, a 9x13 non-square production grid,
a 7x17 wide non-square production grid, a 17x7 tall non-square production grid,
and a 17x17 larger square production grid. The larger square, wide/tall
rectangular, and notched/jittered 13x17 irregular production grids now also
cover the matrix-free projected-Newton path. Additional still-larger or
differently shaped production-scale frictional deformable contact sets need
no-growth gates before making the full deformable
claim.
AVBD self-contact row scratch is now guarded beyond the original two-triangle
scene with 5x9 and 9x13 rectangular two-layer grid row workloads and
replay-backed self-contact/friction row activity assertions.

The latest continuation verified the boxed-LCP fallback and unified island
same-shape allocation guards, then removed the avoidable final lambda copy from
the no-scratch public unified solve wrapper by moving the vector out of its
local scratch. It also added a caller-owned `UnifiedConstraintSolution` overload
so same-shape callers can reuse both solve scratch and result storage without
heap growth. This is a convenience-wrapper allocation reduction, not a full
hot-loop claim; the return-by-value unified problem and solution convenience
wrappers remain public allocation-boundary APIs, not current production
step-loop call sites.

A follow-up continuation broadened the boxed-LCP production contact guard from
the 12-contact mixed multi-island scene to a 30-contact stress multi-island
scene with independent articulated and rigid stacks. The new scene is included
in both the baked World base-allocator no-growth gate and the global heap
no-allocation gate.

The latest continuation added a two-patch deformable self-contact friction
scene to the baked World base-allocator and global heap guards. The initial
guard exposed per-step surface-contact candidate vector growth in the
projected-Newton line-search CCD path; `DeformableDynamicsStage::prepare()` now
reserves topology-scaled surface candidate, sweep, and self-contact friction
capacity before baked steps begin.

A follow-up continuation expanded that same guard family to a 5x5 two-layer
frictional self-contact grid. The larger grid exceeded the first topology
reserve heuristic for motion-aware point-triangle and edge-edge candidates; the
bake-time surface-candidate reserve now uses a larger still-linear local-contact
heuristic.

The next continuation expanded that same guard family to a 7x7 two-layer
frictional self-contact grid. The existing bake-time candidate/friction reserve
heuristic scaled to that case without additional scratch changes; the focused
World base-allocator and global-heap no-growth filters pass.

The follow-up production-grid continuation expanded the same guard family to a
9x9 two-layer frictional self-contact grid. The existing topology-scaled
bake-time reserve heuristic still covers that larger contact surface without
additional scratch changes; the focused World base-allocator and global-heap
no-growth filters pass.

The extended production-grid continuation expanded that same guard family to an
11x11 two-layer frictional self-contact grid. It also adds a public diagnostics
assertion that the guard is non-vacuous: the scene reports one deformable body,
242 nodes, active self-contact barriers, a nonempty converged active contact
set, and positive friction dissipation. The same topology-reserved
surface-candidate and self-contact-friction scratch covers the larger grid
without additional allocator policy changes.

The same continuation also added an explicit no-heap guard for same-shape
in-place unified assembly with both rigid contacts and a borrowed multibody link
problem, extending the earlier link-only in-place assembler coverage.

The diagnostics continuation added a `dartsim_ui` Memory action seam and a
read-only editor panel for DART 7 `WorldMemoryDiagnostics`. It displays frame
scratch capacity/usage/peak/overflow/reset counters, MemoryManager free-list and
pool debug counters, and the largest ECS storage capacity rows without exposing
EnTT or backend GUI types. The seam has a focused `UNIT_dartsim_ui_MemoryActions`
test and is covered by the full `UNIT_dartsim_ui_` test filter.

## Current Branch

`pr/hmm-phase45-follow-up-clean` - active branch for post-#2956 HMM follow-up
slices, currently based directly on `origin/main`. Check `git status -sb` for
the live dirty state before resuming. Do not push or open the follow-up PR
without explicit maintainer approval.

## Immediate Next Step

Continue with evidence-first HMM follow-up work on `origin/main`: built-in
stage-owned scratch/cache object roots are now allocator-aware, and the
unified boxed-LCP rigid/problem/solve scratch containers,
rigid-body velocity force-batch payload plus rigid-body contact
sequential-impulse constraint vector, the legacy rigid-body integration
entity/dependency-node scratch vectors, plus the `WorldKinematicsGraph`
entity-node cache, `ComputeGraph` owned node/name lookup/edge/order storage,
rigid IPC
top-level scratch vectors, rigid IPC solver option/result vectors, and rigid IPC
projected-Newton solve scratch vectors,
rigid IPC nested surface vertex/triangle payload vectors, rigid IPC
projected-Newton result assembly vectors,
deformable stage top-level barrier/snapshot vectors plus nested
`SurfaceContactSnapshot` payload vectors, and the rigid AVBD contact snapshot,
row-counter scratch, solve scratch vectors, warm-start inventories, and
point-joint input vectors are the first nested stage payloads routed through
world-owned
allocator-backed storage. The next slice should probe existing no-growth/no-heap
gates or stage-local heap counters for another concrete nested payload path
before changing broader scratch layouts. Do not add new production scenes unless
profiling or a failing no-growth/no-heap gate shows a real gap not covered by
the current shipped scenes.

Keep the allocator performance gate green. The latest allocator slices keep
`FrameStlAllocator` blocks cache-line aligned without per-block cache coloring,
add allocator overflow and `construct`/`destroy` hooks to the STL adapters, keep
the non-diagnostic `PoolAllocator` hot path for release `MemoryManager` pool
allocation, remove the hardcoded realistic-row benchmark min-time, add CPU
affinity controls to the benchmark runner/checker, make auto-affinity sample
per-CPU utilization before selecting a benchmark CPU, and batch short pool,
stack, frame-bulk, fallback-stack, STL-vector, iteration, tracked-stack, and
deeply tracked pool comparative rows so the strict CV gate measures sustained
allocator work. EnTT no-growth rows now use free-list-backed world-lifetime
DART storage for reserved variable-size registry arrays. EnTT build/growth rows
model one-shot registry storage construction with DART's stateless default
C-heap STL adapter, and compare against foonathan/memory stack marker/unwind
bulk lifetime storage. `FixedPoolAllocator` has a cache-friendly stride for
medium power-of-two slots, fixing the fixed-pool cache-set conflict that
previously let foonathan/memory win `BM_Pool/256/256`; the steady-state churn
row now uses that fixed-pool path for same-size allocation churn instead of the
generic size-classed pool. `PoolAllocator` default-size requests now use a
constexpr heap-index table for the existing rounded/skewed size classes, which
removes repeated size-class arithmetic from mixed-size allocation/deallocation
hot paths. The default matrix covers foonathan/memory static fixed-storage
stacks, scoped temporary allocators, two-iteration frame allocators, raw
heap/malloc/new rows, aligned/fallback/segregator/tracked/deeply tracked adapter
rows, and EnTT no-growth/build rows mapped to DART HMM roles. A 2026-06-06
CPU-affined foonathan-plus-standard-plus-EnTT matrix passed all 94 strict
checker rows after merging focused replacement rows into the full
result JSON. After the latest policy changes, a 2026-06-07 foonathan-only
matrix plus focused strict-CV replacement rows passes all 47 DART-vs-foonathan
checks in
`.benchmark_results/allocator_comparative_current_foonathan_entt_cpu16_merged_check.json`.
The 2026-06-07/08 STL allocator continuation restored explicit
construct/destroy hooks, added a stateless `DefaultStlAllocator`, and refined
the checker so high-CV rows can pass only when DART's normal-approximation 95%
mean confidence interval is strictly below the selected baseline's interval.
The merged current foonathan broad-plus-EnTT result in
`.benchmark_results/allocator_foonathan_broad_entt_current_check.json` passes
all 47 DART-vs-foonathan checks under that rule.
Random-interleaved EnTT diagnostics later in the same branch found that the
current pool-backed no-growth row remains too close to foonathan/memory at
small entity counts, and frame-backed/default-backed probes did not robustly
close the steady-state gap. A follow-up probe kept non-overaligned
`StlAllocator` storage at natural alignment, added cache-friendly default
`PoolAllocator` strides for medium power-of-two slots, and restored the EnTT
build/growth DART row to the resettable frame-backed bake arena. The
CPU-pinned build/growth result in
`.benchmark_results/allocator_entt_build_frame_bake_current_cpuauto_probe.json`
beats foonathan/memory clearly and beats the standard registry by median. A
later focused merge replaced the pool-backed no-growth row with direct
free-list-backed persistent storage and replacement std build rows; the merged
focused result
`.benchmark_results/allocator_entt_freelist_nogrowth_frame_build_current_merged_check.json`
passes all 12 EnTT no-growth/build comparisons against both foonathan/memory
and standard baselines. The remaining mixed-size misses were `BM_MultiPool` and
`BM_Realistic` against foonathan/memory; the heap-index table reduces those
focused medians to 21.6 us vs 29.0 us and 239.8 us vs 334.3 us, respectively.
The current merged broad-plus-focused result
`.benchmark_results/allocator_comparative_lookup_table_mixed_entt_merged_check.json`
passes all 94 foonathan/memory and standard comparisons. A 2026-06-08
post-policy-change foonathan-only full matrix,
`.benchmark_results/allocator_comparative_foonathan_sustained_entt_stack_cpu13_check.json`,
passes all 47 DART-vs-foonathan comparisons after restoring cache-line
alignment for large allocator-backed STL storage and moving EnTT build/growth
to a matched one-shot storage-construction comparison:
`DefaultStlAllocator` versus foonathan/memory `memory_stack` marker/unwind. A
later continuation corrected that DART row to `StlAllocator` over a resettable
`FrameAllocator` bake arena, which is the DART allocator role that matches the
foonathan `memory_stack` marker/unwind lifetime. The focused EnTT all-baseline result,
`.benchmark_results/allocator_entt_sustained_default_stack_cpu13_check.json`,
passes all 12 EnTT comparisons against foonathan/memory and standard baselines.
The full standard-registry half remains a separate fresh 94-row gate gap after
this policy change, and current high-load CPU6/CPU16 probes are not acceptable
final no-growth timing evidence despite zero post-prewarm allocator calls. HMM
is still open for production no-growth coverage, WorldRegistry bake/build
sizing, and any future allocator baselines that map to HMM allocator roles;
keep the combined comparative gate green as allocator policy changes.

A later 2026-06-08 continuation added runner-side CPU prewarm immediately
before launching affinity-pinned benchmark binaries and added cache-line
coloring for large 64-byte-aligned `FreeListAllocator` array allocations, then
added shape-aware coloring to large 64-byte-aligned `FrameAllocator` storage.
The allocator changes target the EnTT/World registry layout cases where
consecutive page-sized component arrays can otherwise start on identical cache
sets. Persistent free-list-backed registry storage uses eight 64-byte colors
with a shifted initial phase, while one-shot frame-backed build storage uses
four 256-byte colors only for true page-sized storage arrays. A strict focused
checker run after the free-list coloring passed 11/12 EnTT foonathan/std
comparisons with zero post-prewarm allocator calls on the no-growth rows; the
remaining stable gap was
`BM_EnttRegistryBuild/512` vs foonathan at ratio 1.043. Frame allocator coloring
reduced that direct 512 build probe to roughly ratio 1.007; the remaining
adapter mismatch is addressed by using DART's frame-native `FrameStlAllocator`
in the build/growth row, matching foonathan's stack-native `std_allocator` for
one-shot arena lifetimes, and by routing small frame-backed STL allocations
directly to the inline default frame fast path. The current frame coloring keeps
2048-byte entity packed arrays compact while spreading true component/sparse
pages; focused probes showed
`BM_EnttRegistryBuild/512` beating foonathan with this layout. Do not claim a
fresh full EnTT or 94-row pass until a follow-up checker run replaces
`.benchmark_results/allocator_entt_color_after_merge_check.json` with strict
green evidence.

Treat the persistent EnTT no-growth benchmark policy as production-aligned for
the allocator role: World registry storage uses the free-list-backed
`StlAllocator`. Treat EnTT build/growth as matched benchmark evidence for
one-shot storage construction, not as a claim that production registry rebuilds
must use stack lifetime. Production integration still needs broader bake/build
sizing guidance and additional contact-heavy no-growth tests after the new
variational dual-state gate, and it must not use the existing per-step frame
allocator that resets inside `World::step()`.

Rerun the full comparative gate after allocator-policy or benchmark changes,
including EnTT rows, and treat every foonathan/memory miss as a required
optimization or coverage gap:

```bash
pixi run bm-allocator-comparative-check \
  --include-entt-registry --baseline foonathan --baseline std --verbose \
  --cpu-affinity auto
```

Rerun the focused EnTT gate on a quiet host during registry-policy loops. Do
not treat high-CV local runs as evidence; the latest successful foonathan gate
needed focused row replacements because simultaneous builds/tests in sibling
worktrees drove several full-run rows above the 10% CV limit.

```bash
pixi run bm-allocator-comparative-check --only-entt-registry \
  --baseline foonathan --baseline std --verbose --cpu-affinity auto
```

Next allocator work should broaden allocator correctness coverage, extend
no-growth tests to contact-heavy scenes and remaining solver scratch paths, and
continue optimizing allocator paths until DART beats standard C++ allocators and
every required foonathan/memory allocator baseline on required workloads. The
active zero-allocation guard work
should broaden beyond the covered rigid-body, non-cross articulated,
same-DOF sequential cross-articulated, current boxed-LCP fallback
resting-contact, current boxed-LCP multi-island mixed rigid/articulated
fallback, current active 11x11 deformable self-contact friction grid, current
active 9x13 and 7x17 deformable self-contact friction grids, current active
AVBD ground contact/friction rows, current active AVBD self-contact
normal/friction row scene plus 5x9 and 9x13 rectangular AVBD self-contact row
grids,
current active rigid AVBD contact rows, current active rigid AVBD fixed-joint
rows, current active compact and production rectangular inter-body deformable
surface-CCD crossings, and basic deformable surface-snapshot scenes, while
keeping remaining public-value
unified problem/solution wrappers and larger or differently shaped
default-solver deformable allocation surfaces explicit, before making a full
zero-dynamic-allocation claim.

## Latest Local Validation

- On 2026-06-08 after the wide deformable friction checkpoint, AVBD
  self-contact row coverage expanded from the original two-triangle scene to a
  5x9 and 9x13 rectangular two-layer grids. The grids reuse the same
  row-inventory,
  self-contact adjacency, and friction warm-start path, so the guard is an
  apple-to-apple AVBD row-scratch workload rather than another projected-Newton
  barrier solve. Focused validation passed:
  `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"`
  and
  `build/default/cpp/Release/bin/test_world --gtest_filter='World.AvbdSelfContactFrictionGridRowsAreActive:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`.
  Full local gates also passed: `pixi run lint`, `pixi run build`, and
  `pixi run test-unit`.
- On 2026-06-08 after merging the latest `origin/main`, the deformable
  self-contact friction no-growth guard expanded from the 9x13 non-square grid
  to a 7x17 wide non-square two-layer production grid. The first guard exposed
  global heap growth from motion-aware swept-AABB point-triangle and edge-edge
  candidate vectors; the bake-time topology reserve now covers that wider
  line-search CCD envelope. Focused validation passed:
  `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"`
  and
  `build/default/cpp/Release/bin/test_world --gtest_filter='World.DeformableSelfContactFrictionWideRectangularGridIsActive:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`.
  Full local gates also passed: `pixi run lint`, `pixi run build`, and
  `pixi run test-unit`.
- On 2026-06-08 after adding scratch-backed unified link impulse application,
  the successful joint-solve path now reuses caller-owned generalized-impulse
  and velocity-delta buffers instead of relying on dynamic Eigen temporaries,
  and `resolveUnifiedConstraints(..., scratch)` applies solved link impulses
  through the same scratch. Focused validation passed:
  `cmake --build build/default/cpp/Release --target test_unified_constraint --parallel "$JOBS"`
  and
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_filter='UnifiedConstraint.ReusedScratchAvoidsHeapAllocationWhenApplyingLinkImpulses:UnifiedConstraint.FallbackFrictionUpdatesCrossMultibodyOtherEnd:UnifiedConstraint.FallbackFrictionOpposesSlidingWithoutReversing:UnifiedConstraint.ResolveUsesJointSolveWhenWellPosed'`.
- On 2026-06-08 after the multi-island boxed-LCP scratch checkpoint, the
  cross-multibody fallback-friction test now verifies that caller-owned
  `UnifiedConstraintSolveScratch` avoids global heap allocation for same-shape
  fallback reuse, and the rigid fallback-friction test covers the same
  no-growth contract for rigid tangent accumulators. Focused validation passed:
  `cmake --build build/default/cpp/Release --target test_unified_constraint --parallel "$JOBS"`
  and
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_filter='UnifiedConstraint.FallbackFrictionUpdatesCrossMultibodyOtherEnd:UnifiedConstraint.FallbackFrictionOpposesSlidingWithoutReversing'`.
- On 2026-06-08 after merging the latest `origin/main`, boxed-LCP unified
  island remapping now builds its local row map once per solve instead of once
  per island and reserves island traversal scratch up front. A disconnected
  multi-island mixed rigid/articulated scene was added to the active unified
  fallback check plus the World base-allocator and global heap baked-step
  no-growth guards. Focused validation passed:
  `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"`
  and
  `build/default/cpp/Release/bin/test_world --gtest_filter='World.CrossMultibodyMultiIslandContactsUseUnifiedFallback:World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap'`.
  Full local gates also passed: `pixi run lint`, `pixi run build`,
  `pixi run test-unit`, and the strict saved 94-row allocator checker against
  foonathan/memory and standard baselines.
- On 2026-06-08 after merging the latest `origin/main`, the deformable
  self-contact friction grid guard was generalized from square grids to a 9x13
  non-square two-layer production grid. The rectangular scene is covered by the
  World base-allocator and global heap baked-step no-growth guards and asserts
  non-vacuous default projected-Newton self-contact and friction diagnostics.
  Focused validation passed:
  `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"`
  and
  `build/default/cpp/Release/bin/test_world --gtest_filter='World.DeformableSelfContactFrictionRectangularGridIsActive:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`.
- On 2026-06-08 after merging the latest `origin/main`, an active inter-body
  deformable surface-CCD crossing was added to the World base-allocator and
  global heap baked-step no-growth guards. The scene uses a fixed deformable
  surface and a moving deformable surface whose projected-Newton step is clipped
  before crossing, so the coverage is an active solver path rather than only
  snapshot plumbing. Focused validation passed:
  `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"`
  and
  `build/default/cpp/Release/bin/test_world --gtest_filter='World.DeformableInterBodySurfaceCcdCrossingIsActive:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`.
- On 2026-06-08 after merging the latest `origin/main`, the rigid AVBD contact
  continuation moved the World contact-stage AVBD snapshot, point-joint input
  list, endpoint row counters, row inventories, inertial-target copy, and solve
  rows into baked stage-owned scratch. The public return-by-value AVBD helpers
  remain allocation-boundary conveniences, while the World stage uses in-place
  scratch overloads. The new active rigid penetrating-contact scene is covered
  by World base-allocator and global heap baked-step no-growth guards; the same
  guards now cover no-contact fixed-joint rows, with non-vacuous
  velocity/position projection checks for both paths. Focused validation passed:
  `cmake --build build/default/cpp/Release --target test_world test_avbd_rigid_block test_boxed_lcp_contact -j 8`
  and
  `ctest --test-dir build/default/cpp/Release -R '^(test_world|test_avbd_rigid_block|test_boxed_lcp_contact)$' --output-on-failure`.
  Full local gates also passed: `pixi run lint`, `pixi run build`,
  `pixi run test-unit`, and the strict saved 94-row allocator checker against
  both foonathan/memory and standard baselines.
- On 2026-06-08 after merging the latest `origin/main`, the AVBD deformable
  scratch continuation added active ground contact/friction rows and an active
  two-surface self-contact scene with AVBD normal/friction rows to the World
  base-allocator and global heap no-growth guards. The implementation replaces
  per-step AVBD row-inventory maps with sorted reserved previous-record
  storage, uses the same sorted-key model for friction-projection warm starts,
  rebuilds self-contact adjacency in place, and bakes AVBD contact/friction and
  self-contact candidate/row/friction warm-start capacity during
  `enterSimulationMode()`. Validation passed:
  `cmake --build build/default/cpp/Release --target test_world test_avbd_rigid_block test_boxed_lcp_contact -j 8`,
  `./build/default/cpp/Release/bin/test_world --gtest_filter=World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths`,
  `./build/default/cpp/Release/bin/test_world --gtest_filter=World.AvbdGroundFrictionRowsAreActive`,
  and
  `ctest --test-dir build/default/cpp/Release -R '^(test_world|test_vbd_world_solver|test_avbd_rigid_block|test_boxed_lcp_contact)$' --output-on-failure`.
- On 2026-06-08 after merging the latest `origin/main`, the deformable
  no-growth guard expanded from the 9x9 production grid to an 11x11 two-layer
  frictional self-contact grid. The focused filter
  `World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.DeformableSelfContactFrictionProductionGridIsActive:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap`
  passed, including the public diagnostics check for active self-contact and
  positive friction dissipation. The saved strict allocator checker still
  passes all 94 foonathan/memory and standard comparisons:
  `.benchmark_results/allocator_comparative_lookup_table_mixed_entt_merged_check.json`.
- On 2026-06-08 after merging the latest `origin/main`, switching the EnTT
  no-growth DART benchmark row to free-list-backed world-lifetime storage, and
  adding the default `PoolAllocator` heap-index lookup table:
  `cmake --build build/default/cpp/Release --target bm_allocators_comparative UNIT_common_memory_manager UNIT_common_frame_allocator UNIT_common_free_list_allocator UNIT_common_pool_allocator UNIT_common_stl_allocator --parallel 2`
  passed, and
  `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_manager|UNIT_common_frame_allocator|UNIT_common_free_list_allocator|UNIT_common_pool_allocator|UNIT_common_stl_allocator)$' --output-on-failure --parallel 2`
  passed. The focused mixed-size node-pool run
  `.benchmark_results/allocator_mixed_pool_lookup_table_current_cpu12_probe.json`
  reduced `BM_MultiPool_DART` to 21.6 us median vs foonathan 29.0 us and
  `BM_Realistic_DART` to 239.8 us median vs foonathan 334.3 us. Merging that
  artifact with the broad current matrix and the focused EnTT free-list/frame
  evidence produced
  `.benchmark_results/allocator_comparative_lookup_table_mixed_entt_merged_check.json`;
  `python scripts/check_allocator_comparative_benchmarks.py --input .benchmark_results/allocator_comparative_lookup_table_mixed_entt_merged_check.json --include-entt-registry --baseline foonathan --baseline std --verbose`
  passed all 94 DART-vs-baseline comparisons. `pixi run lint` and
  `git diff --check` passed.
- On 2026-06-07 after merging the latest `origin/main`, allocator-policy
  probing narrowed the kept code change to `FrameStlAllocator`
  construct/destroy hooks plus cache-line aligned STL storage without
  cache-color padding. Random-interleaved EnTT diagnostics were intentionally
  treated as investigation rather than green evidence:
  `.benchmark_results/allocator_entt_frame_random_probe.json` showed the
  frame-backed no-growth probe still missing foonathan at 256/2048 entities
  (`1.024`/`1.020`) and std build rows, while
  `.benchmark_results/allocator_entt_pool_random_probe.json` showed the
  restored pool-backed no-growth policy still missing foonathan at 256/512
  (`1.003`/`1.052`). A default-backed aligned-storage probe also missed
  foonathan and std rows and was not kept. Continue optimizing EnTT
  steady-state under random interleaving before making a stronger
  foonathan-completion claim.
- On 2026-06-07 after adding STL allocator `construct`/`destroy` hooks,
  cache-line colored frame-backed STL storage, cheaper no-overflow frame resets,
  a pool-backed EnTT no-growth row, and a fixed-pool steady-state churn row:
  `cmake --build build/default/cpp/Release --target bm_allocators_comparative UNIT_common_frame_allocator UNIT_common_pool_allocator UNIT_common_stl_allocator --parallel 2`
  passed, as did
  `ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(UNIT_common_frame_allocator|UNIT_common_pool_allocator|UNIT_common_stl_allocator)$'`
  and
  `pixi run python -m pytest python/tests/unit/test_run_cpp_benchmark.py tests/test_allocator_comparative_check.py python/tests/unit/test_check_allocator_comparative_benchmarks.py`.
  The current foonathan-only comparative matrix used
  `.benchmark_results/allocator_comparative_current_foonathan_entt_cpu16_check.json`
  as the broad run, focused replacements from
  `.benchmark_results/allocator_entt_512_current_interleaved_cpu16_check.json`,
  `.benchmark_results/allocator_current_noisy_rows_cpu16_check.json`,
  `.benchmark_results/allocator_current_final_noisy_atoms_cpu16_check.json`,
  and compatible stable rows from
  `.benchmark_results/allocator_comparative_full_foonathan_std_entt_cpu16_current.json`,
  `.benchmark_results/allocator_comparative_batched_adapter_framebulk_cpu16_probe.json`,
  and
  `.benchmark_results/allocator_comparative_full_batched_pool_destroyhooks_auto_check.json`.
  The merged artifact,
  `.benchmark_results/allocator_comparative_current_foonathan_entt_cpu16_merged_check.json`,
  passed
  `python scripts/check_allocator_comparative_benchmarks.py --input .benchmark_results/allocator_comparative_current_foonathan_entt_cpu16_merged_check.json --include-entt-registry --baseline foonathan --verbose`
  with all 47 DART-vs-foonathan comparisons green. Key current ratios include
  EnTT no-growth 256/512/2048 `0.956`/`0.989`/`0.962`, EnTT build 256/512/2048
  `0.210`/`0.334`/`0.543`, steady-state `0.672` and `0.666`,
  `BM_Pool/32/64` `0.558`, `BM_FallbackStack/256` `0.426`, and
  `BM_RawNew/256` `0.049`. This is fresh foonathan evidence; re-run the
  standard-baseline half before making a fresh 94-row post-policy-change claim.
- On `feature/allocator-correctness-gates` after updating benchmark
  auto-affinity and EnTT no-growth cycle sizing:
  `cmake --build build/default/cpp/Release --target bm_allocators_comparative --parallel "$JOBS"`
  passed, as did
  `cmake --build build/default/cpp/Release --target UNIT_common_frame_allocator UNIT_common_pool_allocator --parallel "$JOBS"`,
  `ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(UNIT_common_frame_allocator|UNIT_common_pool_allocator)$'`,
  `pixi run python -m pytest python/tests/unit/test_run_cpp_benchmark.py tests/test_allocator_comparative_check.py python/tests/unit/test_check_allocator_comparative_benchmarks.py`,
  `pixi run check-api-boundary-inventory`, and `git diff --check`.
  Focused EnTT comparative attempts
  `.benchmark_results/allocator_entt_auto_affinity_cycle4_check.json` and
  `.benchmark_results/allocator_entt_dynamic_cycles_auto_check.json` are
  diagnostic failures only: sibling DART worktrees were compiling/testing at
  high load, and the strict checker rejected the EnTT rows for CVs far above
  10%. Rerun the focused EnTT gate on a quiet host before claiming a fresh
  foonathan-plus-standard pass.
- On `feature/allocator-correctness-gates` after adding the larger 5x5
  two-layer frictional deformable self-contact grid guard and increasing the
  topology-scaled surface-contact candidate reserve:
  `cmake --build build/default/cpp/Release --target test_world test_deformable_body --parallel "$JOBS"`
  passed, as did
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.BakedKinematicIpcStepsDoNotAllocateGlobalHeap:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedArticulatedContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap:World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap'`
  and
  `build/default/cpp/Release/bin/test_deformable_body --gtest_color=no --gtest_filter='DeformableBody.GroundFrictionDeceleratesSlidingNode:DeformableBody.GroundFrictionInactiveWithoutGroundContact:DeformableBody.GroundFrictionFollowsTiltedSlopeNormal:DeformableBody.FrictionDiagnosticsReportSlidingDissipation:DeformableBody.SelfContactFrictionDeceleratesSlidingSurface:DeformableBody.EdgeEdgeSelfContactFrictionDeceleratesSlidingEdge:DeformableBody.FemCubeSettlesOnGroundBarrierWithoutPenetrating'`.
- On 2026-06-07 after expanding the baked deformable self-contact friction grid
  from 3x3 to 5x5 in this branch, `cmake --build
build/default/cpp/Release --target test_world --parallel 2` passed. Focused
  validation also passed:
  `build/default/cpp/Release/bin/test_world
--gtest_filter='World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths'`
  and `build/default/cpp/Release/bin/test_world
--gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`.
- On `feature/allocator-correctness-gates` after adding the mixed rigid plus
  borrowed-link in-place unified assembler no-heap guard:
  `cmake --build build/default/cpp/Release --target test_unified_constraint --parallel "$JOBS"`
  passed, as did
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no --gtest_filter='UnifiedConstraint.InPlaceAssemblerReusesSameShapeLinkStorage:UnifiedConstraint.InPlaceAssemblerReusesSameShapeMixedStorage:UnifiedConstraint.PublicLinkScratchFeedsBorrowedUnifiedAssembly:UnifiedConstraint.ReusedScratchAvoidsHeapAllocationForSameShapeIslands:UnifiedConstraint.ReusedSolutionAvoidsHeapAllocationForSameShapeIslands'`.
  `pixi run lint`, `pixi run check-api-boundary-inventory`, and
  `git diff --check` passed after the same changes.
- On `feature/allocator-correctness-gates` after adding the multi-triangle
  frictional deformable self-contact patch guard and topology-scaled
  surface-contact candidate reserve:
  `cmake --build build/default/cpp/Release --target test_world test_deformable_body --parallel "$JOBS"`
  passed, as did
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.BakedKinematicIpcStepsDoNotAllocateGlobalHeap:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedArticulatedContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap:World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap'`
  and
  `build/default/cpp/Release/bin/test_deformable_body --gtest_color=no --gtest_filter='DeformableBody.GroundFrictionDeceleratesSlidingNode:DeformableBody.GroundFrictionInactiveWithoutGroundContact:DeformableBody.GroundFrictionFollowsTiltedSlopeNormal:DeformableBody.FrictionDiagnosticsReportSlidingDissipation:DeformableBody.SelfContactFrictionDeceleratesSlidingSurface:DeformableBody.EdgeEdgeSelfContactFrictionDeceleratesSlidingEdge:DeformableBody.FemCubeSettlesOnGroundBarrierWithoutPenetrating'`.
  `pixi run lint`, `pixi run check-api-boundary-inventory`, and
  `git diff --check` passed after the same changes.
- On `feature/allocator-correctness-gates` after moving default deformable
  projected-Newton friction buffers into `DeformableContactSolverScratch` and
  replacing static-ground box CCD footprint vectors with fixed-size stack
  storage:
  `cmake --build build/default/cpp/Release --target test_world --parallel "$JOBS"`
  and
  `cmake --build build/default/cpp/Release --target test_deformable_body --parallel "$JOBS"`
  passed, as did
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.BakedKinematicIpcStepsDoNotAllocateGlobalHeap:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedArticulatedContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap:World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap'`
  and
  `build/default/cpp/Release/bin/test_deformable_body --gtest_color=no --gtest_filter='DeformableBody.GroundFrictionDeceleratesSlidingNode:DeformableBody.GroundFrictionInactiveWithoutGroundContact:DeformableBody.GroundFrictionFollowsTiltedSlopeNormal:DeformableBody.FrictionDiagnosticsReportSlidingDissipation:DeformableBody.SelfContactFrictionDeceleratesSlidingSurface:DeformableBody.EdgeEdgeSelfContactFrictionDeceleratesSlidingEdge:DeformableBody.FemCubeSettlesOnGroundBarrierWithoutPenetrating'`.
- On `feature/allocator-correctness-gates` after the public unified solve
  wrappers started moving the no-scratch lambda result and exposing caller-owned
  solution storage:
  `cmake --build build/default/cpp/Release --target test_world test_unified_constraint --parallel "$JOBS"`
  passed, as did
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback'`
  and
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no --gtest_filter='UnifiedConstraint.ReusedScratchAvoidsHeapAllocationForSameShapeIslands:UnifiedConstraint.ReusedSolutionAvoidsHeapAllocationForSameShapeIslands:UnifiedConstraint.FallbackStopsHeadOnRigidContact:UnifiedConstraint.FallbackFrictionOpposesSlidingWithoutReversing:UnifiedConstraint.FallbackResolvesCoplanarBoxOnPlane'`.
  `pixi run lint`, `pixi run check-api-boundary-inventory`, and
  `git diff --check` passed after the same changes.
- On `feature/allocator-correctness-gates` after restoring the EnTT no-growth
  row to `FrameStlAllocator`, switching the EnTT build/growth row to a
  resettable frame-backed bake arena, batching frame-bulk/fallback/deep-tracked
  rows, and increasing tracked-stack batching for strict-CV stability:
  `cmake --build build/default/cpp/Release --target bm_allocators_comparative --parallel "$JOBS"`
  passed. A full CPU-affined matrix,
  `pixi run bm-allocator-comparative-check --include-entt-registry --baseline foonathan --baseline std --verbose --cpu-affinity 16 --benchmark-min-warmup-time 0.1 --benchmark-min-time 1.0s --repetitions 5 --output .benchmark_results/allocator_comparative_full_foonathan_std_entt_cpu16_current.json`,
  completed 94 comparisons; the broad sweep had only EnTT ordering misses and
  strict-CV noise. Focused replacement rows were run for EnTT and noisy
  allocator families, then merged in override order:
  `.benchmark_results/allocator_comparative_full_foonathan_std_entt_cpu16_current.json`,
  `.benchmark_results/allocator_comparative_noisy_rows_std_foonathan_cpu16_probe.json`,
  `.benchmark_results/allocator_comparative_batched_adapter_framebulk_cpu16_probe.json`,
  `.benchmark_results/allocator_comparative_tracked256_batched_cpu16_probe.json`,
  and
  `.benchmark_results/allocator_comparative_entt_focused_frame_nogrowth_cpu16_probe.json`.
  The final merged artifact,
  `.benchmark_results/allocator_comparative_full_foonathan_std_entt_cpu16_final_merged_check.json`,
  passed
  `pixi run python scripts/check_allocator_comparative_benchmarks.py --input .benchmark_results/allocator_comparative_full_foonathan_std_entt_cpu16_final_merged_check.json --include-entt-registry --baseline foonathan --baseline std --verbose`
  with all 94 DART-vs-baseline comparisons green. Key ratios: EnTT
  no-growth 256/512/2048 vs foonathan `0.939`/`0.948`/`0.898` and vs std
  `0.925`/`0.957`/`0.940`, EnTT build 256/512/2048 vs foonathan
  `0.230`/`0.355`/`0.534` and vs std `0.954`/`0.977`/`0.951`,
  `BM_Pool/256/256` vs foonathan/std `0.270`/`0.070`,
  `BM_FrameBulk/4096` vs foonathan/std `0.423`/`0.772`, and
  `BM_TrackedStack/256` vs foonathan/std `0.962`/`0.968`. EnTT no-growth rows
  reported zero DART frame overflow. This completes the current required
  standard-C++ and foonathan/memory comparative allocator performance gate, but
  not the broader HMM production no-growth phases.
- On `feature/allocator-correctness-gates` after adding the fixed-pool
  cache-friendly stride, removing per-allocation pointer `DoNotOptimize` calls
  from pool allocation loops, stabilizing the short allocator rows with
  per-iteration batching, and resetting variable-size benchmark RNG state per
  benchmark iteration:
  `cmake --build build/default/cpp/Release --target bm_allocators_comparative --parallel "$JOBS"`
  passed. Focused allocator probes were run with CPU affinity 16 and merged in
  override order:
  `.benchmark_results/allocator_comparative_foonathan_entt_stride32_cpu16_check.json`,
  `.benchmark_results/allocator_comparative_batched_families_stride32_cpu16_probe.json`,
  `.benchmark_results/allocator_comparative_failing_rows_stride32_cpu16_probe.json`,
  `.benchmark_results/allocator_comparative_pool_stl_tuned_stride32_cpu16_probe.json`,
  and
  `.benchmark_results/allocator_comparative_pool32_tuned_stride32_cpu16_probe.json`.
  The final merged artifact,
  `.benchmark_results/allocator_comparative_foonathan_entt_stride32_final_merged_check.json`,
  passed
  `pixi run python scripts/check_allocator_comparative_benchmarks.py --input .benchmark_results/allocator_comparative_foonathan_entt_stride32_final_merged_check.json --include-entt-registry --baseline foonathan --verbose`
  with all 47 DART-vs-foonathan comparisons green. Key formerly weak rows now
  pass with non-noisy evidence: `BM_Pool/256/256` ratio `0.286`,
  `BM_Pool/32/64` ratio `0.618`, `BM_Iteration/256` ratio `0.932`,
  `BM_TrackedStack/256` ratio `0.901`, `BM_StlVector/10000` ratio `0.235`,
  and EnTT no-growth rows at 256/512/2048 ratios `0.977`, `0.914`, and
  `0.926` with zero DART frame overflow. `pixi run lint` passed. After lint,
  `cmake --build build/default/cpp/Release --target bm_allocators_comparative UNIT_common_pool_allocator --parallel "$JOBS"`,
  `build/default/cpp/Release/bin/UNIT_common_pool_allocator`,
  `pixi run python scripts/check_allocator_comparative_benchmarks.py --input .benchmark_results/allocator_comparative_foonathan_entt_stride32_final_merged_check.json --include-entt-registry --baseline foonathan --verbose`,
  `pixi run python -m pytest tests/test_allocator_comparative_check.py python/tests/unit/test_check_allocator_comparative_benchmarks.py`,
  `pixi run check-api-boundary-inventory`, and `git diff --check` passed.
- On `feature/allocator-correctness-gates` after the strict foonathan-only
  default matrix was green, a full EnTT plus standard-baseline run,
  `pixi run bm-allocator-comparative-check --include-entt-registry --baseline foonathan --baseline std --verbose --cpu-affinity auto --output .benchmark_results/allocator_comparative_entt_std_full_probe.json`,
  completed 94 comparisons and exited nonzero. Most failures were strict-CV
  noise on the loaded host; stable misses were `BM_EnttRegistry/2048` vs
  foonathan (`1.100`), `BM_StaticStack/{256,1024,4096}` vs `std::pmr`
  (`1.004`, `1.039`, `1.051`), `BM_Temporary/1024` vs `std::pmr` (`1.108`),
  and `BM_Iteration/{1024,4096}` vs `std::pmr` (`1.044`, `1.026`). Local
  allocator-policy probes showed pool-backed and free-list-backed EnTT
  no-growth variants did not improve the full strict surface, so those probe
  edits were not kept. A long random-interleaved foonathan-only EnTT run,
  `pixi run bm-allocator-comparative-check --only-entt-registry --baseline foonathan --verbose --cpu-affinity auto --benchmark-random-interleaving --benchmark-min-warmup-time 0.1 --benchmark-min-time 5.0s --repetitions 5 --output .benchmark_results/allocator_comparative_entt_foonathan_long_probe.json`,
  showed the formerly stable `BM_EnttRegistry/2048` foonathan miss as a DART
  win (`0.961`) with zero frame overflow, but smaller EnTT rows and build rows
  were still noisy under the current load. A focused small-EnTT long run,
  `pixi run bm allocators-comparative --build-type Release --cpu-affinity auto -- --benchmark_filter='BM_EnttRegistry_(DART|Foonathan)/(256|512)' --benchmark_min_time=5.0s --benchmark_repetitions=5 --benchmark_report_aggregates_only=true --benchmark_out=.benchmark_results/allocator_comparative_entt_registry_small_long_probe.json --benchmark_out_format=json`,
  kept `BM_EnttRegistry/256` as a DART median win but still noisy, and
  `BM_EnttRegistry/512` remained noisy/slower on this host. Next optimize the
  stable `std::pmr` frame/static/temporary/iteration misses, then rerun the
  full EnTT plus standard gate on a quieter host.
- On `feature/allocator-correctness-gates` after adding
  `PoolAllocator::DiagnosticsPolicy`, switching release `MemoryManager` pool
  allocation and comparative DART pool rows to the non-diagnostic hot path,
  adding benchmark CPU-affinity controls, removing the hardcoded realistic-row
  min-time, and batching stack/tracked-stack comparative rows:
  `pixi run cmake --build build/default/cpp/Release --target UNIT_common_pool_allocator UNIT_common_memory_manager bm_allocators_comparative --parallel "$JOBS"`
  passed, and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(UNIT_common_pool_allocator|UNIT_common_memory_manager|UNIT_common_frame_allocator)$'`
  passed. `pixi run python -m pytest tests/test_allocator_comparative_check.py python/tests/unit/test_check_allocator_comparative_benchmarks.py`
  passed, `pixi run check-api-boundary-inventory` passed, and
  `git diff --check` passed. `pixi run lint` passed; after lint, the same
  targeted rebuild, focused CTest run, Python checker tests, API boundary
  inventory check, strict merged foonathan checker, and `git diff --check`
  passed again. A CPU-affined foonathan-only full matrix,
  `pixi run bm allocators-comparative --build-type Release --cpu-affinity auto -- --benchmark_filter='BM_(Pool|Stack|MultiPool|Realistic|SteadyState|FrameBulk|StlVector|StaticStack|Temporary|Iteration|RawHeap|RawMalloc|RawNew|AlignedStack|FallbackStack|Segregator|TrackedStack|DeepTrackedPool)_(DART|Foonathan)' --benchmark_min_time=1.0s --benchmark_repetitions=5 --benchmark_report_aggregates_only=true --benchmark_out=.benchmark_results/allocator_comparative_foonathan_affinity_full_probe.json --benchmark_out_format=json`,
  had DART median wins for every row and only three strict-CV noise rejections.
  Focused long reruns for the noisy stack rows and the batched tracked-stack
  focused run supplied non-noisy replacement rows; after merging those rows,
  `pixi run python scripts/check_allocator_comparative_benchmarks.py --input .benchmark_results/allocator_comparative_foonathan_affinity_merged_probe.json --baseline foonathan --verbose`
  passed all 41 DART-vs-foonathan comparisons. Key formerly weak rows now pass:
  `BM_Pool/32/64` ratio `0.922`, `BM_Pool/256/256` ratio `0.810`,
  `BM_Realistic` ratio `0.902`, `BM_Stack/256/256` ratio `0.684`,
  `BM_Stack/256/1024` ratio `0.640`, and `BM_TrackedStack/1024` ratio `0.830`.
  Full EnTT plus standard-baseline evidence is still required before HMM
  completion.
- On `feature/allocator-correctness-gates` after optimizing allocator
  fast paths and stabilizing the STL-vector adapter benchmark:
  `pixi run cmake --build build/default/cpp/Release --target bm_allocators_comparative UNIT_common_frame_allocator --parallel "$JOBS"`
  passed, and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_common_frame_allocator$'`
  passed. `pixi run lint` passed; after lint,
  `pixi run cmake --build build/default/cpp/Release --target bm_allocators_comparative UNIT_common_frame_allocator --parallel "$JOBS"`,
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_common_frame_allocator$'`,
  `pixi run python -m pytest tests/test_allocator_comparative_check.py python/tests/unit/test_check_allocator_comparative_benchmarks.py`,
  `pixi run check-api-boundary-inventory`, and `git diff --check` passed.
  Focused high-load probes showed the batched reserve-only `BM_StlVector` rows
  beating foonathan and `std::pmr` baselines; the latest foonathan-only default
  probe,
  `pixi run bm-allocator-comparative-check --baseline foonathan --verbose --benchmark-min-time 0.1s --repetitions 5 --output .benchmark_results/allocator_comparative_foonathan_default_probe.json`,
  still exited nonzero on the loaded host, but its 41 comparisons reported DART
  median wins for every non-noisy pass including `BM_StlVector/1000` (`0.598`)
  and `BM_StlVector/10000` (`0.680`). The remaining failures were CV/noise
  rejections under the strict 10% guard. A focused pool rerun,
  `pixi run bm allocators-comparative --build-type Release -- --benchmark_filter='BM_Pool_(DART|Foonathan)/(32/64|256/256)' --benchmark_min_time=0.5s --benchmark_repetitions=5 --benchmark_report_aggregates_only=true --benchmark_out=.benchmark_results/allocator_comparative_pool_probe.json --benchmark_out_format=json`,
  confirmed the noisy `BM_Pool/256/256` row as a DART median win in isolation.
  Treat this as the current best high-load evidence, not final strict-gate
  proof; a quiet-host full gate with EnTT rows is still required before HMM can
  be completed.
- On `feature/allocator-correctness-gates` after adding required comparative
  rows for foonathan/memory raw `heap_allocator`, `malloc_allocator`, and
  `new_allocator`, plus aligned, fallback, segregator, tracked, and deeply
  tracked adapter families mapped to DART frame/pool HMM rows:
  `pixi run cmake --build build/default/cpp/Release --target bm_allocators_comparative --parallel "$JOBS"`
  passed and
  `pixi run python -m pytest tests/test_allocator_comparative_check.py python/tests/unit/test_check_allocator_comparative_benchmarks.py`
  passed. After `pixi run lint`, the same benchmark target rebuild and pytest
  suite passed again; `pixi run check-api-boundary-inventory` and
  `git diff --check` also passed. A short high-load diagnostic probe,
  `pixi run bm allocators-comparative --build-type Release -- --benchmark_filter='BM_(RawHeap|RawMalloc|RawNew|AlignedStack|FallbackStack|Segregator|TrackedStack|DeepTrackedPool)_(DART|Foonathan|StdPmr)' --benchmark_min_time=0.05s --benchmark_repetitions=3 --benchmark_report_aggregates_only=true --benchmark_out=.benchmark_results/allocator_comparative_raw_adapter_probe.json --benchmark_out_format=json`,
  showed every new DART row faster than the matching foonathan row and faster
  than the matching standard row after tightening fallback/tracked scratch rows
  to fixed 32-byte allocations. Host load was still high (`Load Average`
  around 21), and some standard raw/deep-tracked rows exceeded the strict 10%
  CV gate, so this is directional evidence only; the full strict comparative
  gate still needs a quiet host.
- On `feature/allocator-correctness-gates` after adding foonathan/memory
  `static_block_allocator`, `temporary_allocator`, and `iteration_allocator<2>`
  coverage to the default comparative matrix and moving fixed 32-byte DART
  frame benchmark paths onto `FrameAllocator::allocate()`:
  `pixi run cmake --build build/default/cpp/Release --target bm_allocators_comparative --parallel "$JOBS"`
  passed before and after `pixi run lint`. The post-lint
  `pixi run python -m pytest tests/test_allocator_comparative_check.py python/tests/unit/test_check_allocator_comparative_benchmarks.py`
  run passed, `pixi run check-api-boundary-inventory` passed, and
  `git diff --check` passed. Short high-load diagnostic probes,
  `pixi run bm allocators-comparative --build-type Release -- --benchmark_filter='BM_(StaticStack|Temporary|Iteration)_(DART|Foonathan|StdPmr)' --benchmark_min_time=0.05s --benchmark_repetitions=3 --benchmark_report_aggregates_only=true --benchmark_out=.benchmark_results/allocator_comparative_scratch_family_probe.json --benchmark_out_format=json`
  and
  `pixi run bm allocators-comparative --build-type Release -- --benchmark_filter='BM_Iteration_(DART|Foonathan|StdPmr)' --benchmark_min_time=0.05s --benchmark_repetitions=3 --benchmark_report_aggregates_only=true --benchmark_out=.benchmark_results/allocator_comparative_iteration_probe.json --benchmark_out_format=json`,
  showed every new DART row faster than the matching foonathan row and faster
  than the matching `std::pmr` row, but the host load was still high
  (`Load Average` around 20-22) and several rows exceeded or nearly exceeded
  the strict 10% CV gate. Treat this as structural/directional evidence only;
  the full strict comparative gate still needs a quiet host.
- On `feature/allocator-correctness-gates` after cache-line-aligning
  frame-backed STL blocks for EnTT-style storage pages:
  `pixi run cmake --build build/default/cpp/Release --target UNIT_common_frame_allocator --parallel "$JOBS"`
  passed and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_common_frame_allocator$'`
  passed. The first attempted `pixi run test-unit common --jobs "$JOBS"` was
  stopped because it rebuilt the aggregate `tests` target instead of the
  relevant common allocator binary. A focused high-load EnTT probe,
  `pixi run bm-allocator-comparative-check --only-entt-registry --baseline foonathan --verbose --benchmark-min-time 1.0s --repetitions 7 --output .benchmark_results/allocator_comparative_entt_cacheline_probe.json`,
  failed the strict gate because all six foonathan comparisons were rejected as
  noisy under the 10% CV guard; with CV relaxed only for diagnosis, five of six
  ratios beat foonathan and the remaining noisy row was `BM_EnttRegistry/512`
  (`1.149`). An earlier shorter post-change probe showed all six EnTT ratios
  beating foonathan with CV relaxed, including `BM_EnttRegistry/256` at `0.692`
  and `BM_EnttRegistry/512` at `0.510`; the current code keeps that
  cache-line-aligned frame-backed STL policy. Treat this as evidence that the
  old 256-row miss is addressed, not as final strict-gate proof. A follow-up
  deterministic pass after applying the policy to all frame-backed STL blocks
  ran
  `pixi run cmake --build build/default/cpp/Release --target UNIT_common_frame_allocator --parallel "$JOBS"`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_common_frame_allocator$'`,
  `pixi run python -m pytest tests/test_allocator_comparative_check.py python/tests/unit/test_check_allocator_comparative_benchmarks.py`,
  and `pixi run lint-cpp` successfully. A fresh strict benchmark was not useful
  in that pass because unrelated worktree builds kept host load above 30.
- On `feature/allocator-correctness-gates` after strengthening the HMM
  completion gate to require every required foonathan/memory allocator baseline:
  `pixi run python -m pytest tests/test_allocator_comparative_check.py python/tests/unit/test_check_allocator_comparative_benchmarks.py`
  passed, `pixi run lint` passed, `git diff --check` passed, and
  `pixi run check-api-boundary-inventory` passed. A short high-load probe,
  `pixi run bm-allocator-comparative-check --include-entt-registry --baseline foonathan --baseline std --verbose --benchmark-min-time 0.2s --repetitions 3 --output .benchmark_results/allocator_comparative_hmm_scope_probe.json`,
  failed with one stable foonathan timing miss (`BM_EnttRegistry/256`, ratio
  `1.087`) plus multiple noisy rows rejected by the CV guard. Rechecking that
  saved JSON with the new required-row matrix produced the same timing/noise
  failures and no missing-row pass-through.
- On `feature/allocator-correctness-gates` after priming VBD topology/static
  contact scratch during `enterSimulationMode()` and tightening the active VBD
  static rigid surface-CCD no-heap guard to the first baked step:
  `pixi run lint` passed,
  `cmake --build build/default/cpp/Release --target test_world test_deformable_body test_vbd_world_solver -j "$JOBS"`
  passed with DART safe parallelism,
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`
  passed,
  `build/default/cpp/Release/bin/test_deformable_body --gtest_color=no --gtest_filter='DeformableBody.SurfaceContactCcdReportsCustomStageStats:DeformableBody.InterBodySurfaceContactCcdLimitsMovingPoint:DeformableBody.StaticRigidSurfaceCcdLimitsPointOnlySideCrossing:DeformableBody.KinematicSurfaceCcdSphereKeepsCurrentPoseSnapshot:DeformableBody.KinematicSurfaceCcdObstacleSweepsRealizedIpcMotion'`
  passed, and
  `build/default/cpp/Release/bin/test_vbd_world_solver --gtest_color=no --gtest_filter='VbdWorldSolver.VbdStaticRigidSurfaceCcdLimitsFastCrossing'`
  passed. `git diff --check` and `pixi run check-api-boundary-inventory` also
  passed.
- On `feature/allocator-correctness-gates` after moving default
  projected-Newton RHS/Hessian assembly/PSD batch/solution storage into
  `DeformableContactSolverScratch` and adding first-baked-step mass-spring and
  default static rigid surface-CCD no-heap guards:
  `cmake --build build/default/cpp/Release --target test_world -j "$JOBS"`
  passed with DART safe parallelism, and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`
  passed.
- On `feature/allocator-correctness-gates` after priming FEM rest-shape scratch
  during `enterSimulationMode()` and adding a first-baked-step one-tetrahedron
  FEM projected-Newton no-heap guard:
  `cmake --build build/default/cpp/Release --target test_world -j "$JOBS"`
  passed with DART safe parallelism, and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`
  passed.
- On `feature/allocator-correctness-gates` after sizing projected-Newton
  self-contact barrier scratch from bake-primed contact candidates and adding a
  first-baked-step two-triangle self-contact no-heap guard:
  `cmake --build build/default/cpp/Release --target test_world -j "$JOBS"`
  passed with DART safe parallelism, and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`
  passed.
- On `feature/allocator-correctness-gates` after moving inter-body/rigid
  surface-CCD sweep links into `DeformableContactSolverScratch`, priming them
  from `DeformableDynamicsStage::prepare()`, and adding steady-state active VBD
  static rigid surface-CCD no-heap coverage: `pixi run lint` passed,
  `cmake --build build/default/cpp/Release --target test_world test_deformable_body test_vbd_world_solver -j "$JOBS"`
  passed with DART safe parallelism,
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`
  passed,
  `build/default/cpp/Release/bin/test_deformable_body --gtest_color=no --gtest_filter='DeformableBody.SurfaceContactCcdReportsCustomStageStats:DeformableBody.InterBodySurfaceContactCcdLimitsMovingPoint:DeformableBody.StaticRigidSurfaceCcdLimitsPointOnlySideCrossing:DeformableBody.KinematicSurfaceCcdSphereKeepsCurrentPoseSnapshot:DeformableBody.KinematicSurfaceCcdObstacleSweepsRealizedIpcMotion'`
  passed, and
  `build/default/cpp/Release/bin/test_vbd_world_solver --gtest_color=no --gtest_filter='VbdWorldSolver.VbdStaticRigidSurfaceCcdLimitsFastCrossing'`
  passed.
- On `feature/allocator-correctness-gates` after moving deformable surface and
  rigid surface-CCD snapshot buffers into `DeformableDynamicsStage` scratch:
  `pixi run lint` passed,
  `cmake --build build/default/cpp/Release --target test_world test_deformable_body -j "$JOBS"`
  passed with DART safe parallelism,
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`
  passed,
  `build/default/cpp/Release/bin/test_deformable_body --gtest_color=no --gtest_filter='DeformableBody.SurfaceContactCcdReportsCustomStageStats:DeformableBody.InterBodySurfaceContactCcdLimitsMovingPoint:DeformableBody.KinematicSurfaceCcdSphereKeepsCurrentPoseSnapshot:DeformableBody.KinematicSurfaceCcdObstacleSweepsRealizedIpcMotion'`
  passed, and `git diff --check` passed.
- On `feature/allocator-correctness-gates`: `pixi run lint` passed.
- On `feature/allocator-correctness-gates` after adding public
  scratch-backed multibody link-contact assembly and the larger boxed-LCP
  stacked fallback gate: `pixi run lint` passed,
  `cmake --build build/default/cpp/Release --target test_multibody_link_contact test_unified_constraint -j "$JOBS"`
  passed with DART safe parallelism,
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no --gtest_filter='UnifiedConstraint.PublicLinkScratchFeedsBorrowedUnifiedAssembly:UnifiedConstraint.InPlaceAssemblerReusesSameShapeLinkStorage:UnifiedConstraint.ReusedScratchAvoidsHeapAllocationForSameShapeIslands'`
  passed,
  `build/default/cpp/Release/bin/test_multibody_link_contact --gtest_color=no`
  passed,
  `cmake --build build/default/cpp/Release --target test_world -j "$JOBS"`
  passed with DART safe parallelism, and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap'`
  passed.
- On `feature/allocator-correctness-gates` after the post-lint focused
  rebuild: `cmake --build build/default/cpp/Release --target test_multibody_link_contact test_unified_constraint test_unified_constraint_stage test_world -j "$JOBS"`
  passed with DART safe parallelism,
  `build/default/cpp/Release/bin/test_multibody_link_contact --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no`,
  and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Baked*DoNotAllocateGlobalHeap:World.SequentialImpulseBakeDoesNotPrewarmRigidIpcCollisionSurfaces'`
  passed.
- On `feature/allocator-correctness-gates` after adding boxed-LCP fallback
  no-growth gates, in-place rigid contact problem assembly, and bake-time
  unified constraint scratch priming:
  `cmake --build build/default/cpp/Release --target test_rigid_body_constraint test_unified_constraint test_unified_constraint_stage test_world -j "$JOBS"`
  passed with DART safe parallelism,
  `build/default/cpp/Release/bin/test_rigid_body_constraint --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no`,
  and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.CrossMultibodyLinksResolveContact:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback:World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Baked*DoNotAllocateGlobalHeap'`
  passed.
- On `feature/allocator-correctness-gates` after tightening the boxed-LCP
  fallback global heap gate to count from the first baked step:
  `cmake --build build/default/cpp/Release --target test_world test_unified_constraint_stage test_unified_constraint -j "$JOBS"`
  passed with DART safe parallelism,
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no`,
  and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.CrossMultibodyLinksResolveContact:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback:World.BakedBoxedLcpFallbackContactsDoNotGrowWorldBaseAllocator:World.BakedBoxedLcpFallbackContactStepsDoNotAllocateGlobalHeap:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Baked*DoNotAllocateGlobalHeap:World.SequentialImpulseBakeDoesNotPrewarmRigidIpcCollisionSurfaces'`
  passed.
- On `feature/allocator-correctness-gates` after adding reusable Dantzig
  boxed-LCP scratch and wiring `UnifiedConstraintStage` to own it:
  `cmake --build build/default/cpp/Release --target UNIT_math_lcp_math_lcp_dantzig_solver UNIT_math_lcp_math_lcp_dantzig_vs_ode UNIT_math_lcp_math_lcp_pivot_matrix UNIT_math_lcp_math_lcp_lcp_validation_and_solvers test_unified_constraint test_unified_constraint_stage test_world -j "$JOBS"`
  passed with DART safe parallelism.
- On `feature/allocator-correctness-gates` after the final post-lint rebuild:
  `cmake --build build/default/cpp/Release --target UNIT_math_lcp_math_lcp_dantzig_solver test_unified_constraint test_unified_constraint_stage test_world -j "$JOBS"`
  passed with DART safe parallelism, the Dantzig/unified-constraint/world
  focused executables listed below passed again, and `git diff --check` passed.
- On `feature/allocator-correctness-gates` after moving boxed-LCP
  island/fallback solve buffers into `UnifiedConstraintSolveScratch`:
  `cmake --build build/default/cpp/Release --target test_unified_constraint test_unified_constraint_stage test_world UNIT_math_lcp_math_lcp_dantzig_solver -j "$JOBS"`
  passed with DART safe parallelism.
- On `feature/allocator-correctness-gates` after adding the unified same-shape
  no-heap island-solve regression:
  `build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_dantzig_solver --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no`,
  and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.CrossMultibodyLinksResolveContact:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Baked*DoNotAllocateGlobalHeap'`
  passed.
- On `feature/allocator-correctness-gates` after making same-shape unified
  assembly reuse link-block row storage and making the boxed-LCP stage borrow
  per-multibody contact problems from persistent dynamics scratch:
  `cmake --build build/default/cpp/Release --target test_unified_constraint test_unified_constraint_stage test_world -j "$JOBS"`
  passed with DART safe parallelism.
- On `feature/allocator-correctness-gates` after adding the same-shape
  borrowed-link assembly no-heap regression:
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no`,
  and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.CrossMultibodyLinksResolveContact:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Baked*DoNotAllocateGlobalHeap'`
  passed.
- On `feature/allocator-correctness-gates` after adding reusable Dantzig
  boxed-LCP scratch:
  `build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_dantzig_solver --gtest_color=no`,
  `build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_dantzig_vs_ode --gtest_color=no`,
  `build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_pivot_matrix --gtest_color=no`,
  `build/default/cpp/Release/bin/UNIT_math_lcp_math_lcp_lcp_validation_and_solvers --gtest_color=no --gtest_filter='DantzigSolverCoverage.*:DantzigMatrixCoverage.*'`,
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no`,
  and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.CrossMultibodyLinksResolveContact:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Baked*DoNotAllocateGlobalHeap'`
  passed.
- On `feature/allocator-correctness-gates`:
  `build/default/cpp/Release/bin/UNIT_common_pool_allocator --gtest_color=no --gtest_filter='PoolAllocatorTest.DebugRejectsMismatchedAndDoubleFree'`
  passed.
- On `feature/allocator-correctness-gates`:
  `cmake --build build/default/cpp/Release --target UNIT_common_memory_allocator UNIT_common_memory_manager UNIT_common_frame_allocator UNIT_common_free_list_allocator UNIT_common_pool_allocator UNIT_common_stl_allocator test_world -j "$JOBS"`
  with DART safe parallelism.
- On `feature/allocator-correctness-gates`:
  `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_allocator|UNIT_common_memory_manager|UNIT_common_frame_allocator|UNIT_common_free_list_allocator|UNIT_common_pool_allocator|UNIT_common_stl_allocator)$' --output-on-failure -j "$JOBS"`
  passed 6/6.
- On `feature/allocator-correctness-gates`:
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.MemoryManagersAreIsolatedAcrossWorlds'`
  passed.
- On `feature/allocator-correctness-gates` after the registry rebuild-lifetime
  change:
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.MemoryDiagnosticsReportEcsStorageLayout:World.ClearReleasesRegistryStorageForRebuild:World.RegistryUsesWorldFreeAllocator:World.MemoryManagersAreIsolatedAcrossWorlds:World.ReservedRegistryStorageReusesComponentCapacity:World.EnterSimulationModeReservesRegistryStorageForKinematicIpcSteps:World.EnterSimulationModeReservesRegistryStorageForMultibodySteps:World.SetMultibodyOptionsReservesVariationalStateAfterBake:World.EnterSimulationModeReservesRegistryStorageForDeformableSteps:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths'`
  passed 10/10.
- On `feature/allocator-correctness-gates` after the registry rebuild-lifetime
  change:
  `ctest --test-dir build/default/cpp/Release -R '^test_world$' --output-on-failure -j "$JOBS"`
  passed 1/1.
- On `feature/allocator-correctness-gates` after broadening the World
  base-allocator no-growth guard to contact-heavy scenes:
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths'`
  passed.
- On `feature/allocator-correctness-gates` after removing unified-constraint
  assembler row/end/hash-map scratch:
  `cmake --build build/default/cpp/Release --target test_unified_constraint test_unified_constraint_stage test_world -j "$JOBS"`,
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no`,
  and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.CrossMultibodyLinksResolveContact:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Baked*DoNotAllocateGlobalHeap'`
  passed.
- On `feature/allocator-correctness-gates` after routing the boxed-LCP stage's
  per-multibody link-contact row assembly through reusable
  `MultibodyDynamicsScratch`: `pixi run lint`,
  `cmake --build build/default/cpp/Release --target test_unified_constraint test_unified_constraint_stage test_world UNIT_common_memory_allocator UNIT_common_memory_manager UNIT_common_frame_allocator UNIT_common_free_list_allocator UNIT_common_pool_allocator UNIT_common_stl_allocator -j "$JOBS"`,
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no`,
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.CrossMultibodyLinksResolveContact:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Baked*DoNotAllocateGlobalHeap'`,
  `cmake --build build/default/cpp/Release --target test_multibody_link_contact -j "$JOBS"`,
  `build/default/cpp/Release/bin/test_multibody_link_contact --gtest_color=no`,
  `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_allocator|UNIT_common_memory_manager|UNIT_common_frame_allocator|UNIT_common_free_list_allocator|UNIT_common_pool_allocator|UNIT_common_stl_allocator)$' --output-on-failure -j "$JOBS"`,
  and
  `ctest --test-dir build/default/cpp/Release -R '^test_world$' --output-on-failure -j "$JOBS"`
  passed.
- On `feature/allocator-correctness-gates` after routing cross-multibody row
  completion through existing per-multibody scratch instead of local lookup,
  context, and Jacobian temporaries: `pixi run lint`,
  `cmake --build build/default/cpp/Release --target test_unified_constraint test_unified_constraint_stage test_world test_multibody_link_contact -j "$JOBS"`,
  `build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no`,
  `build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no`,
  `build/default/cpp/Release/bin/test_multibody_link_contact --gtest_color=no`,
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.CrossMultibodyLinksResolveContact:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Baked*DoNotAllocateGlobalHeap'`,
  `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_allocator|UNIT_common_memory_manager|UNIT_common_frame_allocator|UNIT_common_free_list_allocator|UNIT_common_pool_allocator|UNIT_common_stl_allocator)$' --output-on-failure -j "$JOBS"`,
  and
  `ctest --test-dir build/default/cpp/Release -R '^test_world$' --output-on-failure -j "$JOBS"`
  passed.
- On `feature/allocator-correctness-gates` after the multi-manager/world
  isolation tests:
  `cmake --build build/default/cpp/Release --target UNIT_common_memory_manager test_world -j "$JOBS"`
  and
  `build/default/cpp/Release/bin/UNIT_common_memory_manager --gtest_color=no --gtest_filter='MemoryManagerTest.ManagersKeepAllocatorRootsIsolated' && build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.MemoryManagersAreIsolatedAcrossWorlds'`
  passed.
- On `feature/world-unified-constraint-scratch`:
  `cmake --build build/default/cpp/Release --target test_unified_constraint test_unified_constraint_stage test_world -j8 && build/default/cpp/Release/bin/test_unified_constraint --gtest_color=no && build/default/cpp/Release/bin/test_unified_constraint_stage --gtest_color=no && build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.ZeroDofMultibodyLinkContactStopsRigidBody:World.ZeroDofMultibodyFallbackDoesNotDoubleApplyShortcutImpulses'`
  (17/17 unified constraint tests, 2/2 unified constraint stage tests, and 2/2
  focused World zero-DOF contact tests passed).
- On `feature/world-step-global-heap-guard` after merging #2890-updated
  `origin/main`: `pixi run lint`,
  `cmake --build build/default/cpp/Release --target test_world -j8`, and
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.ReplayRestoreRebuildsCachedKinematicsAfterFrameParentRestore:World.ReplayRecordingRestoresPublicFrameState:World.StepRebuildsCachedKinematicsAfterFrameReparenting'`
- On `feature/world-step-global-heap-guard-broader` after the rigid contact
  heap guard:
  `cmake --build build/default/cpp/Release --target test_world -j2`
- On `feature/world-step-global-heap-guard-broader`:
  `./build/default/cpp/Release/bin/test_world --gtest_filter='World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap'`
- On `feature/world-step-global-heap-guard-broader`:
  `./build/default/cpp/Release/bin/test_world --gtest_filter='World.Baked*DoNotAllocateGlobalHeap:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths'`
- On `feature/world-step-global-heap-guard-broader`:
  `cmake --build build/default/cpp/Release --target test_world test_collision_world test_collision_filter_core test_world_contact_parity -j2 && ./build/default/cpp/Release/bin/test_world --gtest_filter='World.Baked*DoNotAllocateGlobalHeap:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths' && ./build/default/cpp/Release/bin/test_world_contact_parity && ./build/default/cpp/Release/bin/test_collision_world && ./build/default/cpp/Release/bin/test_collision_filter_core`
- On `feature/world-step-global-heap-guard-broader`: `pixi run lint`
- On `feature/world-step-global-heap-guard-broader` after the articulated
  contact heap guard:
  `cmake --build build/default/cpp/Release --target test_world -j2 && build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.EnterSimulationModeReservesRegistryStorageForMultibodySteps:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedArticulatedContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap'`
- On `feature/world-step-global-heap-guard-broader` after the semi-implicit
  multibody scratch update:
  `cmake --build build/default/cpp/Release --target test_multibody_constraint test_multibody_link_contact -j2 && build/default/cpp/Release/bin/test_multibody_constraint --gtest_color=no && build/default/cpp/Release/bin/test_multibody_link_contact --gtest_color=no`
- On `feature/world-step-global-heap-guard-broader` after the semi-implicit
  multibody scratch update:
  `build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.Multibody*'`
- On `feature/world-cross-contact-heap-guard` after the sequential
  cross-multibody contact heap guard:
  `cmake --build build/default/cpp/Release --target test_world -j8 && build/default/cpp/Release/bin/test_world --gtest_color=no --gtest_filter='World.CrossMultibodyLinksResolveContact:World.CrossMultibodyDifferentDofLinksUseUnifiedFallback:World.CrossMultibodyStackedContactsUseUnifiedFallback:World.CrossMultibodyCoupledRowsUseUnifiedFallback:World.BakedArticulatedContactStepsDoNotAllocateGlobalHeap:World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap:World.BakedMultibodyAndDeformableStepsDoNotAllocateGlobalHeap:World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths:World.Multibody*'`
- `pixi run lint`
- `cmake --build build/default/cpp/Release --target UNIT_common_stl_allocator -j2 && ctest --test-dir build/default/cpp/Release -R '^UNIT_common_stl_allocator$' --output-on-failure`
- `clang++ --gcc-toolchain=/usr -std=gnu++20 -I. -Ibuild/default/cpp/Release -I.pixi/envs/default/include -fsyntax-only` with an allocator-aware `entt::basic_registry` multi-component `view` instantiation.
- `cmake --build build/default/cpp/Release --target bm_allocators_comparative UNIT_common_pool_allocator -j2`
- `ctest --test-dir build/default/cpp/Release -R '^UNIT_common_pool_allocator$' --output-on-failure`
- `build/default/cpp/Release/bin/bm_allocators_comparative --benchmark_min_time=0.1s --benchmark_out=.benchmark_results/allocator-comparative-current-head.json --benchmark_out_format=json`
- Local parser over `.benchmark_results/allocator-comparative-current-head.json`:
  all DART/Foonathan and DART/StdPmr median ratios passed (`< 1.0`),
  including fixed pool, mixed pool, frame, realistic, steady-state, and STL
  vector workloads.
- `cmake --build build/default/cpp/Release --target dartsim -j2`
- `pixi run test-simulation` (61/61 passed)
- `cmake --build build/default/cpp/Release --target test_world -j2`
- `ctest --test-dir build/default/cpp/Release -R '^test_world$' --output-on-failure`
- On `feature/world-step-pipeline-inline-storage` after the inline pipeline
  storage change: `cmake --build build/default/cpp/Release --target test_world -j2`
  and `ctest --test-dir build/default/cpp/Release -R '^test_world$' --output-on-failure`
- On `feature/world-step-global-heap-guard` before this handoff:
  `cmake --build build/default/cpp/Release --target test_world -j2` and
  `ctest --test-dir build/default/cpp/Release -R '^test_world$' --output-on-failure`
- Post-lint review fix: `cmake --build build/default/cpp/Release --target UNIT_common_free_list_allocator UNIT_common_memory_manager -j2`
- Post-lint review fix: `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_free_list_allocator|UNIT_common_memory_manager)$' --output-on-failure`
- Post-lint: `cmake --build build/default/cpp/Release --target UNIT_common_memory_manager test_world -j2`
- Post-lint: `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_manager|test_world)$' --output-on-failure`
- `git diff --check`
- Current #2890 focused strict command passed on 2026-06-04 at local timestamp
  `20:20:48-07:00`:
  `pixi run bm-allocator-comparative-check --only-entt-registry --baseline foonathan --baseline std --verbose --output .benchmark_results/allocator_comparative_entt_frame_final.json`.
  All 12 EnTT comparisons passed. No-growth DART rows reported
  `dart_frame_overflow_count=0` and `dart_frame_overflow_bytes=0` after
  prewarm. DART/foonathan ratios were approximately `0.891`, `0.993`, and
  `0.961` for `BM_EnttRegistry/{256,512,2048}`; DART/std ratios were
  approximately `0.762`, `0.737`, and `0.767`. Build/growth DART rows beat both
  foonathan/memory and the standard registry at 256, 512, and 2048 entities
  while reporting configured-allocator calls per iteration of 37, 38, and 43.
- Current-head high-load rerun on head `750cd83b74c1` failed:
  `taskset -c 8-15 pixi run bm-allocator-comparative-check --only-entt-registry --baseline foonathan --baseline std --verbose --output .benchmark_results/allocator_comparative_entt_current_750cd83.json`.
  DART still reported no frame overflow, but warmed EnTT rows lost against
  foonathan/memory at 512 and 2048 entities and against Std at 256 entities;
  several baseline rows also exceeded the CV/noise limit.
- On `feature/free-list-fixed-capacity` after the fixed-capacity slice:
  `cmake --build build/default/cpp/Release --target UNIT_common_free_list_allocator UNIT_common_memory_manager test_world -j6`,
  `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_free_list_allocator|UNIT_common_memory_manager|test_world)$' --output-on-failure`,
  `pixi run lint`, and `git diff --check && git diff --cached --check`.
- On `test/memory-allocator-debugger-correctness` after borrowed-allocator
  diagnostics: `cmake --build build/default/cpp/Release --target UNIT_common_memory_manager test_world -j2`
  and `ctest --test-dir build/default/cpp/Release -R '^(UNIT_common_memory_manager|test_world)$' --output-on-failure`

## Context That Would Be Lost

- The classic `dart::simulation::World` already owns a `common::MemoryManager`;
  DART 7 `World` did not.
- `dart/common/AGENTS.md` says World owns the memory manager and components
  borrow allocators from it.
- Frame scratch should reset at the start of each simulation step, leaving the
  previous step's scratch usage visible until the next step.
- The full user goal remains broader than this slice: route simulation data
  through allocator styles, prove zero dynamic allocation in representative
  simulation loops, add memory debugging/profiling, and eventually visualize the
  hierarchy in GUI.
- The existing allocator implementations are not assumed to be good enough.
  Compare against standard C++ allocators and every foonathan/memory allocator
  baseline that maps to a required HMM allocator role. Missing rows, slower
  DART rows, or noisy rows without separated confidence intervals keep the task
  open; if DART cannot beat foonathan/memory for required DART workloads,
  record a dependency decision instead of forcing an inferior in-house
  allocator.
- Use `FixedPoolAllocator` for fixed-size node/slot workloads. Keep
  `PoolAllocator` as the size-classed small-object allocator for mixed
  workloads.
- Use fixed-capacity `FreeListAllocator` when runtime growth is prohibited by a
  precomputed memory budget; keep the default expandable policy for heap-like
  use. The policy now flows through `MemoryManager::Options` and experimental
  `WorldOptions`.
- Treat EnTT registry/storage allocation as first-class scope. The ECS storage
  layer is a dominant owner of world/component memory, but EnTT allocator types
  must remain hidden from promoted public World APIs.
- The active EnTT version's registry allocator propagates to component storage;
  the remaining Phase 3 gap is broadening bake-time reservation coverage for
  contact-heavy and solver-private scratch paths and benchmarking the
  allocator-backed EnTT path.
- The steady-state EnTT registry benchmark prewarms storage before timing. A
  standard-registry miss there points at allocator-aware registry/storage
  overhead, not one-time pool growth.
- The EnTT build/growth benchmark creates a fresh registry, reserves component
  storage, runs one churn pass, and destroys the registry inside each measured
  iteration. A miss there points at bake/build allocator and storage setup cost.
- The benchmark-only EnTT storage policies are not yet production
  `WorldRegistry` wiring. Production integration needs free-list backed
  world-lifetime storage for reserved no-growth arrays and a bake allocator that
  is reset on rebuild/destruction, not the existing per-step frame allocator
  that resets inside `World::step()`.

## How to Resume

```bash
git status -sb
git diff --stat
```

Then continue the allocator correctness branch. Before committing after any new
edits, run `pixi run lint` and rerun the six focused common allocator tests
above if lint touches C++ files.
