# Hierarchical Allocator Design: World-Level Memory Management

## Status: Implemented For Experimental DART 7 World

The experimental `dart::simulation::World` owns one CPU memory hierarchy. A
`World` constructs a `dart::common::MemoryManager` from `WorldOptions`, and its
internal registry, built-in stages, and solver scratch borrow allocators from
that root.

This note describes the DART 7 experimental simulation stack. The classic
`dart::dynamics::Skeleton` and legacy constraint-solver APIs are not fully
migrated by this design note, and they should not be treated as evidence for the
DART 7 no-allocation simulation-loop contract.

The hierarchical-memory-manager dev task is complete. This design note is the
durable owner for the World memory hierarchy, its no-allocation simulation-loop
contract, the current evidence surfaces, and the bounded rules for any future
allocator follow-up work. Remaining DART 7 closure work is tracked by
[`../plans/122-simulation-loop-allocation-hardening.md`](../plans/122-simulation-loop-allocation-hardening.md)
and its coverage matrix; do not reopen the completed dev-task folder for those
follow-ups.

## Architecture

`WorldOptions` carries the public allocator knobs:

- `baseAllocator`: optional borrowed root allocator. If null, the default DART
  allocator is used.
- `freeListInitialAllocation`: initial bytes reserved for world-lifetime
  heap-like storage.
- `freeListGrowthPolicy`: expandable by default, or fixed-capacity for worlds
  whose bake/build size should bound runtime allocation.
- `frameScratchInitialCapacity`: initial per-step frame arena size.

`World` converts those knobs into `MemoryManager::Options` and owns:

- the base allocator reference;
- a `FreeListAllocator` for persistent, variable-size world storage;
- a `PoolAllocator` for small-object pool roles;
- a `FrameAllocator` for resettable per-step scratch;
- debug wrappers and diagnostics for allocator traffic when enabled.

The public facade exposes this through `World::getMemoryManager()` and
`World::getMemoryDiagnostics()`. Diagnostics report allocator debug counters,
frame-scratch capacity/usage/overflow/reset counts, and aggregate ECS registry
storage layout counters without exposing EnTT storage types or solver-private
component IDs. A detailed type-erased storage row distinguishes live
components, materialized packed slots, reserved capacity, holes, live packed
regions, and sparse entity-index extent. A summary row instead uses the
type-erased storage size, which may include tombstones. Capacity and extent are
slot/index-space values, not payload bytes; packed-region fields are layout
observations rather than cache or timing measurements. `storageId` is an
internal diagnostic token for grouping a snapshot and is not a stable public
component identifier.

The accompanying human-readable role is best-effort presentation metadata; it
does not expose an EnTT type and may change when internal storage ownership
changes.

`World` remains the public facade and allocator owner, but it does not own the
diagnostic bookkeeping directly. The branch-neutral public value types live in
`dart/simulation/memory_diagnostics.hpp`; an internal
`detail::MemoryDiagnosticsTracker` lives with the ECS registry in opaque
`WorldStorage`. The tracker owns frame-scratch peak/reset state and performs the
type-erased per-storage summary when `World::getMemoryDiagnostics()` is called.
The default summary walks materialized storages and scans the entity storage to
count live entities, but does not scan packed component slots. Interactive
tools that need hole/live-region detail opt in with
`WorldMemoryDiagnosticsOptions::includeStorageLayoutDetails`. This keeps the
per-component linear packed-slot scan out of ordinary editor frames and keeps
presentation labels out of the already broad `World` implementation without
moving the allocator hierarchy itself.

## Allocation Roles

| Role                                | Lifetime                            | Allocator                                               | Rule                                                                                        |
| ----------------------------------- | ----------------------------------- | ------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| World objects and component storage | World or rebuild boundary           | Free-list allocator                                     | Reserve/bake before steady-state stepping; release on `World::clear()`.                     |
| Solver-owned persistent scratch     | World or rebuild boundary           | Free-list allocator through registry/storage components | Size from the baked shape, keep capacity stable across same-shape steps.                    |
| Per-step transient scratch          | Current `World::step()`             | Frame allocator                                         | Borrow only for data that cannot survive the frame-scratch reset at the next step boundary. |
| Fixed-size slots                    | Explicit fixed-slot workloads       | Fixed pool allocator                                    | Use only when the workload is genuinely fixed-size.                                         |
| Mixed small objects                 | Size-classed small-object workloads | Pool allocator                                          | Keep diagnostics optional so hot paths can use the non-diagnostic fast path.                |

`World::step()` resets frame scratch at the step boundary. Any storage that must
survive across iterations, contacts, solver warm starts, bake/rebuild, or
diagnostic inspection must not live in frame scratch. The first production
consumer is the built-in boxed-LCP rigid-contact step: bake warms the frame
arena for the resolved contact shape, and each step borrows the resettable arena
for the one-shot Delassus, Jacobian, impulse, and Dantzig work arrays. The
diagnostic `BoxedLcpContactSnapshot` and public `solveBoxedLcpContacts()` helper
keep caller-owned/persistent storage because their results outlive the
step-local solve. Return-by-value facade queries are treated the same way:
their output must survive the call, so they use caller-owned output containers,
world free-list cached storage, or ordinary value storage rather than frame
scratch.

## Registry And Bake Boundaries

The DART 7 `World` registry is an allocator-aware EnTT registry:

```cpp
using WorldRegistryAllocator = dart::common::StlAllocator<entt::entity>;
using WorldRegistry
    = entt::basic_registry<entt::entity, WorldRegistryAllocator>;
```

`WorldStorage` itself, the private built-in step-pipeline cache, the built-in
stage-owned scratch/cache objects inside that pipeline cache, the lazy collision
query cache, and the optional replay controller object are allocated from the
World free-list allocator. Standalone custom stage instances keep their heap
fallback constructors unless a caller provides a `MemoryManager`.
`WorldStorage` constructs the registry and differentiable-parameter list with a
`StlAllocator` that borrows `World::getMemoryManager().getFreeAllocator()`.
`World::enterSimulationMode()` runs the bake path that materializes queried
component storages, reserves existing storages, and asks domain-specific reserve
helpers to pre-size private ECS scratch for the current shape. Repeated
same-shape steps should not materialize new registry storages or grow existing
storage capacity. The rigid-body velocity stage's force-batch payload vectors,
the rigid-body contact stage's sequential-impulse constraint vector, and the
contact stage's private AVBD contact snapshot, row-counter scratch, solve
scratch, warm-start inventories, and point-joint input vector also borrow the
World free allocator when the built-in pipeline constructs those stages.
Boxed-LCP contact is the exception by lifetime: its persistent contact-query
cache stays under the World free allocator, but its dense per-step solve arrays
borrow the frame allocator after bake has warmed the arena for the current
contact shape. The rigid IPC
contact stage similarly routes its top-level runtime, solver, surface,
writeback, resting-contact, dynamics-term, projected-Newton result, and solver
scratch vectors through the World free allocator when the stage constructs the
solver scratch, including nested rigid IPC surface vertex/triangle payloads and
projected-Newton result assembly vectors.
`DeformableDynamicsStage` routes its stage-owned ground-barrier,
static-obstacle, deformable-surface snapshot, static rigid surface-CCD snapshot,
and moving rigid surface-CCD snapshot vectors through the same World free
allocator, including each snapshot's position, topology, contact-mask, and edge
payload vectors. `WorldKinematicsGraph` also uses the World free allocator for
its frame-entity-to-node cache and for `ComputeGraph`'s owned node objects,
name-lookup table, dependency-edge storage, and topological-order cache when
constructed by the built-in kinematics stage. The public read-only
`ComputeGraph` accessors expose spans instead of concrete vector references so
the graph can keep those allocator-backed containers private. Other nested
`std::vector`/Eigen payload capacity inside stage scratch objects is still
governed by the same-shape world-base and global heap no-growth gates.

`World::clear()` recreates `WorldStorage` and the built-in step-pipeline cache
with the same world free allocator, drops the lazy collision query cache, and
clears replay frames while keeping the replay controller under the same root if
it was materialized. This intentionally releases registry/component capacities
and private cached query state at the rebuild boundary while preserving the
`World` memory hierarchy and allocator policy. Rebuild-boundary tests should
therefore prove both sides of the contract: same-shape baked steps keep
capacity stable, and clear/rebuild returns to the same baked capacity shape
from zero registry capacity.

## Simulation Loop Contract

The north-star invariant is:

1. `enterSimulationMode()` and stage `prepare()` may allocate or reserve storage
   required by the current world shape.
2. Same-shape baked `World::step()` loops must not grow from the world base
   allocator.
3. Representative baked loops should also pass global heap no-allocation guards
   once third-party and public convenience API boundaries are excluded.
4. Final closure evidence for a DART 7 path starts measurement on the first
   `World::step()` after bake. A test that runs unmeasured warm-up simulation
   steps before enabling the allocation counter is useful steady-state evidence,
   but it is not enough to close the path under the no-allocation-after-bake
   requirement.
5. Public return-by-value helper APIs may remain allocation-boundary
   conveniences, but built-in `World` stages must use caller-owned or
   stage-owned reusable scratch.

Tests should measure allocator-call counters, storage capacities, and global
heap guards directly. Benchmark timing alone is not enough evidence for the
no-allocation contract.

## Completion Evidence

The completed HMM work wires the DART 7 `World` registry, built-in stage
storage, solver scratch, diagnostics, replay buffers, compute-graph storage,
and representative rigid/deformable/multibody/differentiable hot paths through
the World-owned memory hierarchy. The closing raw-malloc slice added or
revalidated focused baked-step gates for:

- rigid IPC active barriers, stacked boxes, deformable surface obstacles,
  kinematic conveyor/turntable scenes, small fixed/revolute articulation rows,
  and large equality KKT rows above the stack solve cap;
- multibody velocity-actuated steps;
- scripted deformable boundary steps, an 81-DOF retained dense FEM
  ground-friction block, and default above-dense FEM rows routed to sparse CG;
- differentiable contact-free derivative capture when `DART_BUILD_DIFF=ON`;
- existing dynamic rigid IPC world-base no-growth and global-heap guards.

The durable rule from those gates is narrower than "all code never allocates":
same-shape baked `World::step()` paths should reuse World-owned, frame, stack,
or stage-owned scratch after prepare/bake. Public return-by-value diagnostics,
standalone helper overloads, and third-party internals may allocate unless a
caller-owned or stage-owned reuse path is part of the built-in World loop.

## Evidence Surfaces

The current implementation is exercised by focused allocator tests, comparative
benchmark checkers, `World` memory diagnostics tests, registry rebuild-boundary
tests, and baked-step no-growth/no-heap gates. The opt-in detailed diagnostics
also snapshot each real free-list/frame backing allocation as a complete
region-relative partition and refine known allocated ranges with exact typed ECS
page overlays. Collection stays below the World facade and adds no allocation
tagging to the normal hot path. It exposes virtual layout and fragmentation,
not access frequency or cache misses; the full evidence contract lives in
[`memory_layout_diagnostics.md`](memory_layout_diagnostics.md).

Prefer evidence in code and verifiers over large copied scene inventories in
this design doc:

- allocator contracts: `tests/unit/common/`;
- allocator comparative checks: `tests/benchmark/common/` and
  `tests/test_allocator_comparative_check.py`;
- World memory hierarchy and diagnostics: `dart/simulation/world.hpp` and
  `dart/simulation/world.cpp`;
- registry/no-growth gates and boxed-LCP frame-scratch production coverage:
  `tests/unit/simulation/world/test_world.cpp`;
- differentiable smooth-Jacobian allocation and correctness coverage:
  `tests/unit/simulation/diff/test_diff_smooth_jacobian.cpp`.

Before claiming a broader zero-allocation result, the evidence must show that
the relevant built-in stage paths have world-base no-growth, global-heap
no-allocation, and raw malloc-family no-allocation gates after bake. The
PLAN-122 coverage matrix owns row-by-row closure and marks prewarm-dependent
raw-malloc tests as steady-state evidence until bake sizes the needed scratch.

## Remaining Design Constraints

- Keep EnTT storage types, solver-private components, and backend resources out
  of the public memory facade.
- Prefer allocator policies that match the workload role. For example,
  free-list storage is the production match for reserved variable-size
  registry arrays, while frame/stack storage is only appropriate for
  resettable one-shot build or per-step lifetimes.
- Treat missing, noisy, or slower allocator comparisons as incomplete evidence,
  not as a pass.
- Keep fixed-capacity free-list policy available for worlds whose build/bake
  budget should make runtime growth a hard failure.
- Do not route GPU memory management or third-party internal allocations through
  this CPU hierarchy without a separate design.
- Future allocator work starts from the PLAN-122 coverage matrix, a fresh
  failing gate, or a benchmark policy change. Known non-claims include
  public return-by-value diagnostics.

## Non-Goals

- Retrofitting every classic DART 6-style `Skeleton` or legacy constraint
  allocation path in this design note.
- Exposing raw EnTT storage, component IDs, or solver scratch layouts in public
  API.
- Replacing Eigen or third-party library internals globally.
- Claiming every experimental solver path is zero-allocation before direct
  gates prove that path.
