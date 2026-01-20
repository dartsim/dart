# ReactPhysics3D ECS and Collision Pipeline Review

## Scope and Sources (Local Snapshot)

- Repo: `/home/js/dev/physics/reactphysics3d` (commit 7000610a, 2025-01-09)
- Key files examined:
  - `include/reactphysics3d/engine/Entity.h`
  - `src/engine/EntityManager.cpp`
  - `include/reactphysics3d/components/Components.h`
  - `src/components/Components.cpp`
  - `include/reactphysics3d/components/ColliderComponents.h`
  - `src/components/ColliderComponents.cpp`
  - `include/reactphysics3d/engine/OverlappingPairs.h`
  - `include/reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h`
  - `src/systems/BroadPhaseSystem.cpp`
  - `src/systems/CollisionDetectionSystem.cpp`
  - `include/reactphysics3d/collision/broadphase/DynamicAABBTree.h`
  - `src/engine/PhysicsCommon.cpp`

## ECS Data Layout

- Entity = 32-bit id with `index + generation` (24 bits index, 8 bits generation).
- EntityManager recycles indices only when a queue of free indices is large enough
  to avoid fast reuse.
- Components base class stores components in tightly packed, aligned arrays (SoA).
- Each Components instance tracks:
  - `mMapEntityToComponentIndex` for entity -> array index
  - Disabled components at the tail for quick enable/disable
  - Manual memory management with a custom allocator
- ColliderComponents stores multiple parallel arrays:
  - Entities, shapes, transforms, broadphase IDs, collision masks, local/world
    transforms, overlapping pair list, flags, and materials.

Implication: data-oriented layout with explicit packing and cache locality, but
entity->index map lookups are still in the hot path for some operations.

## Collision Pipeline (High-Level)

- BroadPhaseSystem:
  - Dynamic AABB tree with fat AABBs (Box2D-style).
  - Tracks "moved shapes" and only recomputes overlaps for them.
  - Updates colliders by recomputing AABBs from shape + transform.
- OverlappingPairs:
  - Persistent pair set for convex and concave pairs.
  - Tracks `needToTestOverlap` for pairs to revalidate.
  - Stores `LastFrameCollisionInfo` for temporal coherence (GJK/SAT warm start).
- NarrowPhaseInfoBatch:
  - Collects per-pair data (AoS) for narrowphase dispatch.
  - Stores contact points in a fixed-size per-pair array.
- CollisionDetectionSystem:
  - Broadphase -> middle phase -> narrowphase pipeline.
  - Uses CollisionDispatch for algorithm selection (SAT, GJK, etc).

## Observed Optimizations

- Fat AABBs reduce broadphase reinsertion frequency.
- Moved-shape list minimizes broadphase pair updates.
- Disabled/sleeping components grouped for fast iteration.
- Custom allocators and containers avoid STL overhead in hot loops.
- Last-frame collision info reduces redundant computations.
- Built-in hierarchical profiler with per-system scopes.

## Observed Gaps / Risks

- No explicit multithreading in broadphase or narrowphase.
- NarrowPhaseInfoBatch is AoS; SIMD-friendly SoA layouts are not obvious.
- Pair persistence is world-step driven; no standalone batch query API.
- Determinism is not explicitly guaranteed across thread counts (not relevant yet).

## Profiling and Benchmarking Plan (Step-by-Step)

1. Build with profiling enabled:
   - CMake option: `-DRP3D_PROFILING_ENABLED=ON`
   - Results written to `rp3d_profiling_<worldName>.txt`
2. Use testbed scenes as baselines:
   - `testbed/scenes/collisiondetection`
   - `testbed/scenes/cubestack`
   - `testbed/scenes/pile`
3. Add a custom "10k objects" scene (testbed) to stress broadphase:
   - Dense cluster and sparse distribution
   - Mixed primitives (sphere/box/capsule)
4. Capture per-stage timings:
   - Broadphase update + pair generation
   - Middle phase (concave handling)
   - Narrowphase dispatch + contact creation
5. Compare with DART experimental benchmarks using identical scenes.
6. Record CPU core usage and memory footprint.

## Profiling Baselines (Sphere-Only)

Baseline runs are tracked in
`reactphysics3d_ecs_profile_results.md` with raw outputs in
`docs/dev_tasks/experimental_collision/results/`. The headless driver lives at
`rp3d_profile_driver.cpp`.

Key takeaways from the 10k sphere cases (ms/frame):

- Dense (range 10): collision detection **~36.6 ms**, broadphase **~22.9 ms**,
  narrowphase **~10.6 ms**
- Sparse (range 50): collision detection **~0.273 ms**, broadphase **~0.139 ms**,
  narrowphase **~0.080 ms**

## Takeaways for DART Experimental Collision

- SoA storage and "disabled tail" grouping are proven patterns for cache locality.
- Persistent pair cache + last-frame info is critical for temporal coherence.
- Broadphase should expose "moved objects" to minimize pair recomputation.
- Profiling hooks should be integrated early to avoid blind optimization.

## Open Questions

- How much of ReactPhysics3D's speed comes from custom allocators vs algorithmic
  choices (dynamic tree, pair caching)?
- Are there measurable wins from moving narrowphase input to SoA?
- What is the overhead of entity->index map lookups at 10k+ colliders?
