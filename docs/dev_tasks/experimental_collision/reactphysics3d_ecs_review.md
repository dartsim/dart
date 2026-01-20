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
  to avoid fast re-use.
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

## Profiling Run (Sparse 10k Spheres, Headless)

Because the testbed requires a GUI (nanogui/GLFW), profiling was run using a
small console driver linked against the profiling-enabled library.

Build (library):

```bash
cmake -S /home/js/dev/physics/reactphysics3d \
  -B /home/js/dev/physics/reactphysics3d/build_profile \
  -DCMAKE_BUILD_TYPE=Release \
  -DRP3D_PROFILING_ENABLED=ON \
  -DRP3D_COMPILE_TESTS=OFF \
  -DRP3D_COMPILE_TESTBED=OFF
cmake --build /home/js/dev/physics/reactphysics3d/build_profile --parallel
```

Driver notes:

- Compiled with `-DIS_RP3D_PROFILING_ENABLED` to match the library ABI.
- Scene: 10k dynamic spheres, radius 0.5, positions uniform in `[-50, 50]`.
- `gravity = (0,0,0)`, sleeping disabled, 2 update steps.
- Output file: `/tmp/rp3d_profiling_profile_10000.txt`.

Key timings (ms/frame, 2 frames):

- `PhysicsWorld::update` total: **10.22 ms**
- `CollisionDetectionSystem::computeCollisionDetection`: **8.11 ms**
  - `computeBroadPhase`: **7.65 ms**
  - `computeMiddlePhase`: **0.214 ms**
  - `computeNarrowPhase`: **0.248 ms**
- `BroadPhaseSystem::updateCollidersComponents`: **1.04 ms**
  - `DynamicAABBTree::updateObject`: **0.231 ms** (20k calls)
  - `SphereShape::computeAABB`: **0.172 ms** (20k calls)
- `DynamicAABBTree::reportAllShapesOverlappingWithShapes`: **7.38 ms**

Observations:

- Broadphase dominates for sparse 10k scenes; narrowphase is small at this density.
- AABB update cost is measurable but secondary to broadphase pair generation.
- Significant "Unaccounted" time suggests missing profiler scopes in some paths.

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
