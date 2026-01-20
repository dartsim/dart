# ReactPhysics3D ECS Investigation Plan

Status: in progress
Last updated: 2026-01-19

## Purpose

Evaluate ReactPhysics3D's ECS data layout and collision pipeline to determine
whether its approach is optimized for 10k+ objects, and to extract actionable
ideas for DART's experimental collision backend.

## Key questions

- How is data laid out for transforms, colliders, and shapes (SoA vs AoS)?
- Which broadphase algorithm is used, and how are updates handled?
- How much time is spent in broadphase vs narrowphase for 1k/10k objects?
- Is pair caching used for temporal coherence and stability?

## Initial findings (code review)

These notes are from the local ReactPhysics3D checkout at
`/home/js/dev/physics/reactphysics3d`.

- Components are stored in SoA-style arrays with entity-to-index maps.
  Disabled (sleeping/inactive) components are stored at the end of arrays.
- `ColliderComponents` keeps arrays for collider pointers, broadphase ids,
  transforms, collision shapes, category bits, and per-collider overlap lists.
- Broadphase uses a dynamic AABB tree with fat AABB inflation and reinserts
  objects when they move outside their fat AABB.
- Collision detection pipeline has distinct steps:
  broadphase -> middle phase -> narrowphase, with per-stage profiling hooks.
- Overlapping pairs are stored and reused with last-frame collision info
  (GJK/SAT state), enabling temporal coherence.

These observations confirm the ECS + SoA approach and a dynamic AABB tree
strategy, but do not yet prove performance characteristics.

## Step-by-step profiling plan

1. **Build with profiling enabled**
   - Configure ReactPhysics3D with `IS_RP3D_PROFILING_ENABLED` enabled.
   - Ensure Release build to avoid debug overhead.

2. **Create deterministic scenarios**
   - Dense: regular grid with overlap (1k, 10k).
   - Sparse: randomized distribution with minimal overlaps (1k, 10k).
   - Mixed shapes: spheres/boxes/capsules, fixed RNG seed.
   - Static/moving ratio: 90/10 to test incremental updates.

3. **Collect profiler reports**
   - Use the driver in `docs/dev_tasks/experimental_collision/rp3d_profile_driver.cpp`.
   - Use `PhysicsWorld::getProfiler()` and `Profiler::addFileDestination()`.
   - Warm up for N frames; call `Profiler::reset()` before measurement.
   - Capture `Profiler::printReport()` after M frames.

4. **Extract stage timings**
   - Broadphase update and pair generation.
   - Middle phase (concave handling).
   - Narrowphase (convex and contact generation).

5. **Compare to DART**
   - Run DART pipeline breakdown benchmark on matched scenarios.
   - Compare broadphase vs narrowphase ratios and pair counts.

6. **Document findings**
   - Summarize results and attach profiler output in
     `docs/dev_tasks/experimental_collision/results/`.

## Immediate next steps

1. Use `rp3d_profile_driver.cpp` to capture dense/sparse baselines for 1k/10k.
2. Record results in `reactphysics3d_ecs_profile_results.md` and attach raw logs.
3. Compare against matched DART pipeline breakdown scenarios.

## Driver usage (example)

```bash
c++ -std=c++17 -O3 -DIS_RP3D_PROFILING_ENABLED \
  -I /path/to/reactphysics3d/include \
  docs/dev_tasks/experimental_collision/rp3d_profile_driver.cpp \
  -L /path/to/reactphysics3d/build_profile -lreactphysics3d \
  -Wl,-rpath,/path/to/reactphysics3d/build_profile \
  -o /tmp/rp3d_profile_driver

/tmp/rp3d_profile_driver 10000 10 2 1 /tmp/rp3d_profile_10k_dense.txt
/tmp/rp3d_profile_driver 10000 50 2 1 /tmp/rp3d_profile_10k_sparse.txt
```
