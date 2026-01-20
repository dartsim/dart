# Collision Benchmark Catalog

> **Last Updated**: 2026-01-19

## Purpose

- Centralize benchmark cases and datasets to build the structured suite for the experimental collision module.
- Ensure every case is comparable against FCL, Bullet, and ODE (and optionally Coal/Parry for deeper analysis).
- Use this catalog to prioritize batch-oriented benchmarks and avoid ad-hoc, one-off tests.

## Benchmark sources (survey)

### DART (existing)

- `tests/benchmark/collision/bm_experimental.cpp` - experimental-only microbenchmarks.
- `tests/benchmark/collision/bm_comparative.cpp` - comparative subset vs FCL/Bullet/ODE.
- `tests/benchmark/collision/comparative/bm_distance.cpp` - comparative distance queries.
- `tests/benchmark/collision/comparative/bm_narrow_phase.cpp` - comparative narrow-phase.
- `tests/benchmark/collision/comparative/bm_raycast.cpp` - comparative raycast.
- `tests/benchmark/collision/scenarios/bm_mixed_primitives.cpp` - mixed primitives scenarios.
- `tests/benchmark/collision/scenarios/bm_mesh_heavy.cpp` - mesh-heavy scenes.
- `tests/benchmark/collision/scenarios/bm_raycast_batch.cpp` - batched raycasts.
- `tests/benchmark/collision/experimental/bm_ccd.cpp` - experimental CCD microbenchmarks.
- `tests/benchmark/collision/fixtures/` - shared shape/scene builders.
- `tests/benchmark/collision/data/` - shared mesh/convex fixtures.

### Library-provided benchmarks and demos

- Bullet: `/home/js/dev/physics/bullet3/examples/Benchmarks/BenchmarkDemo.cpp` with Halton/Taru/landscape data.
- Coal (HPP-FCL): `/home/js/dev/physics/hpp-fcl/test/benchmark.cpp`, `accelerated_gjk.cpp`, `profiling.cpp`, `gjk-geometric-tools-benchmark/`.
- Parry: `/home/js/dev/physics/parry/crates/parry3d/benches/` (bounding_volume, query, support_map).
- collision-rs: `/home/js/dev/physics/collision-rs/benches/dbvt.rs`, `polyhedron.rs`.
- BEPUphysics1: `/home/js/dev/physics/bepuphysics1/BEPUphysicsDemos/` and `Documentation/Isolated Demos/`.
- JitterPhysics: `/home/js/dev/physics/jitterphysics/samples/JitterDemo` and `JitterOpenGLDemo`.
- qu3e: `/home/js/dev/physics/qu3e/demo/` (box stacking, raycast, aggregate bodies).
- tinyc2: `/home/js/dev/physics/tinyheaders/` examples (2D); external tests at https://github.com/sro5h/tinyc2-tests.
- ReactPhysics3D: `/home/js/dev/physics/reactphysics3d/testbed/` scenes.
- ODE: `/home/js/dev/physics/ODE/ode/demo/` and unit tests in `/home/js/dev/physics/ODE/tests/`.
- FCL/libccd/OpenGJK: no dedicated benchmarks; use unit tests and example programs for microbench cases.

### Dedicated benchmark suites and datasets

- colbench: `/home/js/dev/physics/colbench` (ellipsoids + YCB).
  - Ellipsoids: `scripts/ellipsoids/generate_ellipsoids_dataset.sh`, `run_ellipsoids_benchmark.sh`
  - YCB: `scripts/ycb/download_and_process_ycb_dataset.sh`, `generate_ycb_collision_problems_dataset.sh`, `run_ycb_benchmark.sh`
- collision-detection-benchmark: `/home/js/dev/physics/collision-detection-benchmark` (ellipsoids + ShapeNet convex meshes).
  - Quick runs: `exp/continuous_ellipsoids/ellipsoids_quick_benchmark.py`, `exp/shapenet/shapenet_quick_benchmark.py`
  - Large runs: `ellipsoids_benchmark.sh`, ShapeNet subset via `exp/shapenet/generate_subshapenet.py`
- spatial-collision-datastructures: `/home/js/dev/physics/spatial-collision-datastructures` (broadphase data structure benchmark).
- YCB dataset (used by colbench).
- ShapeNetCore.v2 (used by collision-detection-benchmark).

## Benchmark taxonomy (organize new cases by category)

| Category                    | Scenarios (examples)                        | Query types                | Candidate sources                          | Notes                                                  |
| --------------------------- | ------------------------------------------- | -------------------------- | ------------------------------------------ | ------------------------------------------------------ |
| Primitive microbench        | Sphere-sphere, box-box, capsule-capsule     | collision, distance        | DART comparative, HPP-FCL tests, qu3e      | Baseline latency + accuracy checks                     |
| Convex-convex distance      | Ellipsoid pairs                             | distance, closest points   | colbench, collision-detection-benchmark    | Good for GJK/EPA and gradient validation               |
| Convex mesh distance        | ShapeNet convex meshes                      | distance, collision        | collision-detection-benchmark              | Tests non-strictly convex cases                        |
| Mesh-heavy scenes           | Static meshes + moving primitives           | collision, contacts        | DART scenarios, Bullet benchmark demo      | Stresses BVH traversal and cache locality              |
| Heightfield/terrain         | Moving primitives on terrain                | collision, contacts        | BEPU/Jitter/ReactPhysics3D demos           | Important for robotics ground contact                  |
| Raycast batch               | 10k rays with hit/miss mix                  | raycast                    | DART raycast batch, Bullet demo            | Measure query/sec and result determinism               |
| CCD/shape cast              | Swept sphere/capsule/convex cast            | time of impact, shape cast | DART bm_ccd, Parry shape casts, BEPU casts | Align linear vs nonlinear CCD modes                    |
| Broadphase scaling          | 1k/10k objects, dense vs sparse             | broadphase + collision     | spatial-collision-datastructures, DART     | Report pairs/sec + update cost                         |
| Contact stability           | Box stacking, aggregates, frictional stacks | contacts, manifold quality | qu3e demo, Bullet/ODE demos, BEPU demos    | Track contact count, stability, and solver convergence |
| 2D sanity checks (optional) | Simple 2D convex pairs and time of impact   | collision, time of impact  | tinyc2, ncollide2d                         | Useful for algorithm regression tests                  |

## Benchmark structure guidelines

- Use identical geometry, transforms, and tolerances across backends.
- Separate data generation from measurement (no allocations in hot loops).
- Report both throughput (pairs/sec, queries/sec) and accuracy deltas (normal, depth).
- Capture batch performance with stable IDs and cached manifolds where supported.
- Fix RNG seeds, include warm-up iterations, and log environment details.

## Adoption checklist for new benchmark cases

1. Identify dataset or generator (e.g., YCB, ShapeNet, ellipsoids).
2. Add a DART fixture that produces identical shapes in FCL/Bullet/ODE.
3. Define expected accuracy metrics (distance, depth, normals) and tolerances.
4. Implement both microbench and batch versions when applicable.
5. Record results in the structured suite with a reproducible config.
