# Collision Benchmark Catalog

> **Last Updated**: 2026-01-19

## Purpose

- Centralize benchmark cases and datasets to build the structured suite for the experimental collision module.
- Ensure every case is comparable against FCL, Bullet, and ODE (and optionally Coal/Parry for deeper analysis).
- Use this catalog to prioritize batch-oriented benchmarks and avoid ad-hoc, one-off tests.
- Record structured results in `benchmark_results.md` after each run.

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

## Per-query, per-shape case checklist

Each entry includes performance-oriented cases, most frequently used cases, and edge cases. Use this to populate the DART structured suite and ensure every {query, shape} combination has coverage.

### Collision and contact (discrete)

| Shape or pair                | Performance-oriented cases             | Most frequent cases                             | Edge cases                                                                  |
| ---------------------------- | -------------------------------------- | ----------------------------------------------- | --------------------------------------------------------------------------- |
| Sphere-sphere                | 100k random pairs, low overlap ratio   | Small radius objects in clutter                 | Touching at 1e-6 gap, concentric centers, very large radii                  |
| Sphere-box                   | 100k pairs, random orientations        | Rolling on ground plane, robot link vs obstacle | Tangent along edge/corner, box near-degenerate extents                      |
| Box-box (OBB)                | 50k pairs with random rotations        | Stacking and pile stability                     | Face-face coplanar, edge-edge grazing, thin box (slab)                      |
| Capsule-capsule              | 50k pairs with random axis directions  | Robot links, humanoid limbs                     | Parallel axes, endcap-only contact, near-zero length capsule                |
| Cylinder-cylinder            | 50k pairs with random axis directions  | Industrial parts, wheels                        | Parallel axes with small gap, cap-cap touch, tall thin cylinders            |
| Plane-primitive              | 10k objects above plane                | Ground contact in simulation                    | Near-parallel motion, penetration depth near zero                           |
| Cone/ellipsoid pairs         | 20k pairs                              | Less common but present in robotics meshes      | Apex contact, extreme aspect ratios                                         |
| Convex-convex (hulls)        | 20k pairs from random convex hulls     | Proxy shapes for meshes                         | Coplanar faces, near-degenerate hulls, repeated vertices                    |
| Mesh-primitive               | 10k moving primitives vs static mesh   | Environment collision                           | Triangle edge grazing, degenerate triangles, non-manifold edges             |
| Mesh-mesh                    | 5k mesh pairs (static vs moving)       | Rare in robotics but necessary                  | Nearly coplanar faces, vertex-face grazing, deep interpenetration           |
| Heightfield-primitive        | 10k moving primitives over heightfield | Terrain and ground planes                       | High-frequency height changes, steep slopes, large coordinate magnitudes    |
| Compound-composite           | 10k compound vs primitive              | Articulated bodies with multiple shapes         | Overlapping child shapes, sparse vs dense child layout                      |
| SDF/voxel vs primitive (opt) | 10k SDF queries with varied resolution | Terrain/implicit geometry                       | Voxel boundary grazing, coarse resolution artifacts, gradient discontinuity |

### Distance and closest points

| Shape or pair                | Performance-oriented cases       | Most frequent cases                     | Edge cases                                                   |
| ---------------------------- | -------------------------------- | --------------------------------------- | ------------------------------------------------------------ |
| Sphere-sphere                | 100k random pairs                | Distance checks for collision avoidance | Near-zero distance, very large distances, concentric spheres |
| Sphere-box                   | 100k pairs with random rotations | Proximity of obstacles to links         | Closest point on edge/corner, box near-degenerate            |
| Box-box (OBB)                | 50k pairs                        | Clearance for stacking and placement    | Parallel faces with tiny gap, edge-edge closest points       |
| Capsule-capsule              | 50k pairs                        | Robot limb clearance                    | Parallel axes, endpoints aligned, near-zero length capsule   |
| Cylinder-cylinder            | 50k pairs                        | Mechanical parts clearance              | Parallel axes with tiny gap, cap-edge closest points         |
| Plane-primitive              | 10k objects                      | Distance to ground/terrain              | Large offset, near-parallel orientation                      |
| Convex-convex (hulls)        | 20k convex pairs                 | Proxy clearance for mesh models         | Non-unique closest points, degenerate faces                  |
| Mesh-primitive               | 10k primitive vs static mesh     | Obstacle distance fields                | Closest point on vertex/edge, degenerate triangles           |
| Mesh-mesh                    | 5k pairs                         | Rare but needed for generality          | Multiple equally closest points, overlapping meshes          |
| Heightfield-primitive        | 10k pairs                        | Terrain clearance                       | High-frequency heightfield, large scales                     |
| SDF/voxel vs primitive (opt) | 10k samples                      | Gradient-based proximity                | Gradient discontinuities, sign flips at voxel boundaries     |

### Raycast

| Shape or pair      | Performance-oriented cases | Most frequent cases        | Edge cases                                                                  |
| ------------------ | -------------------------- | -------------------------- | --------------------------------------------------------------------------- |
| Ray vs primitives  | 100k rays mixed hit/miss   | Sensor and perception rays | Ray origin inside shape, grazing tangents, near-parallel to face            |
| Ray vs mesh        | 10k rays vs static mesh    | Environment queries        | Triangle edges/vertices, degenerate triangles, backface culling consistency |
| Ray vs heightfield | 10k rays                   | Terrain sensing            | Steep slopes, large coordinate scales                                       |
| Ray vs compound    | 10k rays                   | Articulated body picking   | Child overlap, nested transforms                                            |

### Shape cast / CCD (time of impact)

| Shape or pair               | Performance-oriented cases          | Most frequent cases       | Edge cases                                                          |
| --------------------------- | ----------------------------------- | ------------------------- | ------------------------------------------------------------------- |
| Sphere cast vs primitives   | 10k casts with varied sweep lengths | Basic CCD for fast movers | Tangent sweeps, zero-length sweep, starting in penetration          |
| Capsule cast vs primitives  | 10k casts                           | Robot links               | Parallel to plane, grazing edges, near-zero length capsule          |
| Convex cast vs convex       | 5k casts                            | Proxy mesh CCD            | Multiple contact points, feature switching, deep start penetrations |
| Mesh/heightfield cast (opt) | 2k casts                            | Environment sweeps        | Dense mesh, thin triangles, large step sizes                        |

### Broadphase and batch throughput

| Shape or pair             | Performance-oriented cases                 | Most frequent cases                | Edge cases                                                   |
| ------------------------- | ------------------------------------------ | ---------------------------------- | ------------------------------------------------------------ |
| Mixed primitives (1k/10k) | Dense and sparse distributions; stable IDs | Typical robotics scenes            | All objects moving, clustered centers, extreme aspect ratios |
| Mesh-heavy scenes         | Static meshes + moving primitives          | Environments with complex geometry | Large meshes, unbalanced BVH, many overlapping AABBs         |
| Heightfield scenes        | Terrain + many primitives                  | Ground contact stress              | High-frequency terrain, long thin objects                    |
| Compound-heavy scenes     | Many compound bodies                       | Articulated robots                 | Many children per body, nested compounds                     |

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
