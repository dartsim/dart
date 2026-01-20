# Reference Collision Libraries: Core Backends and Ecosystem Survey

## Scope and sources (local snapshot)

- FCL: `/home/js/dev/physics/fcl` (commit 83a2ed2, 2026-01-15; 5 commits in last 12 months)
- Bullet: `/home/js/dev/physics/bullet3` (commit 63c4d67e3, 2025-10-21; 5 commits in last 12 months)
- ODE: `/home/js/dev/physics/ODE` (commit bcfb66cd, 2024-01-14; 0 commits in last 12 months)
- libccd: `/home/js/dev/physics/libccd` (commit 7931e76, 2018-12-22; 0 commits in last 12 months)
- Coal (HPP-FCL): `/home/js/dev/physics/hpp-fcl` (commit aad09897, 2026-01-16; 433 commits in last 12 months)
- ReactPhysics3D: `/home/js/dev/physics/reactphysics3d` (commit 7000610a, 2025-01-09; 0 commits in last 12 months)
- Parry: `/home/js/dev/physics/parry` (commit 72f842d, 2026-01-09; 88 commits in last 12 months)
- OpenGJK: `/home/js/dev/physics/opengjk` (commit 5eca7fa, 2025-12-29; 29 commits in last 12 months)
- BEPUphysics1: `/home/js/dev/physics/bepuphysics1` (commit 88243f8, 2021-08-02; 0 commits in last 12 months)
- JitterPhysics: `/home/js/dev/physics/jitterphysics` (commit 9f108c0, 2021-08-08; 0 commits in last 12 months)
- ncollide: `/home/js/dev/physics/ncollide` (commit f3c3ecb3, 2022-03-18; 0 commits in last 12 months)
- collision-rs: `/home/js/dev/physics/collision-rs` (commit 29090c4, 2021-10-08; 0 commits in last 12 months)
- qu3e: `/home/js/dev/physics/qu3e` (commit 1f519c9, 2021-05-08; 0 commits in last 12 months)
- tinyc2 (cute_c2): `/home/js/dev/physics/tinyheaders` (commit af97c17, 2026-01-07; 8 commits in last 12 months)
- Voxblox: `/home/js/dev/physics/voxblox` (commit c8066b0, 2021-11-22; 0 commits in last 12 months) from https://github.com/ethz-asl/voxblox
- Legacy/inactive references (not cloned locally): ColDet, GIMPACT, OPCODE, SOLID, OZCollide.
- Sources used: README files plus public headers and source trees in each repo.
- Benchmark suites: `/home/js/dev/physics/colbench` (commit e09ed36, 2023-07-03), `/home/js/dev/physics/collision-detection-benchmark` (commit 4073887, 2022-07-07), `/home/js/dev/physics/spatial-collision-datastructures` (commit 3b993fb, 2018-03-05).
- External curated list: /home/js/dev/physics/awesome-collision-detection (commit effd3f8) from https://github.com/jslee02/awesome-collision-detection (library catalog, not exhaustive).
- This is a snapshot. Refresh before release or external publication.

## High-level goals and positioning

### FCL

- Goal: proximity queries (collision, distance, tolerance, continuous) for triangle meshes and primitives.
- Design: heavy BVH use with multiple BV types; optional octree via octomap; convex uses libccd or internal GJK/EPA.
- Strengths: accurate distance and CCD; broad shape coverage; broadphase managers for large scenes.
- Gaps: no explicit public raycast API found in headers; not a full physics engine.

### Bullet

- Goal: real-time collision detection integrated with multi-physics simulation (games, VR, robotics).
- Design: broadphase + dispatch + narrowphase with persistent manifolds; collision margin is central to stability.
- Strengths: broad shape coverage, ray and convex sweep queries, strong real-time performance, mature tooling.
- Gaps: contact margins trade geometric accuracy for stability; mixed heuristics can reduce determinism.

### ODE

- Goal: rigid body dynamics with built-in collision in a C API.
- Design: geom types + space-based broadphase; triangle mesh via OPCODE or GIMPACT; convex via libccd.
- Strengths: simple integration; multiple broadphase "spaces"; straightforward primitives.
- Gaps: no general distance query API; no general continuous collision/time of impact API; slower recent development.

### libccd (convex-only algorithms)

- Goal: minimal C library for convex-convex queries using support functions.
- Design: GJK intersection/separation + EPA penetration; MPR intersection/penetration.
- Strengths: small, portable, easy to embed; MPR reference implementation.
- Gaps: no broadphase, no mesh or concave shapes, no raycast, no general distance.
- Caution: not battle-tested across all edge cases; known algorithmic bugs reported in the wild. Treat as a reference, not a drop-in, and add exhaustive unit tests when porting (cover degeneracies and every algorithm path).

### Coal (HPP-FCL fork of FCL)

- Goal: high-performance proximity queries for robotics; FCL fork renamed to Coal.
- Design: dedicated GJK/EPA (no libccd), accelerated GJK variants, security margins, contact points and patches, lower-bound distance.
- Strengths: strong performance focus, security margin support, richer contact data, Python bindings, serialization.
- Shapes: boxes, spheres, capsules, cylinders, cones, ellipsoids, planes/halfspace, convex meshes, BVHs, height fields, octrees.
- Gaps: not a physics engine; continuous collision API not prominently documented in README.

### ReactPhysics3D

- Goal: full physics engine with discrete collision detection for games/simulation.
- Design: dynamic AABB tree broadphase; SAT/GJK narrowphase; sequential impulse solver.
- Strengths: clear shape set (sphere/box/capsule/convex mesh/static concave mesh/height field), ray casting, collision filtering, integrated profiler.
- Gaps: no general distance/closest-points API exposed in headers; no CCD support found.

### Parry (dimforge)

- Goal: standalone 2D/3D geometry and collision queries (Rust).
- Design: query module with closest points, distance, contact, intersection, ray casting, and shape casts (linear + nonlinear); BVH partitioning utilities.
- Strengths: broad query set including continuous shape casts, contact manifolds, rich shape set (trimesh, heightfield, voxels, compound).
- Gaps: not a full physics engine; world/broadphase orchestration is user-managed.

### OpenGJK

- Goal: robust C implementation of GJK for distance between convex polytopes.
- Strengths: minimal, fast distance queries with witness points; multi-language bindings.
- Gaps: convex-only; no penetration depth/contact manifold; no broadphase or scene management.

### BEPUphysics1

- Goal: C# 3D physics engine with CCD, multithreading, and broad collision support.
- Design: broadphase systems (dynamic hierarchy, sort-and-sweep, brute), convex collision via GJK/MPR, contact manifolds and reducers.
- Strengths: convex/compound/mesh/terrain shapes, convex cast and raycast support, contact manifold infrastructure.
- Gaps: older v1 codebase, low recent activity, conventions not clearly documented.

### JitterPhysics

- Goal: lightweight C# 3D physics engine for games.
- Design: collision systems (SAP, persistent SAP, brute), GJK and XenoCollide (MPR) for convex, manifold-based contacts.
- Strengths: simple API, multithreading, mesh/terrain and convex support, raycast + closest points.
- Gaps: no explicit CCD/time of impact API found; older codebase.

### ncollide

- Goal: 2D/3D Rust collision library (passively maintained, superseded by Parry).
- Design: dynamic BVH broadphase, contact manifolds, GJK/EPA-based convex queries, time of impact for linear motion.
- Strengths: rich shape set, ray cast, time of impact for non-rotating motion.
- Gaps: maintenance mode, limited evolution compared to Parry.

### collision-rs

- Goal: Rust collision primitives and broadphase utilities built on cgmath.
- Design: DBVT + sweep-and-prune + brute force, GJK/EPA for manifolds, continuous GJK for CCD.
- Strengths: explicit DBVT and polyhedron benchmarks, convex distance queries.
- Gaps: not fully covered by tests, limited scene/world orchestration.

### qu3e

- Goal: minimal C++ 3D engine focused on boxes and simplicity.
- Design: dynamic AABB tree broadphase, SAT for OBB, manifold-based contacts.
- Strengths: simple API, stable manifold generation, AABB/point queries, raycast.
- Gaps: box-only shapes, no CCD, intentionally minimal features.

### tinyc2 (cute_c2)

- Goal: 2D header-only narrowphase routines (immediate mode).
- Design: GJK-based closest points, time of impact shape casts, optional manifolds; no broadphase.
- Strengths: clear 2D reference implementation, convex hull generation, time of impact examples.
- Gaps: 2D only, no scene management.

### Voxblox

- Goal: incremental volumetric mapping with TSDF/ESDF distance fields for planning.
- Design: sparse voxel block layers (TSDF/ESDF/occupancy) keyed by hashed block indices; `Layer` stores `Block` objects with dense voxel arrays and per-block update flags; `TsdfVoxel` stores distance/weight/color while `EsdfVoxel` tracks distance, observed, fixed-band status, queue state, and parent direction.
- TSDF pipeline: simple/merged/fast integrators fuse point clouds with truncation, range limits, weight drop-off, and optional clearing; merging and anti-grazing options trade accuracy for throughput.
- ESDF pipeline: raise/lower wavefront updates via a bucketed priority queue (fast marching), with configurable fixed band (`min_distance_m`), minimum change threshold (`min_diff_m`), and full vs quasi-Euclidean distance; neighborhood connectivity uses 6/18/26 offsets with precomputed sqrt(2)/sqrt(3) costs.
- Queries: `EsdfMap` provides distance + gradient queries with optional trilinear interpolation; gradients use voxel-size central differences and can fall back to adaptive one-sided estimates; batch query helpers are exposed for Python.
- Strengths: well-tested mapping stack with TSDF-to-ESDF conversion, batch query helpers, and marching-cubes mesh extraction; clear separation of layers and integrators for reuse.
- Gaps: not a collision detection engine; no contact manifolds, broadphase, or continuous collision/time-of-impact queries; resolution-limited and best suited for static scene clearance checks.
- Build notes: ROS/catkin-centric build and glog/protobuf dependencies; not lightweight for non-ROS environments.
- Relevance to DART: strong reference for TSDF/ESDF algorithms and data layout, but should remain a test/benchmark-only comparison (no core dependency).

## Feature comparison (from local source trees)

### Query types

| Query                                       | FCL | Bullet | ODE | Notes                                                    |
| ------------------------------------------- | --- | ------ | --- | -------------------------------------------------------- |
| Collision (binary + contacts)               | Y   | Y      | Y   | FCL/Bullet return contacts; ODE returns dContactGeom     |
| Distance / closest points                   | Y   | Y      | N   | Bullet exposes closest points via dispatch algorithms    |
| Tolerance-only check                        | Y   | N      | N   | FCL lists tolerance verification in README               |
| Raycast                                     | N   | Y      | Y   | FCL has no public ray API found; ODE uses ray geom       |
| Continuous collision (CCD / time of impact) | Y   | Y      | N   | FCL continuousCollide; Bullet convex sweep / convex cast |
| Broadphase group queries                    | Y   | Y      | Y   | FCL managers; Bullet broadphase; ODE spaces              |

### Query types (additional local libraries)

Legend: Y = supported, N = not supported, P = partial or user-managed.

| Query                                       | Coal | ReactPhysics3D | Parry | OpenGJK | Notes                                                     |
| ------------------------------------------- | ---- | -------------- | ----- | ------- | --------------------------------------------------------- |
| Collision (binary + contacts)               | Y    | Y              | Y     | P       | OpenGJK collision is via distance==0, no contact manifold |
| Distance / closest points                   | Y    | N              | Y     | Y       | ReactPhysics3D does not expose a general distance API     |
| Tolerance-only check                        | Y    | N              | N     | N       | Coal uses security margins                                |
| Raycast                                     | N    | Y              | Y     | N       |                                                           |
| Continuous collision (CCD / time of impact) | P    | N              | Y     | N       | Coal CCD not prominent in README; Parry has shape casts   |
| Broadphase group queries                    | Y    | Y              | P     | N       | Parry provides BVH utilities, not a full world manager    |

### Query types (community libraries)

Legend: Y = supported, N = not supported, P = partial or user-managed.

| Query                                       | BEPUphysics1 | JitterPhysics | ncollide | collision-rs | qu3e | tinyc2 | Notes                                                               |
| ------------------------------------------- | ------------ | ------------- | -------- | ------------ | ---- | ------ | ------------------------------------------------------------------- |
| Collision (binary + contacts)               | Y            | Y             | Y        | Y            | Y    | Y      | tinyc2 is 2D only                                                   |
| Distance / closest points                   | Y            | Y             | Y        | Y            | N    | Y      | collision-rs convex distance; tinyc2 GJK closest points             |
| Tolerance-only check                        | N            | N             | N        | N            | N    | N      |                                                                     |
| Raycast                                     | Y            | Y             | Y        | P            | Y    | Y      | collision-rs has Ray primitives; query coverage varies              |
| Continuous collision (CCD / time of impact) | Y            | N             | P        | P            | N    | Y      | ncollide time of impact is linear-only; collision-rs continuous GJK |
| Broadphase group queries                    | Y            | Y             | Y        | P            | Y    | N      | collision-rs exposes DBVT/SaP; tinyc2 has no broadphase             |

### Batch query support (data structures and throughput)

| Library        | Batch-friendly structures                          | Batch optimizations                                  | Notes                                                                       |
| -------------- | -------------------------------------------------- | ---------------------------------------------------- | --------------------------------------------------------------------------- |
| FCL            | Dynamic AABB tree managers, SaP/SSaP, spatial hash | Manager-level pair enumeration                       | Designed for group queries; BVH traversal overhead can dominate tiny scenes |
| Bullet         | DBVT broadphase, pair cache                        | Persistent manifolds, pair cache reuse               | Strong temporal coherence; margins trade accuracy for stability             |
| ODE            | Spaces (hash, SAP, quadtree)                       | Space hierarchy + callback-based pair traversal      | Simple batch API; performance depends on space choice                       |
| libccd         | None                                               | None                                                 | Pure narrowphase; no scene or batch layer                                   |
| Coal           | Dynamic AABB tree managers, SaP/SSaP, spatial hash | ComputeCollision caches per-pair solver state        | Strong batch usage; security margins influence throughput                   |
| ReactPhysics3D | Dynamic AABB tree                                  | Persistent manifolds, broadphase incremental updates | Discrete pipeline; contact persistence managed by engine                    |
| Parry          | BVH partitioning utilities                         | User-managed pair pipeline (Rapier provides caching) | Broadphase orchestration is external (e.g., in Rapier)                      |
| OpenGJK        | None                                               | None                                                 | Distance-only GJK kernel                                                    |
| BEPUphysics1   | Dynamic hierarchy, sort-and-sweep, brute           | Multithreaded pipeline + manifold refresh            | Internal multithreading, query accelerators                                 |
| JitterPhysics  | SAP, persistent SAP, brute                         | Persistent SAP + manifold caching                    | Persistent SAP favors temporal coherence                                    |
| ncollide       | Dynamic BVH                                        | Pipeline caches contact manifolds                    | Broadphase is integrated into the pipeline/world                            |
| collision-rs   | DBVT, sweep and prune                              | User-managed batch caching                           | Provides data structures; no full world manager                             |
| qu3e           | Dynamic AABB tree                                  | Minimal caching; simple pair updates                 | Minimal engine; batch performance tied to simple data layout                |
| tinyc2         | None                                               | None                                                 | Narrowphase-only 2D library                                                 |

Batch here means many pairwise queries per frame. None of these expose a dedicated SIMD batch API, so throughput depends on per-call overhead, data layout, and how much temporal coherence is captured (pair caches, persistent manifolds).

### Convex-only reference (libccd)

| Capability                | libccd | Notes                                       |
| ------------------------- | ------ | ------------------------------------------- |
| Intersect (GJK)           | Y      | `ccdGJKIntersect`                           |
| Penetration (EPA)         | Y      | `ccdGJKPenetration`                         |
| Intersect (MPR)           | Y      | `ccdMPRIntersect`                           |
| Penetration (MPR)         | Y      | `ccdMPRPenetration`                         |
| Separation vector         | Y      | `ccdGJKSeparate` (for intersecting objects) |
| Distance / closest points | N      | No explicit distance query                  |
| Raycast                   | N      |                                             |
| Broadphase                | N      |                                             |

### Shape types (core)

| Shape                | FCL         | Bullet | ODE     | Notes                                                     |
| -------------------- | ----------- | ------ | ------- | --------------------------------------------------------- |
| Sphere               | Y           | Y      | Y       |                                                           |
| Box                  | Y           | Y      | Y       |                                                           |
| Capsule              | Y           | Y      | Y       | ODE uses CCylinder alias                                  |
| Cylinder             | Y           | Y      | Y       |                                                           |
| Cone                 | Y           | Y      | N       |                                                           |
| Ellipsoid            | Y           | N      | N       | Bullet can approximate via multi-sphere or scaling        |
| Plane / half-space   | Y           | Y      | Y       | FCL includes plane + halfspace                            |
| Convex hull          | Y           | Y      | Y       |                                                           |
| Triangle mesh        | Y           | Y      | Y       | FCL BVHModel; Bullet Bvh/GImpact; ODE OPCODE/GIMPACT      |
| Heightfield          | N           | Y      | Y       | Bullet heightfield; ODE heightfield                       |
| Octree / voxels      | Y (octomap) | N      | N       | FCL optional octree                                       |
| Compound / aggregate | N           | Y      | Partial | Bullet btCompoundShape; ODE uses spaces or geom transform |
| SDF / implicit       | N           | Y      | N       | Bullet btSdfCollisionShape / btMiniSDF                    |
| 2D shapes            | N           | Y      | N       | Bullet has 2D collision shapes                            |

### Shape types (additional local libraries)

Legend: Y = supported, N = not supported, P = generic convex/user-defined.

| Shape              | Coal | ReactPhysics3D | Parry | OpenGJK | Notes                                                     |
| ------------------ | ---- | -------------- | ----- | ------- | --------------------------------------------------------- |
| Sphere/ball        | Y    | Y              | Y     | P       | OpenGJK uses user-supplied convex polytopes               |
| Box/cuboid         | Y    | Y              | Y     | P       |                                                           |
| Capsule            | Y    | Y              | Y     | P       |                                                           |
| Cylinder           | Y    | N              | Y     | P       |                                                           |
| Cone               | Y    | N              | Y     | P       |                                                           |
| Ellipsoid          | Y    | N              | N     | P       |                                                           |
| Plane/half-space   | Y    | N              | Y     | N       |                                                           |
| Convex hull        | Y    | Y              | Y     | P       |                                                           |
| Mesh (concave)     | Y    | Y (static)     | Y     | N       | ReactPhysics3D supports static concave mesh               |
| Heightfield        | Y    | Y              | Y     | N       |                                                           |
| Compound/composite | P    | Y              | Y     | N       | Coal lacks explicit compound shapes; use multiple objects |
| Voxels/SDF         | N    | N              | Y     | N       | Parry includes voxel shapes                               |

### Shape types (community libraries)

Legend: Y = supported, N = not supported, P = limited or 2D-only.

| Shape               | BEPUphysics1 | JitterPhysics | ncollide | collision-rs | qu3e | tinyc2 | Notes                                     |
| ------------------- | ------------ | ------------- | -------- | ------------ | ---- | ------ | ----------------------------------------- |
| Sphere/ball         | Y            | Y             | Y        | Y            | N    | P      | tinyc2 uses circles (2D)                  |
| Box/cuboid          | Y            | Y             | Y        | Y            | Y    | P      | tinyc2 has AABB/convex polygons           |
| Capsule             | Y            | Y             | Y        | Y            | N    | P      | tinyc2 supports 2D capsule                |
| Cylinder            | Y            | Y             | Y        | Y            | N    | N      |                                           |
| Cone                | Y            | Y             | Y        | N            | N    | N      |                                           |
| Convex hull/poly    | Y            | Y             | Y        | Y            | N    | P      | tinyc2 supports convex polygons (2D)      |
| Mesh (concave)      | Y            | Y             | Y        | N            | N    | N      |                                           |
| Heightfield/terrain | Y            | Y             | Y        | N            | N    | N      |                                           |
| Compound/composite  | Y            | Y             | Y        | N            | P    | N      | qu3e aggregates boxes only                |
| 2D shapes           | N            | N             | Y        | Y            | N    | Y      | ncollide/collision-rs include 2D variants |

### Broadphase algorithms

- FCL: Dynamic AABB tree, dynamic AABB tree array, sweep and prune (SaP/SSaP), spatial hash, interval tree, brute force.
- Bullet: DBVT (dynamic BVH), axis sweep (3D), simple broadphase; overlapping pair cache.
- ODE: simple space, hash space, sweep and prune space, quadtree space; nested spaces allowed.

### Narrowphase and mesh algorithms

- FCL: GJK and EPA (internal or libccd), specialized primitive solvers (sphere-sphere, box-box SAT, sphere-box, etc); mesh and mesh-mesh via BVH with multiple BV types (AABB, OBB, RSS, OBBRSS, kDOP, kIOS).
- Bullet: GJK/EPA (btGjkPairDetector, btGjkEpa), MPR (btMprPenetration), polyhedral contact clipping, specialized box-box; persistent manifolds; mesh via BvhTriangleMeshShape (static concave) and GImpact (dynamic concave).
- ODE: analytic primitives plus libccd for convex (GJK/EPA); triangle mesh via OPCODE or GIMPACT; ray-mesh in collision_trimesh_ray.
- libccd: GJK/EPA + MPR only, convex-only, driven by user-defined support functions.

### Narrowphase and mesh algorithms (community libraries)

- BEPUphysics1: GJK (boolean and closest points) and MPR, manifold generators for convex/mesh/terrain contacts.
- JitterPhysics: GJK + XenoCollide (MPR) for convex, mesh and terrain via multishape handling.
- ncollide: GJK/EPA-based convex, contact manifold generators including trimesh; linear time of impact only.
- collision-rs: GJK/EPA for convex, continuous GJK for CCD; no mesh collision pipeline.
- qu3e: SAT-based OBB manifolds with polygon clipping; discrete only.
- tinyc2: 2D GJK + time of impact with convex hull support; narrowphase only.

## Conventions and Terminology

See `conventions.md` for the full DART normalization rules and adapter mapping,
and `contact_manifolds.md` for manifold representation guidance.

### Normal direction and distance sign

| Library        | Normal convention                         | Penetration sign                    | Signed distance support      | Notes                                                                                            |
| -------------- | ----------------------------------------- | ----------------------------------- | ---------------------------- | ------------------------------------------------------------------------------------------------ |
| DART           | normal points from bodyNode2 to bodyNode1 | penetrationDepth positive           | yes (DistanceResult)         | Signed distance is negative when penetrating; distance normals should align with point2 - point1 |
| FCL            | normal points from o1 to o2               | penetration_depth positive          | yes (enable_signed_distance) | DistanceResult is positive when separated; negative if signed enabled                            |
| Coal           | normal points from o1 to o2               | penetration_depth positive          | yes                          | Signed distance used to define normal/witness points                                             |
| Bullet         | normalWorldOnB points from B to A         | distance1 negative when penetrating | yes (distance1)              | If B is treated as object2, normal aligns with object2->object1                                  |
| ODE            | normal points into body1                  | depth positive                      | no                           | Equivalent to object2->object1 if g1 is object1                                                  |
| Parry          | normal1 points from shape1 to shape2      | dist negative when penetrating      | yes (dist)                   | Explicit signed distance in Contact                                                              |
| ReactPhysics3D | normal from body1 to body2                | penetrationDepth positive           | no                           | Discrete collision only                                                                          |
| libccd         | direction is translation to move obj2 out | penetration depth positive          | partial                      | Direction must be flipped to match DART contact normal; no distance API                          |
| OpenGJK        | N/A (distance-only)                       | N/A                                 | no                           | Returns minimum distance (0 if intersecting)                                                     |

### Contact representations and manifolds

| Library        | Contact representation              | Manifold behavior                                                                                                                                                           |
| -------------- | ----------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| DART           | Contact points + ContactManifold    | Legacy backend returns flat list; experimental uses fixed-size manifolds (4) with deterministic reduction; 4-point caps are surface manifolds (not volume); persistence TBD |
| FCL            | Contact points (pos, normal, depth) | No explicit manifold; returns list                                                                                                                                          |
| Coal           | Contact points + ContactPatch       | Supports patches (polygonal contact regions)                                                                                                                                |
| Bullet         | Persistent manifolds of points      | Up to 4 points per manifold, cached across frames                                                                                                                           |
| ODE            | Contact points (dContactGeom)       | No explicit manifold; caller aggregates                                                                                                                                     |
| Parry          | Contact + ContactManifold           | Explicit manifolds; normals + multiple points                                                                                                                               |
| ReactPhysics3D | ContactManifold with points         | Up to 4 points; cached for stability                                                                                                                                        |
| BEPUphysics1   | Contact manifolds                   | Manifold generators + contact reduction/refresher                                                                                                                           |
| JitterPhysics  | Contact manifolds                   | Manifold-based contacts in collision systems                                                                                                                                |
| ncollide       | ContactManifold                     | Explicit manifolds; part of pipeline                                                                                                                                        |
| collision-rs   | Contact data via GJK/EPA            | Manifold support per README; no world persistence                                                                                                                           |
| qu3e           | Contact manifolds                   | SAT manifolds; reduction not implemented                                                                                                                                    |
| tinyc2         | Contact manifold (2D)               | Optional manifold generation per query                                                                                                                                      |
| libccd         | Single penetration info             | No manifold; GJK/EPA gives one direction/depth                                                                                                                              |
| OpenGJK        | Witness points from simplex         | Distance-only, no contact manifold                                                                                                                                          |

### Contact result types (for DART to standardize)

| Type                   | Output                                 | Notes                                                           |
| ---------------------- | -------------------------------------- | --------------------------------------------------------------- |
| Point                  | position + normal + depth              | Baseline DART contact format                                    |
| Manifold (multi-point) | small set of points with shared normal | Improves stacking and friction stability; used in Bullet/Parry  |
| Patch / surface        | polygon or contact region              | Coal exposes ContactPatch; better torque/friction for face-face |
| Volume / SDF overlap   | volume metric + gradient               | Enables gradient-based methods; heavier storage and tuning      |

Edge-edge and face-face interactions are typically represented via a multi-point manifold or a patch output.

### DART consistency guidelines

- DART Contact defines the normal from bodyNode2 to bodyNode1, with positive penetration depth (see `dart/collision/Contact.hpp`).
- DART DistanceResult uses signed distance; negative means penetration (see `dart/collision/DistanceResult.hpp`).
- Experimental distance helpers currently mix normal conventions (object1->object2 vs object2->object1). Standardize on pointOnObject2 - pointOnObject1 and add regression tests.
- Provide adapter helpers when using libraries with opposite normal conventions (FCL/Coal/Parry/React) or signed depth (Bullet/Parry).
- Keep terminology consistent: signed distance is positive when separated, penetration depth is positive overlap.
- For BEPUphysics1, JitterPhysics, ncollide, collision-rs, qu3e, and tinyc2, normal/sign conventions are not clearly documented; verify in code before adapter work.

## Cross-Backend Validation Policy

These checks exist to ensure experimental and reference backends agree on
correctness for the same inputs.

**Test inputs**:

- Use shared fixtures and deterministic seeds across all backends
- Run common cases plus edge cases (grazing, deep penetration, thin features,
  near-parallel faces)
- Include scale sweeps for both geometry and transforms (tiny/nominal/large,
  e.g. 1e-3, 1, 1e3)

**Comparison rules**:

- Convert normals to DART's convention before comparing
- Compare hit/no-hit, signed distance, and penetration depth within tolerances
- Compare contact points with positional tolerance; compare normals with angular
  tolerance
- For Bullet, account for collision margins; either disable margins or document
  expected deltas
- For ESDF/SDF backends (Voxblox, Bullet SDF), sample query points and compare
  signed distance and gradients against analytic primitives or FCL/Parry
  distance queries; account for voxel resolution and interpolation

**Mismatch triage**:

- Re-check conventions (normal direction, signed distance, depth sign)
- Compare against analytic references for primitive pairs
- Use majority agreement across multiple backends when possible
- Document known deviations and add a follow-up task before integration

## SDF and Gradient-Based Queries

See `sdf_and_gradients.md` for a focused SDF + gradient survey and integration targets.

### SDF support (observed)

| Library                                       | SDF or voxel support            | Notes                                                                        |
| --------------------------------------------- | ------------------------------- | ---------------------------------------------------------------------------- |
| Bullet                                        | btSdfCollisionShape / btMiniSDF | Signed distance fields for implicit collision (btMiniSDF exposes gradients)  |
| Voxblox                                       | TSDF + ESDF + occupancy layers  | ESDF distance/gradient queries with optional interpolation and batch helpers |
| Parry                                         | Voxels                          | Voxel shapes exist; not explicitly SDF                                       |
| FCL / Coal                                    | None explicit                   | Focus on meshes/convex/BVH and GJK/EPA                                       |
| ODE / libccd / OpenGJK                        | None                            | No SDF primitives                                                            |
| ReactPhysics3D                                | None                            | Discrete contact shapes only                                                 |
| BEPU/Jitter/ncollide/collision-rs/qu3e/tinyc2 | None                            | No SDF primitives in inspected sources                                       |

### Voxblox implementation notes (local source)

- Data layout: sparse voxel blocks (Layer/Block) storing TSDF, ESDF, occupancy, and intensity voxels; blocks are keyed by hashed block indices.
- ESDF generation: EsdfIntegrator propagates TSDF into ESDF using fixed-band voxels plus raise/lower wavefront updates with a bucketed priority queue; optional full or quasi-Euclidean distance. EsdfOccIntegrator can build ESDF from occupancy with 26-connectivity.
- Query surface: EsdfMap exposes distance, distance+gradient, observed checks, and batch query helpers; Interpolator supports nearest or trilinear sampling and adaptive gradients.
- Integration/meshing: TSDF integrators (simple/merged/fast) fuse point clouds; MeshIntegrator runs marching cubes over SDF layers for mesh extraction.

### SDF references and implications (from curated list)

- Macklin 2020 (SDF contact) suggests robust SDF collision with local optimization and stable gradients.
- Koschier 2016 and Voxblox 2017 highlight hierarchical and incremental SDF construction pipelines; Voxblox uses raise/lower wavefront updates for ESDFs.
- Xu/Barbic 2014 covers SDF generation for polygon soup meshes; relevant for mesh import.

### Robotics physics use cases (SDF/ESDF)

- Proximity/clearance queries for navigation, manipulation, and footstep planning in complex static scenes.
- Gradient-based trajectory optimization (CHOMP/TrajOpt-style) and constraint-based planners that use distance + gradient signals.
- Contact smoothing or penalty-based contact for irregular terrain/geometry where mesh contact is noisy.
- RL training: dense shaping rewards (clearance, progress), safety constraints, and curriculum difficulty using distance-to-obstacle signals.

### Why SDF/ESDF over other queries or data structures

- Continuous distance + gradient signals; contact-only queries are sparse and discontinuous for optimization/RL.
- Query cost is near constant per sample and decoupled from mesh complexity; BVH traversal scales with scene complexity.
- ESDF enables fast batched queries for large rollouts, particles, or policy evaluations with predictable latency.
- Tradeoffs: memory and resolution limits, discretization artifacts, and costly updates for highly dynamic scenes.

### Placement in DART (core vs extension)

- Treat SDF/ESDF as an optional shape/backend in DART rather than a core replacement for exact geometry.
- Best fit: static environment or slow-changing obstacles; dynamic SDF updates are expensive and resolution-bound.
- Integrate through the standard query API (distance, gradient, raycast), while keeping exact convex/mesh paths for accuracy.

### CCD considerations for SDF/ESDF

- ESDF can support conservative advancement (distance + gradient along motion) for approximate CCD in static scenes.
- Not a drop-in replacement for exact CCD: discretization and interpolation can miss thin features or fast motions.
- For dynamic-dynamic CCD, a time-varying SDF or analytic CCD remains necessary for correctness.

### Experimental scope fit

- The experimental collision module is the right place to prototype SDF/ESDF shapes, gradients, and query APIs.
- Use it to evaluate runtime tradeoffs, then decide whether a stable SDF shape belongs in the core API.

### Runtime comparison plan (accuracy + performance)

- Compare dense SDF and Voxblox ESDF on identical query sets; track mean/max distance error and gradient alignment vs analytic SDFs.
- Benchmark query throughput for distance and distance+gradient; report queries/sec vs voxel size and interpolation mode.
- Use `dart/collision/experimental/benchmarks/bm_sdf_compare.cpp` and `tests/unit/collision/experimental/test_sdf_compare.cpp` as baselines and extend to Bullet/Parry when available.

### Gradient considerations for convex shapes

- GJK/EPA provide witness points and a separating/penetration direction; this is the natural gradient or subgradient of signed distance.
- For convex shapes, gradients are well-defined almost everywhere but can be non-unique at edges/vertices; return a stable normal with tie-breaking.
- Penetration gradients should align with the penetration direction (EPA or conservative advancement) and be consistent with the signed distance convention.
- Differentiable collision detection (randomized smoothing, Montaut 2022) can stabilize gradients near feature switches.
- When only convex shapes are present, the signed distance gradient should equal the normalized vector (pointOnObject2 - pointOnObject1); expose this explicitly to avoid recomputation in optimizers.

## Wider ecosystem snapshot (awesome-collision-detection)

Source: /home/js/dev/physics/awesome-collision-detection (commit effd3f8). The list is not exhaustive and many entries are physics engines with collision subsystems.

Active libraries inspected above: Bullet, FCL, HPP-FCL (Coal), libccd, ODE, OpenGJK, Parry, ReactPhysics3D, BEPUphysics1, JitterPhysics, ncollide, collision-rs, qu3e, tinyc2, Voxblox.

The curated list is not exhaustive; additional candidates may exist outside this list.

### Legacy/inactive references (still useful)

- ColDet: early triangle-mesh collision library; useful for legacy mesh contact heuristics.
- SOLID: classic convex/mesh collision stack (GJK + BVH), still a reference for API design.
- GIMPACT: dynamic triangle mesh collision (used by ODE/Bullet), BVH-based with triangle-triangle tests.
- OPCODE: legacy BVH/trimesh backend (used by ODE); optimized AABB trees and temporal coherence.
- OZCollide: legacy collision library with broadphase + narrowphase components.

### Mesh processing and decomposition references

- bounding-mesh: bounding mesh and convex decomposition algorithms.
- cinolib, libigl: geometry processing utilities (repair, decimation, SDF generation).

### Research and benchmarks (selected from curated list)

- GJK++ / Collision Detection Accelerated (Montaut 2022-2023): accelerated GJK and benchmarks.
- colbench: ellipsoid and YCB collision benchmarks built around HPP-FCL.
- collision-detection-benchmark: ellipsoid and ShapeNet convex mesh benchmarks for GJK variants.
- Differentiable collision detection (Montaut 2022): randomized smoothing for stable gradients.
- Robust mesh contact generation (Hauser 2013): contact for unstructured meshes.
- Penetration depth (PolyDepth 2012): iterative contact-space projection.
- SDF collision (Macklin 2020) and hierarchical SDFs (Koschier 2016); Voxblox for SDF construction.
- CCD (Tang 2009/2013, Brochu 2012): controlled advancement and geometrically exact CCD.
- Broadphase data structure benchmarks (ttvd/spatial-collision-datastructures).
- Reference texts: Real-Time Collision Detection (Ericson) and Collision Detection in Interactive 3D Environments.

## Performance notes (architecture-informed, not benchmarked here)

- FCL: strong for mesh-heavy scenes and accurate distance/CCD, but generic templates and BVH traversal add overhead for simple primitives.
- Bullet: optimized for real-time scenes with temporal coherence (persistent manifolds, pair caches); accuracy depends on margins and contact reduction.
- ODE: simpler collision pipeline that can be fast for basic primitives; lacks distance queries and modern CCD, and mesh handling depends on external backends.
- Coal: targets high-performance GJK/EPA, includes accelerated variants and security margins; reports large speedups in its own benchmarks.
- Parry: broad query set including shape casts; performance depends on BVH usage and user-side batching.
- ReactPhysics3D: discrete pipeline with custom containers (no STL) and integrated profiling; emphasis on stable real-time steps.
- BEPUphysics1: multiple broadphase options with multithreading; performance influenced by manifold reduction and C# runtime overhead.
- JitterPhysics: SAP and persistent SAP favor temporal coherence; overall speed depends on managed runtime and collision system choice.
- ncollide / collision-rs: performance depends on DBVT and user-managed pipelines; less tuned for large real-time scenes.
- qu3e: minimal and fast for box-only scenes; not representative for complex shape mixes.
- tinyc2: narrowphase-only 2D; useful for microbenchmarks, not scene-scale performance.
- Batch throughput: FCL and Bullet favor repeated-world queries with stable object sets; ODE favors simple per-step space traversal; libccd requires a separate batch layer.

## Dev activity snapshot (local repos)

| Library              | Last commit (date) | Last commit summary                                                                                                | Commits last 12 months |
| -------------------- | ------------------ | ------------------------------------------------------------------------------------------------------------------ | ---------------------- |
| FCL                  | 2026-01-15         | Feat: Check for local gtest first                                                                                  | 5                      |
| Bullet               | 2025-10-21         | add pyproject.toml                                                                                                 | 5                      |
| ODE                  | 2024-01-14         | Cosmetic: Commentary spelling corrections                                                                          | 0                      |
| libccd               | 2018-12-22         | Fixed division by zero in the computation of a direction vector in ccdMPRPenetration. (fixes #49)                  | 0                      |
| Coal (HPP-FCL)       | 2026-01-16         | Merge pull request #803 from jorisv/topic/fix-pixi-toml                                                            | 433                    |
| ReactPhysics3D       | 2025-01-09         | Fix the documentation URL in the README.md                                                                         | 0                      |
| Parry                | 2026-01-09         | Migrate codebase to glam + release v0.26.0 (#401)                                                                  | 88                     |
| OpenGJK              | 2025-12-29         | Enhance witness point verification with scale-aware tolerance                                                      | 29                     |
| BEPUphysics1         | 2021-08-02         | Removed event handler from ShapeChanged when ignoring shape changes. Necessary to avoid leaks when reusing shapes. | 0                      |
| JitterPhysics        | 2021-08-08         | Updated Readme: Jitter physics is back                                                                             | 0                      |
| ncollide             | 2022-03-18         | Release v0.33.0                                                                                                    | 0                      |
| collision-rs         | 2021-10-08         | fmt (#135)                                                                                                         | 0                      |
| qu3e                 | 2021-05-08         | Merge pull request #57 from wmcnamara/readme-fix                                                                   | 0                      |
| tinyc2 (tinyheaders) | 2026-01-07         | fix: Properly shutdown AudioUnit on macOS to avoid crash on exit (#419)                                            | 8                      |
| Voxblox              | 2021-11-22         | Merge pull request #385 from ethz-asl/pr-update-catkin-grpc                                                        | 0                      |

## Takeaways and recommendations for DART experimental collision

- Match or exceed feature parity: full primitive matrix, convex and mesh, heightfield, compound shapes, and optional octree or voxel support.
- Provide a complete query set: collision + contacts, signed distance + closest points, raycast, and CCD (sweep + time of impact) for all shapes.
- Offer multiple broadphase strategies (dynamic AABB tree, sweep and prune, spatial hash) with a clear default and deterministic ordering.
- Keep narrowphase specialized and fast for primitives, but retain a robust GJK/EPA path for convex and mesh.
- Keep a clean support-function interface (libccd style) to enable custom convex shapes and fast experimentation.
- Make batch query performance a first-class goal: stable IDs, cached pairs/manifolds, and benchmarks reporting queries/sec for scene-scale sweeps.
- Provide explicit batched query entry points (collideAll, distanceAll, raycastAll) to amortize overhead and enable parallelization/structure-of-arrays layouts.
- Add accelerated GJK variants and optional lower-bound distance outputs (Coal) to speed up non-colliding cases.
- Provide shape-cast APIs for linear and nonlinear motions (Parry) to cover CCD beyond simple sweeps.
- Consider cached per-pair solver objects for repeated queries (Coal ComputeCollision pattern).
- If porting or adapting libccd logic, add exhaustive unit tests (edge cases, degeneracies, iteration limits) to validate every algorithm path.
- Consider a "security margin" mode (HPP-FCL style) alongside exact geometry mode for stability vs accuracy tradeoffs.
- Add persistent manifold caching and contact reduction options to match Bullet stability while keeping exact geometry modes for accuracy.
- Support contact manifolds/patches where possible (Coal/Parry) to improve stacking stability and contact richness.
- Include SAP/persistent SAP and dynamic hierarchy options (BEPU/Jitter) and DBVT (collision-rs) to validate batch throughput tradeoffs.
- Add scene dump/replay tooling to capture hard collision bugs and share reproducible benchmarks (qu3e style).
- Add numeric robustness tests for large coordinate scales and degenerate convex features (tinyc2 notes on GJK sensitivity).
- Use external datasets (YCB, ShapeNet, ellipsoids) and broadphase benchmarks (spatial-collision-datastructures) in the structured suite.
- Standardize result conventions across all backends (normal direction, signed distance, penetration depth) to match DART Contact/DistanceResult.
- If gradients are exposed (convex or SDF), return them explicitly and test edge/vertex degeneracies for sign consistency.
- Make determinism a first-class option (stable ordering, fixed tolerances, reproducible queries) while still enabling fast paths.
- Invest in mesh robustness: edge and vertex welding, triangle adjacency hints, and BVH refit for moving meshes.
- Consider optional SDF or heightfield shapes to exceed current backends and improve large terrain handling.
- Provide instrumentation hooks (timers, pair counts, contact stats) and keep benchmarks side-by-side with FCL/Bullet/ODE.
- Keep the API small and consistent across queries to reduce adoption friction (shared option structs, consistent normal conventions).
