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
- Sources used: README files plus public headers and source trees in each repo.
- External curated list: https://github.com/jslee02/awesome-collision-detection (library catalog, not exhaustive).
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
- Caution: not battle-tested across all edge cases; community reports note algorithmic bugs; treat as a reference, not a drop-in.

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

### Batch query support (data structures and throughput)

| Library        | Batch-friendly structures                          | Typical batch usage                                            | Notes                                                                       |
| -------------- | -------------------------------------------------- | -------------------------------------------------------------- | --------------------------------------------------------------------------- |
| FCL            | Dynamic AABB tree managers, SaP/SSaP, spatial hash | Register objects once, update transforms, query many pairs     | Designed for group queries; BVH traversal overhead can dominate tiny scenes |
| Bullet         | DBVT broadphase, pair cache, persistent manifolds  | World step builds/updates pairs, narrowphase over cached pairs | Strong temporal coherence; margins trade accuracy for stability             |
| ODE            | Spaces (hash, SAP, quadtree)                       | dSpaceCollide over a space each step                           | Simple batch API; performance depends on space choice                       |
| libccd         | None                                               | Single convex pair at a time                                   | Pure narrowphase; no scene or batch layer                                   |
| Coal           | Dynamic AABB tree managers, SaP/SSaP, spatial hash | Register objects once, update transforms, query many pairs     | ComputeCollision caches solver for repeated pair queries                    |
| ReactPhysics3D | Dynamic AABB tree                                  | World step builds/updates pairs                                | Discrete pipeline; contact persistence managed by engine                    |
| Parry          | BVH partitioning utilities                         | User-managed BVH queries                                       | Broadphase orchestration is external (e.g., in Rapier)                      |
| OpenGJK        | None                                               | Single convex pair at a time                                   | Distance-only GJK kernel                                                    |

Batch here means many pairwise queries per frame; none of these expose a dedicated SIMD batch API, so throughput depends on per-call overhead and data layout.

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

### Broadphase algorithms

- FCL: Dynamic AABB tree, dynamic AABB tree array, sweep and prune (SaP/SSaP), spatial hash, interval tree, brute force.
- Bullet: DBVT (dynamic BVH), axis sweep (3D), simple broadphase; overlapping pair cache.
- ODE: simple space, hash space, sweep and prune space, quadtree space; nested spaces allowed.

### Narrowphase and mesh algorithms

- FCL: GJK and EPA (internal or libccd), specialized primitive solvers (sphere-sphere, box-box SAT, sphere-box, etc); mesh and mesh-mesh via BVH with multiple BV types (AABB, OBB, RSS, OBBRSS, kDOP, kIOS).
- Bullet: GJK/EPA (btGjkPairDetector, btGjkEpa), MPR (btMprPenetration), polyhedral contact clipping, specialized box-box; persistent manifolds; mesh via BvhTriangleMeshShape (static concave) and GImpact (dynamic concave).
- ODE: analytic primitives plus libccd for convex (GJK/EPA); triangle mesh via OPCODE or GIMPACT; ray-mesh in collision_trimesh_ray.
- libccd: GJK/EPA + MPR only, convex-only, driven by user-defined support functions.

## Wider ecosystem snapshot (awesome-collision-detection)

The curated list includes many additional libraries. Coal/HPP-FCL,
ReactPhysics3D, Parry, and OpenGJK are locally inspected above. The
following are not inspected yet, so treat them as leads to verify.

| Library       | Focus                  | Notes for DART                                                     |
| ------------- | ---------------------- | ------------------------------------------------------------------ |
| ncollide      | 3D collision/proximity | Older Rust library; may have legacy algorithms worth comparing     |
| collision-rs  | 3D collision           | Rust crate; feature list not specified in the curated list         |
| BEPUphysics   | 3D physics engine      | C# engine; potential ideas for batch throughput and cache layout   |
| JitterPhysics | 3D physics engine      | C# engine; may have simpler collision pipelines to compare         |
| qu3e          | 3D physics engine      | Minimal engine; good for reviewing simplified contact generation   |
| tinyc2        | 2D only                | Not directly relevant to DART 3D but useful for 2D algorithm ideas |

Inactive but still relevant for mesh collision history:

- GIMPACT (mesh collision, used by ODE/Bullet)
- OPCODE (legacy BVH; used by ODE)
- SOLID, ColDet (older references for mesh collision)

Related mesh processing tools listed:

- bounding-mesh (mesh approximation, convex decomposition)
- cinolib, libigl (mesh processing utilities)

Benchmarks and references listed:

- colbench / collision-detection-benchmark (HPP-FCL focused)
- spatial-collision-datastructures (broadphase data structure benchmark)

## Performance notes (architecture-informed, not benchmarked here)

- FCL: strong for mesh-heavy scenes and accurate distance/CCD, but generic templates and BVH traversal add overhead for simple primitives.
- Bullet: optimized for real-time scenes with temporal coherence (persistent manifolds, pair caches); accuracy depends on margins and contact reduction.
- ODE: simpler collision pipeline that can be fast for basic primitives; lacks distance queries and modern CCD, and mesh handling depends on external backends.
- Coal: targets high-performance GJK/EPA, includes accelerated variants and security margins; reports large speedups in its own benchmarks.
- Parry: broad query set including shape casts; performance depends on BVH usage and user-side batching.
- ReactPhysics3D: discrete pipeline with custom containers (no STL) and integrated profiling; emphasis on stable real-time steps.
- Batch throughput: FCL and Bullet favor repeated-world queries with stable object sets; ODE favors simple per-step space traversal; libccd requires a separate batch layer.

## Dev activity snapshot (local repos)

| Library        | Last commit (date) | Last commit summary                                                                               | Commits last 12 months |
| -------------- | ------------------ | ------------------------------------------------------------------------------------------------- | ---------------------- |
| FCL            | 2026-01-15         | Feat: Check for local gtest first                                                                 | 5                      |
| Bullet         | 2025-10-21         | add pyproject.toml                                                                                | 5                      |
| ODE            | 2024-01-14         | Cosmetic: Commentary spelling corrections                                                         | 0                      |
| libccd         | 2018-12-22         | Fixed division by zero in the computation of a direction vector in ccdMPRPenetration. (fixes #49) | 0                      |
| Coal (HPP-FCL) | 2026-01-16         | Merge pull request #803 from jorisv/topic/fix-pixi-toml                                           | 433                    |
| ReactPhysics3D | 2025-01-09         | Fix the documentation URL in the README.md                                                        | 0                      |
| Parry          | 2026-01-09         | Migrate codebase to glam + release v0.26.0 (#401)                                                 | 88                     |
| OpenGJK        | 2025-12-29         | Enhance witness point verification with scale-aware tolerance                                     | 29                     |

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
- Make determinism a first-class option (stable ordering, fixed tolerances, reproducible queries) while still enabling fast paths.
- Invest in mesh robustness: edge and vertex welding, triangle adjacency hints, and BVH refit for moving meshes.
- Consider optional SDF or heightfield shapes to exceed current backends and improve large terrain handling.
- Provide instrumentation hooks (timers, pair counts, contact stats) and keep benchmarks side-by-side with FCL/Bullet/ODE.
- Keep the API small and consistent across queries to reduce adoption friction (shared option structs, consistent normal conventions).
