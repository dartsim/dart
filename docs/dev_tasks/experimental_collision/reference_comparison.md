# Reference Collision Libraries: FCL, Bullet, ODE, libccd

## Scope and sources (local snapshot)

- FCL: `/home/js/dev/physics/fcl` (commit 83a2ed2, 2026-01-15; 5 commits in last 12 months)
- Bullet: `/home/js/dev/physics/bullet3` (commit 63c4d67e3, 2025-10-21; 5 commits in last 12 months)
- ODE: `/home/js/dev/physics/ODE` (commit bcfb66cd, 2024-01-14; 0 commits in last 12 months)
- libccd: `/home/js/dev/physics/libccd` (commit 7931e76, 2018-12-22; 0 commits in last 12 months)
- Sources used: README files plus public headers and source trees in each repo.
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
- Gaps: no general distance query API; limited CCD; slower recent development.

### libccd (convex-only algorithms)

- Goal: minimal C library for convex-convex queries using support functions.
- Design: GJK intersection/separation + EPA penetration; MPR intersection/penetration.
- Strengths: small, portable, easy to embed; MPR reference implementation.
- Gaps: no broadphase, no mesh or concave shapes, no raycast, no general distance.

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

### Broadphase algorithms

- FCL: Dynamic AABB tree, dynamic AABB tree array, sweep and prune (SaP/SSaP), spatial hash, interval tree, brute force.
- Bullet: DBVT (dynamic BVH), axis sweep (3D), simple broadphase; overlapping pair cache.
- ODE: simple space, hash space, sweep and prune space, quadtree space; nested spaces allowed.

### Narrowphase and mesh algorithms

- FCL: GJK and EPA (internal or libccd), specialized primitive solvers (sphere-sphere, box-box SAT, sphere-box, etc); mesh and mesh-mesh via BVH with multiple BV types (AABB, OBB, RSS, OBBRSS, kDOP, kIOS).
- Bullet: GJK/EPA (btGjkPairDetector, btGjkEpa), MPR (btMprPenetration), polyhedral contact clipping, specialized box-box; persistent manifolds; mesh via BvhTriangleMeshShape (static concave) and GImpact (dynamic concave).
- ODE: analytic primitives plus libccd for convex (GJK/EPA); triangle mesh via OPCODE or GIMPACT; ray-mesh in collision_trimesh_ray.
- libccd: GJK/EPA + MPR only, convex-only, driven by user-defined support functions.

## Performance notes (architecture-informed, not benchmarked here)

- FCL: strong for mesh-heavy scenes and accurate distance/CCD, but generic templates and BVH traversal add overhead for simple primitives.
- Bullet: optimized for real-time scenes with temporal coherence (persistent manifolds, pair caches); accuracy depends on margins and contact reduction.
- ODE: simpler collision pipeline that can be fast for basic primitives; lacks distance queries and modern CCD, and mesh handling depends on external backends.

## Dev activity snapshot (local repos)

| Library | Last commit (date) | Last commit summary                                                                               | Commits last 12 months |
| ------- | ------------------ | ------------------------------------------------------------------------------------------------- | ---------------------- |
| FCL     | 2026-01-15         | Feat: Check for local gtest first                                                                 | 5                      |
| Bullet  | 2025-10-21         | add pyproject.toml                                                                                | 5                      |
| ODE     | 2024-01-14         | Cosmetic: Commentary spelling corrections                                                         | 0                      |
| libccd  | 2018-12-22         | Fixed division by zero in the computation of a direction vector in ccdMPRPenetration. (fixes #49) | 0                      |

## Takeaways and recommendations for DART experimental collision

- Match or exceed feature parity: full primitive matrix, convex and mesh, heightfield, compound shapes, and optional octree or voxel support.
- Provide a complete query set: collision + contacts, signed distance + closest points, raycast, and CCD (sweep + time of impact) for all shapes.
- Offer multiple broadphase strategies (dynamic AABB tree, sweep and prune, spatial hash) with a clear default and deterministic ordering.
- Keep narrowphase specialized and fast for primitives, but retain a robust GJK/EPA path for convex and mesh.
- Keep a clean support-function interface (libccd style) to enable custom convex shapes and fast experimentation.
- Add persistent manifold caching and contact reduction options to match Bullet stability while keeping exact geometry modes for accuracy.
- Make determinism a first-class option (stable ordering, fixed tolerances, reproducible queries) while still enabling fast paths.
- Invest in mesh robustness: edge and vertex welding, triangle adjacency hints, and BVH refit for moving meshes.
- Consider optional SDF or heightfield shapes to exceed current backends and improve large terrain handling.
- Provide instrumentation hooks (timers, pair counts, contact stats) and keep benchmarks side-by-side with FCL/Bullet/ODE.
- Keep the API small and consistent across queries to reduce adoption friction (shared option structs, consistent normal conventions).
