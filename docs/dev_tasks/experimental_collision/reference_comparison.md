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

## Conventions and Terminology

### Normal direction and distance sign

| Library        | Normal convention                          | Penetration sign                    | Signed distance support      | Notes                                                                 |
| -------------- | ------------------------------------------ | ----------------------------------- | ---------------------------- | --------------------------------------------------------------------- |
| DART           | normal points from bodyNode2 to bodyNode1  | penetrationDepth positive           | yes (DistanceResult)         | Signed distance is negative when penetrating                          |
| FCL            | normal points from o1 to o2                | penetration_depth positive          | yes (enable_signed_distance) | DistanceResult is positive when separated; negative if signed enabled |
| Coal           | normal points from o1 to o2                | penetration_depth positive          | yes                          | Signed distance used to define normal/witness points                  |
| Bullet         | normalWorldOnB points from B to A          | distance1 negative when penetrating | yes (distance1)              | If B is treated as object2, normal aligns with object2->object1       |
| ODE            | normal points into body1                   | depth positive                      | no                           | Equivalent to object2->object1 if g1 is object1                       |
| Parry          | normal1 points from shape1 to shape2       | dist negative when penetrating      | yes (dist)                   | Explicit signed distance in Contact                                   |
| ReactPhysics3D | normal from body1 to body2                 | penetrationDepth positive           | no                           | Discrete collision only                                               |
| libccd         | direction is separation/penetration vector | penetration depth positive          | partial                      | Provides penetration depth + direction; no signed distance API        |
| OpenGJK        | N/A (distance-only)                        | N/A                                 | no                           | Returns minimum distance (0 if intersecting)                          |

### Contact representations and manifolds

| Library        | Contact representation              | Manifold behavior                                 |
| -------------- | ----------------------------------- | ------------------------------------------------- |
| DART           | Contact points (Contact struct)     | No explicit manifold; returns list                |
| FCL            | Contact points (pos, normal, depth) | No explicit manifold; returns list                |
| Coal           | Contact points + ContactPatch       | Supports patches (polygonal contact regions)      |
| Bullet         | Persistent manifolds of points      | Up to 4 points per manifold, cached across frames |
| ODE            | Contact points (dContactGeom)       | No explicit manifold; caller aggregates           |
| Parry          | Contact + ContactManifold           | Explicit manifolds; normals + multiple points     |
| ReactPhysics3D | ContactManifold with points         | Up to 4 points; cached for stability              |
| libccd         | Single penetration info             | No manifold; GJK/EPA gives one direction/depth    |
| OpenGJK        | Witness points from simplex         | Distance-only, no contact manifold                |

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
- Provide adapter helpers when using libraries with opposite normal conventions (FCL/Coal/Parry/React) or signed depth (Bullet/Parry).
- Keep terminology consistent: signed distance is positive when separated, penetration depth is positive overlap.

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

**Mismatch triage**:

- Re-check conventions (normal direction, signed distance, depth sign)
- Compare against analytic references for primitive pairs
- Use majority agreement across multiple backends when possible
- Document known deviations and add a follow-up task before integration

## SDF and Gradient-Based Queries

### SDF support (observed)

| Library                | SDF or voxel support            | Notes                                                                       |
| ---------------------- | ------------------------------- | --------------------------------------------------------------------------- |
| Bullet                 | btSdfCollisionShape / btMiniSDF | Signed distance fields for implicit collision (btMiniSDF exposes gradients) |
| Parry                  | Voxels                          | Voxel shapes exist; not explicitly SDF                                      |
| FCL / Coal             | None explicit                   | Focus on meshes/convex/BVH and GJK/EPA                                      |
| ODE / libccd / OpenGJK | None                            | No SDF primitives                                                           |
| ReactPhysics3D         | None                            | Discrete contact shapes only                                                |

### SDF references and implications (from curated list)

- Macklin 2020 (SDF contact) suggests robust SDF collision with local optimization and stable gradients.
- Koschier 2016 and Voxblox 2016 highlight hierarchical and incremental SDF construction pipelines.
- Xu/Barbic 2014 covers SDF generation for polygon soup meshes; relevant for mesh import.

### Gradient considerations for convex shapes

- GJK/EPA provide witness points and a separating/penetration direction; this is the natural gradient or subgradient of signed distance.
- For convex shapes, gradients are well-defined almost everywhere but can be non-unique at edges/vertices; return a stable normal with tie-breaking.
- Penetration gradients should align with the penetration direction (EPA or conservative advancement) and be consistent with the signed distance convention.
- Differentiable collision detection (randomized smoothing, Montaut 2022) can stabilize gradients near feature switches.

## Wider ecosystem snapshot (awesome-collision-detection)

Source: /home/js/dev/physics/awesome-collision-detection (commit effd3f8). The list is not exhaustive and many entries are physics engines with collision subsystems.

Active libraries already inspected above: Bullet, FCL, HPP-FCL (Coal), libccd, ODE, OpenGJK, Parry, ReactPhysics3D.

### Additional active libraries (not locally inspected here)

| Library       | Scope                  | Notes for DART                                                   |
| ------------- | ---------------------- | ---------------------------------------------------------------- |
| BEPUphysics 1 | 3D physics engine      | C# engine; potential ideas for batch throughput and cache layout |
| collision-rs  | 3D collision/proximity | Rust crate; likely shares older ncollide ideas                   |
| JitterPhysics | 3D physics engine      | C# engine; may have simpler contact pipelines to compare         |
| ncollide      | 3D collision/proximity | Predecessor of Parry; legacy algorithms and shape support        |
| qu3e          | Minimal 3D physics     | Small codebase; good for reviewing manifold generation           |
| tinyc2        | 2D collision           | 2D only; useful for test harness patterns                        |

### Legacy/inactive references (still useful)

- ColDet, SOLID: older mesh collision references.
- GIMPACT: dynamic triangle mesh collision (used by ODE/Bullet).
- OPCODE: legacy BVH/trimesh backend (used by ODE).
- OZCollide: legacy collision library.

### Mesh processing and decomposition references

- bounding-mesh: bounding mesh and convex decomposition algorithms.
- cinolib, libigl: geometry processing utilities (repair, decimation, SDF generation).

### Research and benchmarks (selected from curated list)

- GJK++ / Collision Detection Accelerated (Montaut 2022-2023): accelerated GJK and benchmarks.
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
- Standardize result conventions across all backends (normal direction, signed distance, penetration depth) to match DART Contact/DistanceResult.
- If gradients are exposed (convex or SDF), return them explicitly and test edge/vertex degeneracies for sign consistency.
- Make determinism a first-class option (stable ordering, fixed tolerances, reproducible queries) while still enabling fast paths.
- Invest in mesh robustness: edge and vertex welding, triangle adjacency hints, and BVH refit for moving meshes.
- Consider optional SDF or heightfield shapes to exceed current backends and improve large terrain handling.
- Provide instrumentation hooks (timers, pair counts, contact stats) and keep benchmarks side-by-side with FCL/Bullet/ODE.
- Keep the API small and consistent across queries to reduce adoption friction (shared option structs, consistent normal conventions).
