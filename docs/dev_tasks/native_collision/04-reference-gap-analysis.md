# Reference Capability Gap Analysis

This document turns the north-star plan into implementation-ready gaps. It is
not a request to keep external collision engines as runtime backends. Bullet,
FCL, and ODE are treated here as reference baselines for API shape, feature
coverage, correctness tests, and performance benchmarks only.

`01-design.md` is the canonical architecture and design contract. This file is
the implementation gap list for satisfying that contract.

## Current Built-In Inventory

The current built-in stack already has substantial native infrastructure:

- Public detector path: `DartCollisionDetector` registers the canonical `dart`
  key and keeps `experimental`, `fcl`, `fcl_mesh`, `bullet`, and `ode` as
  native-backed compatibility aliases.
- Public group path: `DartCollisionGroup` owns the DART `CollisionObject*`
  membership list used by collision, distance, and raycast calls.
- DART adapter: `adaptShape()` converts many DART `dynamics::Shape` types into
  native shapes, including primitive shapes, mesh-backed shapes, convex mesh,
  soft mesh, heightmap triangulation, cone/pyramid/ellipsoid convex
  approximations, multi-sphere convex hull approximation, and voxel-grid
  compound boxes.
- Native scene core: `native::CollisionWorld` has object handles, ECS storage,
  object IDs, broadphase selection, dirty updates, batch storage, cached
  broadphase snapshots, collision, distance, raycast, and sphere/capsule casts.
- Native broadphase choices: brute force, AABB tree, spatial hash, and
  sweep-and-prune implementations exist behind the `BroadPhase` interface.
- Native query data: `CollisionOption`, `DistanceOption`, `RaycastOption`,
  `CcdOption`, result structs, contact manifolds, and persistent manifold cache
  exist in the native layer.
- Native narrowphase: supported work includes primitives, convex support
  shapes, meshes with BVH data, SDF distance, raycast, distance, GJK/MPR paths,
  and sphere/capsule cast internals.
- Reference tests and comparative benchmarks already exist. The working tree
  isolates them with explicit CMake opt-out options for reference tests and
  reference benchmarks.
- Normal pixi configure entry points now default FCL, Bullet, ODE, reference
  tests, and reference benchmarks to `OFF`; explicit comparison jobs opt in
  with `DART_BUILD_COLLISION_*_OVERRIDE=ON`.
- Native-only installed package exports now default to the `dart` component and
  retain legacy `collision-fcl`, `collision-bullet`, and `collision-ode`
  component names only as native-backed interface facades. They no longer
  advertise old collision runtime libraries; installed metadata retains only
  `OFF` state variables for the optional reference-engine build options.
- Default and wheel Pixi dependency metadata now exclude `bullet-cpp`, `fcl`,
  `libode`, and the FCL transitive packages. The explicit
  `collision-reference` environment owns those packages and restores the
  reference component, test, and benchmark option set for comparison work.
- A repaired py312 wheel artifact built with old collision engines disabled
  imports successfully and contains no old collision component files, old
  collision CMake exports, or FCL, Bullet, ODE, or libccd runtime links.
- The Python detector compatibility names `DARTCollisionDetector`,
  `FCLCollisionDetector`, `BulletCollisionDetector`, and `OdeCollisionDetector`
  are native-backed aliases. `dartpy` no longer links legacy collision
  component targets, including in a reference-enabled build.
- Reference tests and benchmarks now call explicit `createReference()` methods
  on FCL, Bullet, and ODE detector classes when old-engine comparison behavior
  is intentional.
- Source-tree top-level FCL, Bullet, and ODE detector/group, All, PascalCase,
  and component headers are native-backed compatibility facades. Old-engine
  implementation headers and sources live under explicit
  `dart/collision/{fcl,bullet,ode}/reference/` paths used by reference tests
  and benchmarks.
- Runtime source isolation is checked by `check-collision-runtime-isolation`,
  which is wired into `lint` and `check-lint`. It fails if non-reference DART
  source paths include FCL, Bullet, ODE, libccd, or explicit collision
  reference backend headers, or if legacy engine implementation sources move
  back outside `reference/` paths.

The first DART adapter limitation is now addressed in the working tree:
`DartCollisionGroup` owns persistent native scene state, syncs dirty transforms
and shape revisions, and routes public collision, distance, and raycast calls
through that scene. Public raycast now uses native broadphase segment-AABB
candidate pruning before narrowphase, and public distance uses scene AABB
lower-bound pruning to skip narrowphase pairs that cannot beat the current best
distance. Manifold warm-starting is now keyed by scene-issued cache IDs, so
shape replacement or object removal drops stale solver impulses instead of
reusing DART object addresses. Dynamic-vertex and dynamic-element shapes now
force scene geometry resync, with soft-mesh mutation coverage proving that
in-place point-mass motion does not leave stale native mesh data behind. The
public DART collision filter now adapts into a native filter callback for
scene-owned native objects, so collision filtering is part of the native pair
check before narrowphase. Pair-order contact normal semantics are now pinned in
native and public DART tests, and the gz custom mesh/plane free-fall has been
reduced to contact normal orientation through the DART/gz pair order. The
remaining adapter limitation is recurring benchmark/profiling checks for the
public DART API path. The current gz-physics blocker is closed locally: a fresh
`pixi run -e gazebo test-gz` run passes 65/65 tests through the built-in
detector and compatibility facades. The latest fixes cap large flat box/mesh
contact patches so gz can observe more than 30 contacts without feeding
thousands into the solver, and preserve gz-required unsupported raycast
behavior on FCL/ODE compatibility facades while keeping Bullet/native raycast
support.

## Reference Capability Baseline

These capabilities define the bar the built-in layer should meet or exceed.

| Area       | Baseline capability to preserve or beat                                                                                  |
| ---------- | ------------------------------------------------------------------------------------------------------------------------ |
| API shape  | Collision worlds/managers own objects, expose group/self/cross-group queries, and keep callback/filter semantics local.  |
| Broadphase | Persistent object registration, explicit update paths, dirty-AABB handling, self-query and cross-manager query support.  |
| Filtering  | Bitmask/category filtering plus callback-level filtering before narrowphase work.                                        |
| Contact    | Contact normal, point, penetration depth, object identity, triangle/side identity where relevant, and contact limits.    |
| Manifolds  | Persistent contact/manifold data that supports stable stacking and solver warm-starting.                                 |
| Distance   | Nearest points, signed/penetration-aware semantics where supported, upper/lower-bound early exits, and error tolerances. |
| Raycast    | Closest-hit and all-hit modes, broadphase-accelerated ray traversal, hit fraction, point, normal, and filtering.         |
| Sweeps     | Convex/sphere/capsule sweep support where DART needs continuous or predictive collision behavior.                        |
| Geometry   | Primitive, convex, compound, triangle mesh, heightfield/terrain, voxel/octree-like occupancy, and DART-specific shapes.  |
| Scaling    | Large dynamic worlds should update dirty objects without full rebuilds and avoid duplicate pair testing.                 |
| Perf       | Hot paths avoid per-pair allocation, expose benchmark/profiler breakdowns, and reuse acceleration structures.            |

## Gap Matrix

| Gap                                | Current state                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | North-star target                                                                                                                                                                                                                                                                                                                                                                                                            | Next implementation step                                                                                                                  |
| ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| DART adapter scene lifecycle       | Working tree has persistent native scene state owned by `DartCollisionGroup` and public adapter-path benchmark coverage.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Public DART queries use persistent native scene state, dirty updates, cached snapshots, and broadphase pruning.                                                                                                                                                                                                                                                                                                              | Broaden shape/filter invalidation coverage and add recurring JSON regression checks.                                                      |
| Cross-group broadphase             | Working tree prunes group-vs-group collision with each group's persistent native broadphase.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Self-group and group-vs-group queries both prune with native broadphase data and avoid duplicate pair testing.                                                                                                                                                                                                                                                                                                               | Add a reusable cross-snapshot query API if the current adapter-local traversal becomes duplicated or too narrow.                          |
| Shape adapter cache                | Working tree covers primitive, mesh-scale, soft-mesh, heightmap, voxel-grid, and point-cloud invalidation/behavior tests.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Shape geometry is cached per DART collision object and invalidated only when shape data changes.                                                                                                                                                                                                                                                                                                                             | Replace conservative dynamic-vertex rebuilds with direct native dynamic-mesh updates if soft-mesh benchmarks require.                     |
| Direct DART shape coverage         | Cone, pyramid, ellipsoid, heightmap, multi-sphere, and voxel-grid are adapted; point clouds are explicitly non-collidable.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Approximations are explicit and tested; high-volume terrain/voxel cases get direct native representations if needed.                                                                                                                                                                                                                                                                                                         | Add native `HeightFieldShape` and sparse voxel acceleration if benchmarks require.                                                        |
| Native type cleanup                | Native `ShapeType` includes direct `Cone`, `HeightField`, and `PointCloud` values without direct classes.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Native shape taxonomy matches real supported code paths and does not imply unsupported direct semantics.                                                                                                                                                                                                                                                                                                                     | Either implement direct classes or remove/mark unused enum values during collision abstraction cleanup.                                   |
| Filtering integration              | Working tree adapts DART collision filters into native pair checks before narrowphase; distance/raycast filters stay public-query specific.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Filtering decisions are pushed into native pair traversal before narrowphase, with DART filters adapted cleanly.                                                                                                                                                                                                                                                                                                             | Extend the native filter adapter to distance/raycast if native query APIs gain filter hooks for those query types.                        |
| Factory and binding alias cleanup  | Working tree routes retained factory keys `fcl`, `fcl_mesh`, `bullet`, and `ode` to `DartCollisionDetector` in native-only and reference-enabled builds. Python detector compatibility names also resolve to the built-in detector, dartpy no longer links legacy collision component targets, direct public C++ legacy detector `create()` paths return native-backed `DartCollisionDetector` facades that may preserve legacy display type strings for gz-physics compatibility, retained `collision-fcl`/`collision-bullet`/`collision-ode` package components plus installed/source-tree legacy headers are native-backed facades, and C++ tests/benchmarks use explicit `reference/` includes, `createReference()` APIs, and `collision-reference-*` targets for intentional old-engine comparisons. | Any backend selected through public factory, Python compatibility names, retained public C++ legacy names, retained package component names, installed compatibility headers, or source-tree top-level compatibility headers uses the built-in detector unless the call site explicitly asks for a reference-only API/target. Legacy display strings on direct C++ facades are compatibility metadata, not engine selection. | Keep downstream migration and CI evidence aligned with the source/package facade split.                                                   |
| gz custom mesh/plane contacts      | Working tree adds native plane/mesh narrowphase dispatch, caps generated plane/mesh contacts, fixes unbounded-plane AABB transforms, and orients contact normals to public pair order. The gz `MeshAndPlane` free-fall is fixed, and fresh local `pixi run -e gazebo test-gz` passes.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | gz-physics mesh-plane and joint/contact feature tests pass through the built-in detector and compatibility facades without selecting FCL, Bullet, or ODE.                                                                                                                                                                                                                                                                    | Keep mesh/plane in focused DART/gz coverage and CI.                                                                                       |
| gz stacked cylinder contacts       | Working tree adds parallel-cylinder cap/side contact selection, axial cylinder-cap support patches against large boxes, tilted cylinder/plane-like-box support, and DART tests for stacked/support depth and normal behavior. Fresh local `pixi run -e gazebo test-gz` passes after these fixes.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Stacked cylinder/cylinder and cylinder/large-box support is stable enough for gz detachable joints and DART simulation contact workloads.                                                                                                                                                                                                                                                                                    | Keep the reduced DART solver/contact regressions and full gz gate in CI evidence.                                                         |
| gz large flat box/mesh contacts    | Working tree adds a native large-flat-box/mesh contact patch path that keeps the deepest contacts capped at 32. This preserves gz's expectation that large mesh-on-plane fallback scenes can expose more than 30 contacts before per-pair max-contact trimming, while avoiding the previous 1000-contact LCP blow-up in `StepWorld`. Native mesh tests and full local `test-gz` pass.                                                                                                                                                                                                                                                                                                                                                                                                                     | Large flat box/mesh support contact patches are deterministic, bounded, and high enough for downstream max-contact selection tests without overwhelming the solver.                                                                                                                                                                                                                                                          | Keep the cap covered by native tests and watch benchmark/solver evidence if larger contact patches need smarter reduction.                |
| gz ray-intersection compatibility  | Working tree keeps Bullet/native raycast support, keeps the legacy uppercase `DARTCollisionDetector` raycast unsupported, and now keeps FCL/ODE compatibility facades raycast-unsupported for gz's `UnsupportedRayIntersections` expectations. Full local `test-gz` passes through those facades.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Compatibility facade display strings can satisfy gz legacy behavior, but they cannot select external runtime collision engines.                                                                                                                                                                                                                                                                                              | Keep direct facade tests and downstream migration evidence aligned with the eventual lowercase/native API path.                           |
| gz joint detach residual           | Closed locally. The former `JointDetach` exact-zero off-axis velocity residual was reduced to base support contacts against gz's plane-as-large-box fallback and addressed with the retained tilted cylinder/plane-like-box support patch plus native solver regression coverage. Fresh local `pixi run -e gazebo test-gz` passes 65/65 tests.                                                                                                                                                                                                                                                                                                                                                                                                                                                            | gz-physics joint feature tests pass through the built-in detector without weakening native collision correctness.                                                                                                                                                                                                                                                                                                            | Keep `ConstraintSolver.GzPlaneBoxJointDetachKeepsSupportVelocitySymmetric` and full gz CI evidence as permanent guards.                   |
| Contact identity and warm-starting | Working tree maps DART objects to scene-issued manifold cache IDs, changes IDs when scene geometry is recreated, writes solver impulses through attached native `CachedContact` metadata even behind legacy display-name facades, and stores warm-start local points in canonical scene-cache ID order so reversed A/B query order does not break manifold refresh or matching.                                                                                                                                                                                                                                                                                                                                                                                                                           | Persistent manifolds are keyed by stable scene object IDs, canonicalized for pair-order-independent matching, and flushed when objects or shapes disappear.                                                                                                                                                                                                                                                                  | Consider moving cache ownership fully into scene/query context when the detector-level cache becomes a cleanup burden.                    |
| Distance pruning                   | Working tree skips public DART distance narrowphase pairs whose scene AABB lower bound cannot improve the best distance.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Distance queries use broadphase AABB distance ordering, upper/lower bounds, and nearest-pair early exit.                                                                                                                                                                                                                                                                                                                     | Add ordered broadphase/AABB-distance traversal if benchmarks show pair-order-preserving pruning is not enough.                            |
| Raycast scaling                    | Working tree prunes public DART raycasts with native broadphase segment-AABB candidates.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Public DART raycast uses native broadphase ray/AABB traversal, closest-hit pruning, all-hit collection, and filtering.                                                                                                                                                                                                                                                                                                       | Add ordered closest-hit traversal/profiling if benchmarks show segment-AABB pruning is too broad.                                         |
| Batch and parallel query path      | Native batch settings/stats exist, but public DART queries do not expose the optimized path.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Large worlds can run deterministic batched pair testing with reusable scratch and visible stats.                                                                                                                                                                                                                                                                                                                             | Add internal `QueryContext` with `BatchSettings`, deterministic merge, stats, and no public API churn.                                    |
| Reference harness isolation        | Working tree adds reference test/benchmark CMake options, defaults normal pixi configure paths to native-only collision, keeps explicit reference opt-in through overrides, names old-engine libraries/components as `collision-reference-*`, keeps retained `collision-*` package component names as native-backed interface facades, moves old-engine implementation files under explicit `reference/` paths, wires runtime source isolation into lint, and has focused reference-off/on, core-link, fresh runtime-link, installed package-export, Pixi lock metadata, `collision-reference` target, repaired py312 wheel artifact evidence, and wheel verifier coverage for old collision runtime artifacts.                                                                                           | Reference engines are enabled only for test/benchmark targets and can be disabled without affecting runtime targets.                                                                                                                                                                                                                                                                                                         | Collect CI wheel-matrix run/artifact evidence and downstream migration evidence so reference engines cannot leak back into runtime paths. |
| Performance guardrails             | Working tree has recurring `bm-collision-check` coverage for native-vs-reference narrowphase, distance, raycast, mixed-primitive, mesh-heavy, and raycast-batch subsets, plus public DART adapter JSON output for collision, dirty-world collision, distance, and raycast through `DartCollisionDetector`. CI Linux now has a scheduled/manual `Collision Benchmark Guard` job that runs the broad guard in the `collision-reference` environment and uploads `.benchmark_results/collision_check_*.json` artifacts.                                                                                                                                                                                                                                                                                      | Benchmarks cover primitive, mesh-heavy, mixed, raycast, distance, batch, dirty-world, and DART adapter API paths.                                                                                                                                                                                                                                                                                                            | Collect GitHub run and artifact evidence for the scheduled/manual guard, then keep extending it as new native capabilities land.          |

## Implementation Architecture For Next Step

The next implementation step should not start by deleting legacy backends. It
should first make the built-in DART path use the scalable native layer.

### 1. `DartCollisionScene`

Create an internal adapter-owned scene object in `dart/collision/dart/`:

- Owns one `native::CollisionWorld`.
- Maps each DART `CollisionObject*` to stable native and manifold-cache IDs.
- Tracks dirty object membership, transform changes, shape changes, filter
  changes, and object removals.
- Caches one deterministic `native::BroadPhaseSnapshot` per synced group.
- Owns or references the persistent manifold cache for that group's native
  object IDs.
- Exposes `sync(objects)`, `collideSelf()`, `collideAgainst(other)`,
  `distanceSelf()`, `distanceAgainst(other)`, `raycast()`, and `clear()`.

`DartCollisionGroup` should own this scene, while `DartCollisionDetector`
continues to own factory registration and compatibility behavior. This keeps
public APIs stable and prevents the detector from becoming a hidden global
world.

### 2. Native Query Snapshot API

Extend the native core only where the current snapshot API is too narrow:

- Keep `BroadPhaseSnapshot` for self-query pairs, but add enough metadata for
  cross-snapshot queries or introduce a `QuerySnapshot` that carries object IDs,
  AABBs, transforms, shape pointers, deterministic ordering, and a revision.
- Add a cross-snapshot candidate generator for group-vs-group collision and
  distance queries.
- Add ray-AABB candidate traversal so closest-hit raycasts can prune farther
  objects.
- Keep result assembly separate from traversal so public DART result ordering
  stays deterministic.

### 3. Shape And Geometry Cache

Shape adaptation should become an explicit cache with invalidation rules:

- Cache native shape data in `DartCollisionObject`.
- Add a shape revision or equivalent dirty marker when DART shape geometry
  mutates.
- Treat dynamic-vertex and dynamic-element shapes as geometry-dirty until the
  native core has a direct dynamic-mesh update path.
- Rebuild mesh BVHs, heightfield data, voxel compounds, and convex
  approximations only when their source geometry changes.
- Preserve approximation metadata for cone, ellipsoid, pyramid, and
  multi-sphere so correctness tests can distinguish expected approximation
  behavior from native query bugs.
- Add `PointCloudShape` coverage or document it as non-collidable if DART does
  not require collision behavior for it.

### 4. Filter Adapter

Filtering should be applied before expensive native work:

- Convert DART `CollisionFilter` and `DistanceFilter` decisions into native
  callbacks that can run during pair visitation.
- Preserve DART's existing semantics for self-collision, category/mask
  filtering, callback filters, and raycast filters.
- Keep fast bitmask checks in native data and call DART callbacks only after
  cheap rejection passes.

### 5. Contact And Manifold Ownership

The manifold cache should follow stable scene object identity:

- Use scene-issued object/cache IDs, not transient query objects or DART object
  addresses, as the primary cache key.
- Remove cache entries when either object leaves the scene or when incompatible
  shape geometry changes.
- Preserve cached normal/friction impulses during result conversion.
- Keep deterministic contact reduction and ordering as a solver-facing
  correctness gate.

### 6. Performance And Benchmark Hooks

The built-in architecture should make performance visible before optimization:

- Record timings for sync, broadphase update, candidate generation,
  narrowphase, result merge, and public result conversion.
- Add public-DART-path benchmarks in addition to native-core benchmarks; the
  adapter path is the one users exercise by default.
- Add dirty-world benchmarks that move a small subset of many objects.
- Add raycast and distance benchmarks that validate broadphase pruning, not only
  pairwise narrowphase speed.
- Keep reference-engine comparisons optional and keep native-only baselines for
  long-term regression tracking.

## Implementation Order

1. Done in working tree: add `DartCollisionScene` and route public
   `collide(group)` through persistent native world sync while preserving
   current results.
2. Done in working tree: route `collide(group1, group2)` through cross-scene
   candidate generation.
3. Done in working tree: public distance and raycast use persistent scene
   state; raycast has segment-AABB broadphase pruning, and distance skips
   narrowphase pairs using scene AABB lower bounds.
4. Done in working tree: move manifold cache keys to stable scene-issued IDs
   and verify shape replacement invalidates stale solver warm-start data.
5. Done in working tree: add public adapter-path benchmarks for collision,
   dirty-world collision, distance, and raycast.
6. Done in working tree: add shape-cache invalidation/behavior tests for mesh
   scale mutation, soft-mesh vertex mutation, heightmap mutation, voxel-grid
   occupancy mutation when OctoMap is enabled, and explicitly non-collidable
   point clouds.
7. Done in working tree: adapt DART collision filters into native pair checks
   before narrowphase.
8. Done in working tree: pin pair-order contact normals in native dispatch,
   collision-world snapshot dispatch, and the public DART collision group path.
9. Done in working tree: add parallel-cylinder cap/side contact selection for
   stacked support and validate it through DART tests plus the focused gz
   detachable-joint test.
10. Done in `6e04945b29d6`: add axial cylinder-cap support patches against
    large boxes and validate maximum-contact handling through native DART
    cylinder-box tests. This matches gz-physics plane geometry as represented
    by its DART plugin's large-box fallback.
11. Started in working tree: split reference tests/benchmarks from runtime
    backend CMake targets with CMake opt-out options, explicit
    `collision-reference-*` targets, native-backed retained `collision-*`
    package component facades, default/wheel Pixi dependency metadata cleanup,
    repaired py312 wheel artifact inspection, and focused validation. Finish CI
    wheel-matrix inspection before marking the gate complete.
12. Done in working tree: public factory keys `fcl`, `fcl_mesh`, `bullet`, and
    `ode` resolve to `DartCollisionDetector`, including when legacy reference
    component libraries are linked.
13. Done in working tree: Python detector compatibility names
    `DARTCollisionDetector`, `FCLCollisionDetector`, `BulletCollisionDetector`,
    and `OdeCollisionDetector` resolve to `DartCollisionDetector`, and dartpy
    does not link legacy collision component targets even in a
    reference-enabled build.
14. Done in working tree: C++ reference tests and benchmarks use explicit
    `createReference()` APIs on FCL, Bullet, and ODE detector classes.
15. Done in working tree: direct public C++ legacy detector `create()` paths
    return the built-in `DartCollisionDetector`.
16. Done in working tree: top-level source-tree FCL, Bullet, and ODE
    detector/group, All, PascalCase, and component headers are native-backed
    facades, and old-engine
    implementation files live under explicit `reference/` paths.
17. Done in working tree: runtime source isolation linting rejects old-engine
    includes from non-reference DART source paths and rejects legacy
    implementation sources outside explicit `reference/` paths.
18. Continue final CI, downstream migration, GitHub benchmark guard evidence,
    and runtime backend deletion validation.

## Ready-To-Implement Checklist

- [x] Internal `DartCollisionScene` helper exists in the DART adapter path.
- [x] `DartCollisionGroup` owns and syncs a `DartCollisionScene`.
- [x] Public collision, distance, and raycast APIs use persistent scene state.
- [x] Cross-group native collision candidate generation exists and is
      deterministic.
- [x] Public raycast uses native broadphase segment-AABB candidate pruning.
- [x] Public distance uses scene AABB lower bounds to prune narrowphase pairs
      that cannot improve the current best distance.
- [x] Shape adaptation invalidation is covered for primitive DART shape
      replacement and in-place mutation.
- [x] Shape adaptation invalidation coverage exists for mesh scale, heightmap,
      and voxel-grid mutation.
- [x] Shape adaptation invalidation coverage exists for soft mesh behavior.
- [x] `PointCloudShape` behavior is implemented or explicitly documented as
      non-collidable.
- [x] Filter adapter applies DART collision filters before narrowphase.
- [x] Manifold cache keys are stable across frames and cleaned on removal or
      scene geometry replacement.
- [x] Adapter-path benchmarks exist for collision, distance, raycast, and dirty
      worlds.
- [x] CMake reference test/benchmark opt-out options exist and focused
      reference-off/reference-on validation passes locally.
- [x] Public factory aliases for `experimental`, `fcl`, `fcl_mesh`, `bullet`,
      and `ode` create the built-in `DartCollisionDetector`.
- [x] SKEL/world tests prove legacy `fcl` and `fcl_mesh` selections resolve to
      native DART collision behavior.
- [x] Python detector compatibility names resolve to the built-in
      `DartCollisionDetector`.
- [x] dartpy does not link legacy collision component targets in native-only or
      reference-enabled builds.
- [x] Reference tests and benchmarks use explicit `createReference()` APIs for
      FCL, Bullet, and ODE detector comparisons.
- [x] Direct public C++ legacy detector `create()` entry points return
      native-backed `DartCollisionDetector` facades; legacy display type
      strings are allowed only as gz-physics compatibility metadata.
- [x] Debug, dartpy, install, coverage, ASAN, Windows, and wheel configure paths
      carry the reference test/benchmark toggles.
- [x] Normal pixi configure paths default FCL, Bullet, ODE, reference tests,
      and reference benchmarks to `OFF`.
- [x] Explicit `DART_BUILD_COLLISION_*_OVERRIDE=ON` opt-in restores reference
      component, correctness-test, and benchmark targets.
- [x] Native-only install-style core build produces `libdart.so` without FCL,
      Bullet, ODE, or libccd runtime links.
- [x] Native-only install probe installs no FCL, Bullet, ODE, libccd, or
      `libdart-collision-reference-*` libraries.
- [x] Native-only installed CMake/pkg-config metadata no longer advertises old
      collision runtime library names. Retained `collision-fcl`,
      `collision-bullet`, and `collision-ode` component names are
      native-backed interface facades; optional reference-engine build state
      variables remain and report `OFF`.
- [x] Full target-by-target runtime link and package-export inspection proves
      reference engines are absent from normal runtime targets.
- [x] Base pixi dependency metadata and wheel environment locks do not expose
      old collision engines as default runtime dependencies.
- [x] The repaired py312 wheel artifact does not bundle or link old collision
      engines as default runtime dependencies.
- [x] `wheel-verify` rejects old collision runtime libraries, reference
      collision libraries, and old FCL/Bullet/ODE collision component exports.
- [x] User-facing examples/tutorials do not require, include, select, or link
      old collision components in the default native-only build path.
- [ ] CI wheel-matrix artifacts do not bundle or link old collision engines as
      default runtime dependencies.
- [x] Reference-engine tests and benchmarks are optional CMake targets only.
- [x] Retained legacy package component names are native-backed interface
      facades, while old-engine libraries/components use explicit
      `collision-reference-*` names.
- [x] Installed legacy detector headers are native-backed facades and compile
      without FCL, Bullet, or ODE headers in native-only and reference-enabled
      installs.
- [x] Broad native-vs-reference collision benchmark JSON guard exists for
      narrowphase, distance, raycast, mixed-primitive, mesh-heavy, and
      raycast-batch subsets.
- [x] Public `DartCollisionDetector` adapter benchmark JSON guard exists for
      collision, dirty-world collision, distance, and raycast scenarios.
- [x] Source-tree legacy detector classes, headers, and old-engine source
      placement are native-backed facades or explicit reference-only surfaces.
- [x] Runtime source isolation check fails if non-reference DART source paths
      include FCL, Bullet, ODE, libccd, or explicit collision reference backend
      headers, or if legacy engine implementation sources move outside
      `reference/` paths.
- [x] Downstream migration contract documents that retained legacy runtime
      names are native-backed aliases and reference engines require explicit
      `collision-reference-*` targets plus `createReference()` APIs.
- [x] gz-physics compatibility passes locally through the built-in detector.
      The custom mesh/plane, detachable-joint, joint-feature `JointDetach`,
      transmitted-wrench, simulation-features, max-contact, and
      ray-intersection gates pass locally, and fresh `pixi run -e gazebo
    test-gz` evidence passes 65/65 tests. CI evidence remains.
- [x] Broad recurring benchmark guards cover native core and public
      `DartCollisionDetector` collision, distance, raycast, dirty-world, mesh,
      mixed-shape, and batch scenarios.
- [x] Scheduled/manual CI job runs the broad collision benchmark guard and
      uploads JSON artifacts.
- [ ] GitHub run artifacts prove the scheduled/manual benchmark guard is green
      in CI.
