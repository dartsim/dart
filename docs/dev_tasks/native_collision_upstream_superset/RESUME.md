# Resume: Native Collision Upstream Superset

## Last Session Summary

Created the long-running dev task for upstream FCL/Bullet/ODE coverage
superset work. Cloned upstream sources into `.deps/upstream-collision-sources/`,
added a generator that writes the inventory plus row-level case map, and added
native tests for capsule AABB size cases, convex support tie cases,
sphere-cylinder edge/internal configurations, and sphere-box collision
frame/rotation/incompatible-scale plus distance configurations. Added
capsule-capsule signed-distance coverage for degenerate center lines, parallel
and skew witness regions, and transformed separated/penetrating/overlapping
poses. Added sphere-sphere signed-distance coverage for separated and
penetrating small-sphere sets plus large-radius transformed cases. Added box
signed-distance coverage for same-frame public distance and rotated edge-face
GJK/MPR cases, plus large-sphere/box common-frame distance cases. Added
large-radius sphere-sphere, capsule-sphere, and equal-radius cylinder-cylinder
intersection boundary cases across identity and transformed common frames.
Mapped FCL broadphase update, duplicate-pair, empty-query,
collision/self-collision, and distance/self-distance rows to native broadphase,
world, distance, and mesh coverage anchors; Bullet OpenCL rows now classify as
prototype fixture follow-up. Mapped FCL shape representation rows to native
shape construction/bounds tests and adapter envelope checks, with explicit
native-scope non-applicability decisions for mass-property and face-topology
rows. Mapped the transformed convex support initializer row to a DART native
test and classified upstream-only simplex/EPA helper rows as outside DART's
public native API. Added direct plane-convex collision coverage with
tetrahedron halfspace configurations, and added cylinder-sphere
incompatible-scale collision plus separated distance witness coverage. Mapped
the Bullet collision unit rows to native sphere-sphere distance/GJK/MPR anchors
and heightmap hit/miss coverage. Mapped the ODE core collision rows to native
tests for zero-depth mesh/sphere shared-edge contact, heightmap ray misses,
ray/convex transform handling, and sphere/box point-depth via zero-radius
signed distance. Mapped FCL halfspace primitive intersection rows to
one-sided DART `PlaneShape` tests across sphere, box, capsule, cylinder,
triangle, and convex-support shapes, and classified FCL two-sided `Plane`
intersection rows as native-scope non-applicability decisions.

## Current Branch

`task/native-collision-performance-complete` — uncommitted changes include the
previous native collision performance completion work plus this new dev-task
setup.

## Immediate Next Step

Continue filling `docs/dev_tasks/native_collision_upstream_superset/02-coverage-map.md`
and `03-case-map.md` by mapping remaining FCL primitive-pair rows to existing
DART tests, new DART tests, or a concrete non-applicability reason. The next
high-value slices are remaining cone/ellipsoid distance rows, triangle/plane
rows, the remaining OBB/AABB broadphase row, and benchmark scenario rows.

## Context That Would Be Lost

- The user explicitly requested a superset inventory, not a small sample of
  three cases per backend.
- Upstream clones and revisions are recorded in the generated inventory:
  FCL `e5efcc41b57b2d0da3bf183480f1298a6d531f44`, Bullet
  `63c4d67e337017f9d8b298c900e9aabdb69296e7`, and ODE
  `0e0e94b59d4336be322a1f654eda0da711d28d3a`.
- Bullet and ODE contain many non-collision tests. They are catalogued so they
  are not ignored, but only collision-behavior cases should become native
  collision detector completion gates.
- New DART tests and benchmarks should be named for the algorithm, query, and
  case explanation. The upstream project stays in the coverage map source
  column; it should not be baked into new DART test or benchmark names.
- After expanded benchmark optimization is complete, run native collision
  detector code coverage and raise the native detector implementation to 95%+
  line coverage before completing the task.
- The user also asked to consider a later GUI example for debugging every
  supported native collision pair with pair selection, object pose controls,
  and rendered contact point/depth/normal data.
- `origin/feature/filament-gui-full-execution` exists and may be useful for
  that example, but no rebase has been done. Rebase/integration should wait for
  an explicit renderer decision.
- `03-case-map.md` is generated and currently reports 111 `covered` rows, 25
  `fixture-needed` rows, 157 `mapping-needed` rows, 10
  `new-benchmark-needed` rows, and 286 `not-applicable` rows.
- `collideCylinderSphere()` now handles sphere centers inside the cylinder by
  choosing the nearest cap or barrel surface; this is required by the FCL
  sphere-cylinder internal-contact cases.
- The generated inventory is intentionally large and temporary; durable
  artifacts must be promoted before deleting this folder at completion.
- Focused verification already run:
  `pixi run cmake --build build/default/cpp/Release --target test_shapes` and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_shapes$'`.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_cylinder` and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_cylinder$'`.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_sphere_box`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_sphere_box$'`.
- `collideSphereBox()` now treats near-boundary local points consistently in
  the identity/translation and rotated paths. This keeps the sphere-box
  corner-on-surface tie-break stable under translated and rotated box frames.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`.
- FCL capsule-box distance rows now map to exact native side, top-face, and
  rotated endpoint witness cases in `test_distance_core`.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`.
- `distanceSphereBox()` now returns signed penetration magnitudes for
  inside/on-boundary sphere centers using the same nearest-face tie-break as the
  collision path; this covers the sphere-box distance rows under DART's
  signed-distance contract.
- `test_distance_core` now covers capsule-capsule center-line degeneracy,
  parallel overlap/separation, skew interior/endpoint witness regions, and
  transformed capsule signed-distance configurations.
- `test_distance_core` also covers sphere-sphere signed-distance boundary
  witnesses, sign, swapped order, large-radius gaps/penetration, and common
  transformed frames.
- `test_distance_core` covers same-orientation box signed distance under
  identity and common transformed frames; `test_gjk` covers rotated box
  edge-face separation, touching, and penetration through GJK plus MPR.
- `test_distance_core` covers large-sphere/box signed distance under identity
  and common transformed frames.
- `test_sphere_sphere`, `test_capsule_capsule`, and `test_cylinder` cover
  large primitive boundary/separation/penetration cases under identity and
  common transformed frames.
- FCL broadphase update, duplicate-pair, empty-query,
  collision/self-collision, and distance/self-distance rows map to native
  broadphase/world/distance/mesh tests; Bullet OpenCL kernel rows remain
  `fixture-needed` until a GPU/prototype gate decision is made.
- FCL shape representation rows map to native shape construction/local-bound
  tests, plane half-space bounds, and cone/ellipsoid adapter envelope tests;
  FCL mass-property and convex topology rows are out of native collision shape
  scope because DART native shapes expose query geometry, not those APIs.
- FCL GJK/EPA implementation-detail rows now distinguish public DART behavior
  from upstream private helpers: transformed convex support is covered in
  `test_gjk.cpp`, while simplex extraction, simplex-region death tests, and EPA
  face/patch expansion helpers are explicit private-helper non-applicability
  rows backed by public GJK/EPA/MPR result tests.
- FCL halfspace-convex and remaining sphere-cylinder primitive rows now map to
  direct DART coverage: `test_convex.cpp` covers tetrahedron halfspace
  collision configurations, `test_cylinder.cpp` covers incompatible-scale
  cylinder-sphere collision, and `test_distance_core.cpp` covers
  cylinder-sphere separated distance witnesses.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_convex test_cylinder test_distance_core`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_convex|test_cylinder|test_distance_core)$'`.
- Bullet collision unit rows now map to DART native coverage. New anchors are
  `DistanceSphereSphere.ZeroRadiusCoincident` and
  `DartCollisionDetector.RaycastHeightmapMissesOutsideVerticalExtent`, with
  existing libccd-style sphere-sphere anchors and heightmap adapter/raycast hit
  tests.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_distance_core UNIT_collision_DartCollisionDetector`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_distance_core|UNIT_collision_DartCollisionDetector)$'`.
- FCL box-box face-patch, contact-reduction, and edge-edge rows now map to
  DART tests named for the geometry/algorithm cases:
  `CubeFacePatchSingleContactAcrossEquivalentOrientations`,
  `RotatedFacePatchReductionRespectsContactLimit`, and
  `EdgeEdgeContactAcrossEquivalentCubeOrientations`.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_box_box`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_box_box$'`.
- FCL sphere-sphere separated distance rows now map to the exact four-sphere
  separated matrix in `DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets`,
  plus direct GJK/libccd-style separation anchors.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_box_box test_distance_core test_gjk`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_box_box|test_distance_core|test_gjk)$'`.
- FCL large sphere-sphere and capsule-sphere signed-distance rows now map to
  exact native signed-distance cases in `test_distance_core`.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`.
- `pixi run lint` passed after the current FCL box-box/sphere-sphere and
  signed-distance mapping batch.
- Focused verification also run after lint:
  `pixi run cmake --build build/default/cpp/Release --target test_box_box test_convex test_cylinder test_distance_core test_gjk UNIT_collision_DartCollisionDetector`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_box_box|test_convex|test_cylinder|test_distance_core|test_gjk|UNIT_collision_DartCollisionDetector)$'`.
- `git diff --check` passed.
- FCL cylinder-sphere and cylinder-box signed-distance regression rows now map
  to exact DART witness-validity tests in `test_distance_core`.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`.
- FCL capsule-box distance rows now map to exact native side, top-face, and
  rotated endpoint witness cases in `test_distance_core`.
- `pixi run lint` passed.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`.
- `git diff --check` passed.
- ODE core collision rows now map to DART tests named for their behavior:
  `SphereTouchesSharedTriangleEdgeAtZeroDepth`,
  `RaycastHeightmapMissesOutsideGridBounds`,
  `RaycastConvex.RespectsRayAndShapeTransforms`,
  `DistanceSphereSphere.PointDepthSamplesAcrossInsideSurfaceOutside`, and
  `DistanceSphereBox.PointDepthSamplesAcrossInsideSurfaceOutside`.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_distance_core test_raycast_core test_mesh_mesh UNIT_collision_DartCollisionDetector`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_distance_core|test_raycast_core|test_mesh_mesh|UNIT_collision_DartCollisionDetector)$'`.
- FCL halfspace rows now map to
  `PlaneShape.HalfspacePrimitiveContactsAcrossTransforms`; FCL two-sided
  plane rows are `not-applicable` because DART native `PlaneShape` has
  one-sided halfspace/contact semantics.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_plane` and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_plane$'`.

## How To Resume

```bash
git checkout task/native-collision-performance-complete
git status --short --branch
pixi run python scripts/inventory_upstream_collision_coverage.py
```

Then continue the coverage-map pass with remaining primitive distance,
triangle/plane, remaining broadphase, or benchmark rows before using expanded
benchmark rows for performance claims.
