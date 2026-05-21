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
explicit non-applicability decisions for the current CPU native detector gate.
Mapped FCL shape representation rows to native shape construction/bounds tests
and adapter envelope checks, with explicit native-scope non-applicability
decisions for mass-property and face-topology rows. Mapped the transformed
convex support initializer row to a DART native test and classified
upstream-only simplex/EPA helper rows as outside DART's public native API.
Added direct plane-convex collision coverage with
tetrahedron halfspace configurations, and added cylinder-sphere
incompatible-scale collision plus separated distance witness coverage. Mapped
the Bullet collision unit rows to native sphere-sphere distance/GJK/MPR anchors
and heightmap hit/miss coverage. Mapped the ODE core collision rows to native
tests for zero-depth mesh/sphere shared-edge contact, heightmap ray misses,
ray/convex transform handling, and sphere/box point-depth via zero-radius
signed distance. Mapped FCL halfspace primitive intersection rows to
one-sided DART `PlaneShape` tests across sphere, box, capsule, cylinder,
triangle, and convex-support shapes, and classified FCL two-sided `Plane`
intersection rows as native-scope non-applicability decisions. Created
checkpoint commit `6a25838ad51` after lint, build, full unit tests, focused
plane tests, and whitespace checks. Mapped FCL collision-matrix, print-only
rotated box, and mesh-mesh split-method rows to existing native coverage, and
classified FCL diagnostic formatting, autodiff/constants, BVH build,
continuous spline collision, OBB predicate, front-list traversal, and deferred
BVH conversion rows as outside DART native collision public scope. Classified
FCL sphere volume, cached GJK guess, VectorN/sampler, and standalone
line/triangle/tetrahedron projection helper rows as outside native collision
detector scope. Added dispatcher pair-order coverage for scale-sensitive
sphere-box, cylinder-sphere, and capsule-sphere contacts, extended
capsule-sphere large-radius endpoint collision/separation coverage, added
capsule-sphere signed distance along the capsule axis, and mapped the upstream
large box/sphere, large cylinder/sphere, and large capsule/sphere rows to those
DART-native anchors. Added exact cylinder-plane halfspace depth coverage,
separated mesh-mesh distance witnesses with swapped pair order, and a rotated
box-box degenerate-simplex distance regression with finite witnesses, then
mapped the corresponding upstream rows plus direct sphere/triangle and
halfspace/triangle rows. Added sphere-sphere and unequal cylinder axis-sweep
collision-transition tests, added unequal-cylinder finite-contact penetration
poses, mapped the corresponding ODE libccd sphere/cylinder GJK/MPR/EPA rows,
and classified libccd polytope/vector helper rows as private-helper
non-applicability rows.
Added box-box axis-sweep, rotation-sweep, and translated/rotated finite-contact
penetration tests, mapped the corresponding ODE libccd box-box GJK/MPR/EPA
rows, and classified libccd separation-vector rows as private-helper
non-applicability rows.
Added cylinder-box translated/rotated intersection and finite-contact
penetration tests, and mapped the remaining ODE libccd box-cylinder
GJK/MPR/EPA rows to those native coverage anchors.
Classified FCL primitive-to-BVH shape/mesh consistency rows as explicit
non-applicability decisions because they target FCL's generated BVH
approximation, BV-family traversal, and solver-selection tolerance contracts
rather than DART native public APIs.
Added exact box-box and sphere-box signed-distance real-world regression tests
with finite witness validation, then mapped the corresponding FCL
RealWorldRegression1 through RealWorldRegression9 rows.
Fixed native convex-convex penetration witness reconstruction and added exact
convex-convex signed-distance regression tests for the remaining prism and
dense-patch real-world witness cases, then mapped the corresponding FCL
RealWorldRegression10 through RealWorldRegression17 rows.
Added content-named deterministic scenario benchmarks for tall box grid stacks,
compound box structures, scaled mixed primitive stacks, elongated convex hull
stacks, mixed primitive terrain stacks, convex terrain contact snapshots, 500
moving raycasts, and Halton-seeded convex cells. The generated benchmark rows
now map to DART benchmark targets; ODE libccd benchmark programs map to
existing native GJK/EPA/MPR and narrowphase benchmark rows. Added scalar
fast paths for axis-aligned sphere-sphere, capsule-sphere, capsule-capsule, and
centered sphere-box contacts so the raw primitive/edge-case benchmark packet
passes the enforced guard in the current run.

## Current Branch

`task/native-collision-performance-complete` — ahead of origin by local
checkpoint commits; latest milestones optimize native primitive fast paths,
update the feature/performance summary tables, raise native detector line
coverage above 95%, and promote durable upstream coverage artifacts into
`docs/plans/`.

## Immediate Next Step

Run lint and checkpoint the benchmark-policy update that makes the raw
primitive and adapter packets enforced under the configured `1.10` guard. Then
decide whether the next completion PR should delete this dev-task folder now
that durable artifacts live in `docs/plans/`, or start the later GUI
pair-debugger scope. The stacked-scene rows are documented as native-only
profiling coverage for this checkpoint, and the row-level map has 0
`mapping-needed`, `fixture-needed`, or `new-benchmark-needed` rows.

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
- After expanded benchmark optimization, native collision detector coverage was
  raised to 95.1% line coverage and 98.0% function coverage from the
  native-only `coverage_native.info` extraction.
- Post-optimization coverage has started. A full `pixi run coverage-report`
  run was interrupted after `UNIT_simulation_World` ran too long under coverage
  instrumentation, so native-only lcov extraction was captured from
  `build/default/cpp/Debug/dart/collision/native`.
- Initial full native coverage was 84.6% line / 90.8% function. Targeted SDF,
  collision-world, dispatcher, AABB-tree, GJK/EPA/MPR, raycast, mesh, CCD,
  narrow-phase, and invalid-handle tests raised it to 95.1% line
  (8,395/8,827) and 98.0% function (640/653). Phase 6 is satisfied for this
  checkpoint.
- The user also asked to consider a later GUI example for debugging every
  supported native collision pair with pair selection, object pose controls,
  and rendered contact point/depth/normal data.
- `origin/feature/filament-gui-full-execution` exists and may be useful for
  that example, but no rebase has been done. Rebase/integration should wait for
  an explicit renderer decision.
- `docs/plans/035-native-collision/upstream-case-map.md` is generated and
  currently reports 207 `covered` rows, 382 `not-applicable` rows, 0
  `fixture-needed` rows, 0 `new-benchmark-needed` rows, and 0
  `mapping-needed` rows.
- `pixi run -e collision-reference bm-collision-check` now enforces the main,
  adapter, and raw primitive packets under the configured `1.10` guard. A
  no-contact sphere-box early-out keeps the raw sphere-box grazing path inside
  the guard, and a sphere-sphere axis fast-path cleanup improves the raw
  sphere-sphere packet.
  Strict `1.0` report-only analysis over the official comparable packet reports
  43 native leads and 2 raw sphere-sphere edge rows that are within `1.10` but
  not strict leads.
- Focused verification for the fast paths has run:
  `pixi run cmake --build build/default/cpp/Release --target test_sphere_sphere test_sphere_box test_capsule_capsule`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_sphere_sphere|test_sphere_box|test_capsule_capsule)$'`.
- Final checkpoint validation for the fast-path batch has run:
  `pixi run lint`, `pixi run build`, `pixi run test-unit` (281/281 tests), and
  `git diff --check`.
- The stacked-scene benchmark packet is currently native-only. The latest
  baseline lives in ignored JSON at
  `.benchmark_results/collision_check_stacked_scenes.json`; add reference
  variants or document native-only scope before making fastest-backend claims
  from those rows.
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
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_box_box` and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_box_box$'`.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_cylinder`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_cylinder$'`.
- Generated inventory verification also run:
  `pixi run python scripts/inventory_upstream_collision_coverage.py`.
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
  broadphase/world/distance/mesh tests; Bullet OpenCL kernel rows are explicit
  non-applicability decisions for the current CPU native detector until DART
  adds a native GPU collision prototype.
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
- Checkpoint commit `6a25838ad51` was created after `pixi run lint`,
  `pixi run build`, `pixi run test-unit`, focused `test_plane`, and
  `git diff --check` all passed.
- FCL collision matrix rows now map to DART's native pair support/adapted-shape
  coverage, FCL `mesh_mesh` and print-only rotated box rows map to existing
  asserted native box/mesh tests, and FCL diagnostic/autodiff/constants/BVH/OBB
  predicate/front-list/continuous-spline rows are explicit non-applicability
  decisions.
- FCL sphere volume, cached GJK guess, VectorN/sampler, and standalone
  projection-helper rows are also explicit non-applicability decisions because
  they target upstream helper APIs rather than DART native collision detector
  behavior.
- FCL large box/sphere, large cylinder/sphere, and large capsule/sphere rows
  now map to DART tests named for the geometry/query behavior:
  `PrimitiveDispatcherPreservesScaleContactsUnderPairOrder`,
  `CapsuleSphere.LargeRadiusBoundaryAndSeparationAcrossFrames`, and
  `DistanceCapsuleSphere.LargeSphereSignedDistanceAlongCapsuleAxis`.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_narrow_phase test_capsule_capsule test_distance_core`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_narrow_phase|test_capsule_capsule|test_distance_core)$'`.
- FCL cylinder-halfspace, mesh-distance, degenerate-simplex, direct
  sphere/triangle, and GJK halfspace/triangle rows now map to DART tests named
  for the geometry/query behavior:
  `CylinderPlane.HalfspaceDepthAcrossAxisDirections`,
  `MeshMesh.SeparatedDistanceWitnessesAcrossPairOrder`,
  `DistanceBoxBox.DegenerateSimplexWitnessRegression`, and existing
  sphere/mesh plus halfspace/mesh anchors.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_cylinder test_distance_core test_mesh_mesh`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_cylinder|test_distance_core|test_mesh_mesh)$'`.
- FCL cone-cone, cylinder-cone, cylinder-cylinder distance, ellipsoid-ellipsoid,
  GJK shape-pair variants, and shape-pair reversibility rows now map to DART
  tests named for the behavior:
  `PrimitiveAndAdaptedConvexPairsCollideAcrossPairOrder` and
  `PrimitiveAndAdaptedConvexDistancesAcrossPairOrder`.
- FCL math/profiler helper rows are explicit non-applicability decisions
  because they target upstream vector, Morton-code, RSS, and profiler utility
  APIs rather than DART native collision detector behavior.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target UNIT_collision_CollisionBackend`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_collision_CollisionBackend$'`.
- FCL octomap collision/distance rows now map to DART's public
  `VoxelGridShape` equivalent, including occupied-cell collision, occupied-cell
  distance, adapter shape, and persistent scene mutation coverage. FCL
  octomap cost-source, primitive-id, and BV-family traversal rows are explicit
  non-applicability decisions because those FCL knobs are not exposed by DART
  native collision.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target UNIT_collision_CollisionBackend`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^UNIT_collision_CollisionBackend$'`.
- ODE libccd sphere-sphere and cylinder-cylinder GJK/MPR/EPA rows now map to
  DART tests named for the geometry/query behavior:
  `SphereSphere.AxisSweepTransitionsAcrossCoordinates`,
  `CylinderCylinder.AxisSweepTransitionsAcrossCoordinates`, and
  `CylinderCylinder.UnequalRadiusPenetrationReportsFiniteContacts`.
- Focused verification also run:
  `pixi run cmake --build build/default/cpp/Release --target test_sphere_sphere test_cylinder`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_sphere_sphere|test_cylinder)$'`.
- ODE libccd box-box and box-cylinder GJK/MPR/EPA rows now map to DART tests
  named for the geometry/query behavior:
  `BoxBox.AxisSweepTransitionsAcrossSizesAndOffsets`,
  `BoxBox.RotationSweepTransitionsAtNearTouchingOffset`,
  `BoxBox.TranslatedAndRotatedPenetrationsReportFiniteContacts`,
  `CylinderBox.TranslatedAndRotatedIntersections`, and
  `CylinderBox.TranslatedAndRotatedPenetrationsReportFiniteContacts`.
- FCL primitive-to-BVH shape/mesh consistency rows are explicit
  non-applicability decisions because DART native does not expose FCL's
  generated BVH approximation pipeline, BV-family traversal knobs, or
  libccd/GJK solver-selection tolerance contract.
- `pixi run lint`, `pixi run build`, and `git diff --check` passed for the
  primitive-to-BVH checkpoint.
- FCL box-box signed-distance real-world rows now map to exact DART tests named
  for edge-nearest, thin tilted, colinear-edge, thin-plate, offset-asymmetric,
  expected-touching, near-perpendicular, and tilted near-touching scale cases.
- FCL sphere-box signed-distance real-world row now maps to exact DART
  near-boundary witness coverage.
- Focused verification already run:
  `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`.
- `pixi run lint`, `pixi run build`, and `git diff --check` passed for the
  box/sphere signed-distance checkpoint.
- Native convex-convex signed-distance penetration now realigns witness points
  when EPA/MPR reports a finite depth but coincident points, preserving the
  contact midpoint while making nearest-point separation match `abs(distance)`.
- FCL convex-convex signed-distance real-world rows now map to DART tests named
  for swept-prism, skew-prism, long-rotated-prism, flat-octagonal-prism,
  hexagonal-prism, pentagonal-prism, triangular-prism, and dense-patch witness
  cases.
- Focused verification already run:
  `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  and
  `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`.
- `pixi run lint`, `pixi run build`, and `git diff --check` passed for this
  checkpoint.

## How To Resume

```bash
git checkout task/native-collision-performance-complete
git status --short --branch
pixi run python scripts/inventory_upstream_collision_coverage.py
```

Then continue with raw-reference optimization/scope decisions and stacked-scene
reference-variant decisions before using expanded benchmark rows for
fastest-backend claims.
