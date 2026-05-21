# Native Collision Upstream Superset — Dev Task

## Current Status

- [x] Phase 0: Clone upstream FCL, Bullet, and ODE sources locally under
      ignored `.deps/upstream-collision-sources/`.
- [x] Phase 1: Generate the first upstream inventory with source revisions,
      source files, extracted test cases, benchmark scenarios, and explicit
      non-collision accounting.
- [x] Phase 2: Build a DART coverage map from every in-scope upstream
      correctness case to an existing DART test, new DART test, fixture
      generator, or explicit non-applicability decision. The generated row-level
      map now has 0 `mapping-needed`, `fixture-needed`, or
      `new-benchmark-needed` rows.
- [x] Phase 3: Build a benchmark coverage map from every in-scope upstream
      benchmark/demo scenario to an existing DART benchmark or new benchmark.
- [x] Phase 4: Implement correctness coverage expansion without losing existing
      reference/native coverage.
- [x] Phase 5: Implement benchmark expansion and rerun native-vs-reference
      performance optimization against the expanded benchmark packet.
      Comparable rows pass in the current run, including the main comparative,
      adapter, and raw primitive/edge-case packets. The stacked-scene packet is
      documented as native-only profiling coverage for this checkpoint, so no
      fastest-reference claim is made from those rows.
- [x] Phase 6: After performance optimization, run native collision detector
      code coverage and raise the native detector implementation to 95%+ line
      coverage. Current native line coverage is 95.1% and function coverage is
      98.0% from the native-only `coverage_native.info` extraction.
- [ ] Phase 7: Consider or implement a GUI collision-pair debugger example
      that can exercise supported native pairs, pose objects, and render
      contact point/depth/normal data. If the Filament renderer branch is used,
      do it with an explicit rebase/integration decision.
- [ ] Phase 8: Promote durable coverage artifacts to `docs/plans/` or
      `docs/onboarding/`, then delete this dev-task folder in the completion PR.

## Goal

Make DART native collision coverage a verified superset of the upstream FCL,
Bullet, and ODE test and benchmark cases that exercise collision behavior. The
end state must include both correctness expansion by tests and performance
optimization expansion by benchmarks.

## Non-Goals For Early Phases

- Do not reintroduce FCL, Bullet, or ODE as runtime collision backends.
- Do not claim completion from summary counts alone; every in-scope upstream row
  needs a DART mapping and verification evidence.
- Do not silently discard Bullet or ODE non-collision tests. Catalogue them and
  mark whether they are outside the native collision detector scope.

## Source Of Truth

- Generated inventory:
  [`01-upstream-inventory.md`](01-upstream-inventory.md)
- Inventory generator:
  [`scripts/inventory_upstream_collision_coverage.py`](../../../scripts/inventory_upstream_collision_coverage.py)
- Coverage map scaffold:
  [`02-coverage-map.md`](02-coverage-map.md)
- Generated row-level case map:
  [`03-case-map.md`](03-case-map.md) currently records 207 covered rows, 382
  not-applicable rows, 0 fixture-needed rows, 0 new-benchmark-needed rows, and 0
  mapping-needed rows.

## Key Decisions

- Upstream sources are cloned under `.deps/` because that path is ignored and
  already used for external workflow checkouts.
- The inventory records non-collision Bullet and ODE tests as accounted, not as
  native-collision completion items. If a case exercises collision behavior
  indirectly, the coverage map must promote it into the in-scope set.
- Benchmark demos are treated as scenario sources even when upstream does not
  expose them as Google Benchmark rows.
- `origin/feature/filament-gui-full-execution` exists as a possible integration
  branch for a GUI collision-pair debugger example. Rebase/integration is not
  automatic; it should happen only after the native coverage/performance scope
  has a clean reason to consume that renderer branch.

## Immediate Next Steps

1. Decide whether the raw-reference packet should become enforced after
   evaluating nanosecond-scale benchmark noise.
2. Keep new DART test and benchmark names focused on the algorithm, query, and
   edge condition, not on the upstream library that inspired the case.
3. Keep the later GUI collision-pair debugger example scoped around the native
   pair matrix, pose controls, and contact point/depth/normal rendering.
4. Promote durable artifacts out of this dev-task folder before completion,
   including the feature/performance summary and coverage evidence.
5. Regenerate the inventory and case map after any upstream clone refresh:
   `pixi run python scripts/inventory_upstream_collision_coverage.py`.

## Verification Log

- `pixi run cmake --build build/default/cpp/Debug --target test_sphere_box test_collision_world test_mesh_mesh test_ccd test_narrow_phase`
  passed after the first post-optimization coverage expansion tranche.
- `pixi run ctest --test-dir build/default/cpp/Debug --output-on-failure -R '^(test_sphere_box|test_collision_world|test_mesh_mesh|test_ccd|test_narrow_phase)$'`
  passed.
- `pixi run cmake --build build/default/cpp/Debug --target test_raycast_core test_collision_world`
  passed after adding raycast, collision-world, and invalid-handle coverage
  cases.
- `pixi run ctest --test-dir build/default/cpp/Debug --output-on-failure -R '^(test_raycast_core|test_collision_world)$'`
  passed.
- Native-only lcov extraction from `build/default/cpp/Debug/dart/collision/native`
  now reports 95.1% line coverage (8,395/8,827) and 98.0% function coverage
  (640/653).
- `pixi run cmake --build build/default/cpp/Release --target test_shapes`
  passed after adding capsule AABB aspect-ratio and convex support tie tests.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_shapes$'`
  passed.
- `pixi run cmake --build build/default/cpp/Release --target test_cylinder`
  passed after adding sphere-cylinder separated/contact frame and orientation
  cases.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_cylinder$'`
  passed.
- `pixi run cmake --build build/default/cpp/Release --target test_sphere_box`
  passed after adding sphere-box collision frame/orientation/scale cases and
  fixing
  boundary tie handling for translated/rotated box frames.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_sphere_box$'`
  passed.
- `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  passed after adding sphere-box separated nearest-point and colliding
  signed-distance frame/orientation/scale cases.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`
  passed.
- FCL capsule-box distance rows now map to exact native side, top-face, and
  rotated endpoint witness cases in `test_distance_core`.
- `pixi run lint` passed.
- `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`
  passed.
- `git diff --check` passed.
- `pixi run lint` passed after the current FCL box-box/sphere-sphere and
  signed-distance mapping batch.
- `pixi run cmake --build build/default/cpp/Release --target test_box_box test_convex test_cylinder test_distance_core test_gjk UNIT_collision_DartCollisionDetector`
  passed after lint.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_box_box|test_convex|test_cylinder|test_distance_core|test_gjk|UNIT_collision_DartCollisionDetector)$'`
  passed after lint.
- `git diff --check` passed.
- FCL cylinder-sphere and cylinder-box signed-distance regression rows now map
  to exact DART witness-validity tests in `test_distance_core`.
- ODE trimesh/sphere exact touch, heightfield ray miss, ray/convex transform,
  sphere point-depth, and box point-depth rows now map to DART native tests
  named for the query/case content.
- FCL halfspace primitive intersection rows now map to DART's one-sided
  `PlaneShape` contact tests across sphere, box, capsule, cylinder, triangle,
  and convex-support cases; FCL's two-sided plane rows are explicit
  non-applicability decisions because DART native does not expose that plane
  semantic.
- `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`
  passed.
- `pixi run cmake --build build/default/cpp/Release --target test_distance_core test_raycast_core test_mesh_mesh UNIT_collision_DartCollisionDetector`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_distance_core|test_raycast_core|test_mesh_mesh|UNIT_collision_DartCollisionDetector)$'`
  passed.
- `pixi run cmake --build build/default/cpp/Release --target test_plane`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_plane$'`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`
  passed after adding capsule-capsule degenerate center-line, parallel,
  skew-witness, and transformed signed-distance cases.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`
  passed after adding sphere-sphere signed-distance witness, swapped-order,
  large-radius, and transformed-frame cases.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_distance_core|test_gjk)$'`
  passed after adding same-frame box signed-distance and rotated box edge-face
  GJK penetration cases.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`
  passed after adding large-sphere/box signed-distance common-frame cases.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_sphere_sphere|test_capsule_capsule|test_cylinder)$'`
  passed after adding large-radius sphere-sphere, capsule-sphere, and
  equal-radius cylinder-cylinder boundary/separation/penetration cases across
  identity and common transformed frames.
- Generator broadphase pass mapped FCL broadphase update, duplicate-pair,
  empty-query, collision/self-collision, and distance/self-distance rows to
  native broadphase, world, distance, and mesh coverage anchors. Bullet OpenCL
  rows are now explicit non-applicability decisions for the current CPU native
  detector until DART adds a native GPU collision prototype.
- Generator shape pass mapped primitive representation rows to native shape
  construction/bounds tests, cone/ellipsoid rows to adapter envelope tests, and
  mass-property/topology rows to explicit native-scope non-applicability
  reasons.
- Generator GJK/EPA detail pass mapped transformed convex support to a native
  support test and classified upstream-only simplex/EPA helper rows as outside
  DART's public native API, with public GJK/EPA/MPR result tests as the
  behavioral coverage anchor.
- `pixi run cmake --build build/default/cpp/Release --target test_convex test_cylinder test_distance_core`
  passed after adding direct plane-convex collision, tetrahedron halfspace
  configurations, incompatible-scale cylinder-sphere collision cases, and
  cylinder-sphere distance witness cases.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_convex|test_cylinder|test_distance_core)$'`
  passed.
- Bullet collision unit rows now map to native sphere-sphere distance,
  GJK/EPA/MPR sphere-sphere anchors, heightmap adaptation, and heightmap
  raycast hit/miss coverage.
- `pixi run cmake --build build/default/cpp/Release --target test_distance_core UNIT_collision_DartCollisionDetector`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_distance_core|UNIT_collision_DartCollisionDetector)$'`
  passed.
- FCL box-box face-patch, contact-reduction, and edge-edge rows now map to
  geometry-focused DART tests that preserve cube-symmetry, argument-order,
  depth, normal, and contact-limit invariants.
- `pixi run cmake --build build/default/cpp/Release --target test_box_box`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_box_box$'`
  passed.
- FCL sphere-sphere separated distance rows now map to the exact four-sphere
  separated matrix in `DistanceSphereSphere.SignedDistanceWitnessesAcrossSphereSets`,
  plus direct GJK/libccd-style separation anchors.
- `pixi run cmake --build build/default/cpp/Release --target test_box_box test_distance_core test_gjk`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_box_box|test_distance_core|test_gjk)$'`
  passed.
- FCL large sphere-sphere and capsule-sphere signed-distance rows now map to
  exact native signed-distance cases in `test_distance_core`.
- `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`
  passed.
- Added dispatcher pair-order coverage for scale-sensitive sphere-box,
  cylinder-sphere, and capsule-sphere contact cases, plus capsule-sphere
  endcap collision/separation and signed-distance axis cases. These map the
  corresponding upstream large-box/sphere, large-cylinder/sphere, and
  large-capsule/sphere rows without putting upstream library names into DART
  test names.
- `pixi run cmake --build build/default/cpp/Release --target test_narrow_phase test_capsule_capsule test_distance_core`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_narrow_phase|test_capsule_capsule|test_distance_core)$'`
  passed.
- Added exact cylinder-plane halfspace depth coverage, separated mesh-mesh
  distance witnesses with swapped pair order, and a rotated box-box
  degenerate-simplex distance regression with finite witnesses. These map the
  corresponding upstream cylinder-halfspace, mesh-distance, and degenerate
  simplex rows.
- `pixi run cmake --build build/default/cpp/Release --target test_cylinder test_distance_core test_mesh_mesh`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_cylinder|test_distance_core|test_mesh_mesh)$'`
  passed.
- Added sphere-sphere and unequal-cylinder axis-sweep transition coverage,
  plus translated/rotated unequal-cylinder finite-contact penetration cases.
  These map ODE libccd sphere/cylinder GJK/MPR/EPA rows without putting
  upstream library names into DART test names.
- `pixi run cmake --build build/default/cpp/Release --target test_sphere_sphere test_cylinder`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_sphere_sphere|test_cylinder)$'`
  passed.
- Added box-box axis-sweep transition, near-touching rotation transition, and
  translated/rotated finite-contact penetration coverage. These map ODE libccd
  box-box GJK/MPR/EPA rows without putting upstream library names into DART
  test names; libccd separation-vector rows are explicit helper API
  non-applicability decisions.
- `pixi run cmake --build build/default/cpp/Release --target test_box_box`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_box_box$'`
  passed.
- Added cylinder-box translated/rotated intersection and finite-contact
  penetration coverage. These map ODE libccd box-cylinder GJK/MPR/EPA rows
  without putting upstream library names into DART test names.
- `pixi run cmake --build build/default/cpp/Release --target test_cylinder`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_cylinder$'`
  passed.
- FCL primitive-to-BVH shape/mesh consistency rows are explicit
  non-applicability decisions because they test FCL's generated BVH
  approximation pipeline, BV-family traversal, and solver-selection tolerance
  contracts rather than DART native public APIs.
- `pixi run python scripts/inventory_upstream_collision_coverage.py` updated
  the generated inventory and row map.
- `pixi run lint` passed.
- `pixi run build` passed.
- `git diff --check` passed.
- Converted Bullet OpenCL GPU kernel rows from unresolved fixture follow-up to
  explicit non-applicability decisions for the current CPU native detector
  gate; they become relevant only if DART adds a native GPU collision
  prototype.
- `pixi run python scripts/inventory_upstream_collision_coverage.py` updated
  the generated inventory and row map; no rows remain `fixture-needed`,
  `new-benchmark-needed`, or `mapping-needed`.
- Added content-named deterministic scenario benchmarks for tall box grid
  stacks, compound box structures, scaled mixed primitive stacks, elongated
  convex hull stacks, mixed primitive terrain stacks, convex terrain contact
  snapshots, 500 moving raycasts, and Halton-seeded convex cells.
- `pixi run cmake --build build/default/cpp/Release --target bm_scenarios_stacked_scenes`
  passed.
- `build/default/cpp/Release/bin/bm_scenarios_stacked_scenes --benchmark_min_time=0.001s --benchmark_filter='BM_BoxGridStack_ContactPipeline/16$|BM_CompoundBoxStructures_ContactPipeline$|BM_ScaledPrimitiveStack_ContactPipeline/8$|BM_ElongatedConvexHullStack_ContactPipeline/6$|BM_MixedPrimitiveStackWithTerrain_ContactPipeline/8$|BM_ConvexHullTerrainSnapshot_ContactPipeline/400$|BM_ConvexHullTerrainSnapshot_MovingRaycasts/400$|BM_HaltonConvexCellPack_ContactPipeline/45$'`
  completed a smoke sample for the smallest new benchmark cases.
- `pixi run python scripts/inventory_upstream_collision_coverage.py` updated
  the generated inventory and row map; no rows remain
  `new-benchmark-needed`.
- `pixi run lint` passed.
- `pixi run cmake --build build/default/cpp/Release --target bm_scenarios_stacked_scenes`
  passed after formatting.
- `build/default/cpp/Release/bin/bm_scenarios_stacked_scenes --benchmark_min_time=0.001s --benchmark_filter='BM_BoxGridStack_ContactPipeline/16$|BM_CompoundBoxStructures_ContactPipeline$|BM_ScaledPrimitiveStack_ContactPipeline/8$|BM_ElongatedConvexHullStack_ContactPipeline/6$|BM_MixedPrimitiveStackWithTerrain_ContactPipeline/8$|BM_ConvexHullTerrainSnapshot_ContactPipeline/400$|BM_ConvexHullTerrainSnapshot_MovingRaycasts/400$|BM_HaltonConvexCellPack_ContactPipeline/45$'`
  passed after formatting.
- `pixi run build` passed.
- `git diff --check` passed.
- `pixi run -e collision-reference bm-collision-check` passed the enforced
  comparative packet: 18 main native-vs-reference rows and 3 adapter rows
  passed with the configured ratio limits. The report-only raw-reference packet
  still reported five ODE-faster micro-path rows, so the blanket
  fastest-backend claim remains open.
- Added scalar native fast paths for axis-aligned sphere-sphere,
  capsule-sphere, capsule-capsule, and centered sphere-box contacts so the five
  previously reported raw-reference micro-paths now pass the current
  fastest-reference comparison.
- `pixi run cmake --build build/default/cpp/Release --target test_sphere_sphere test_sphere_box test_capsule_capsule`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_sphere_sphere|test_sphere_box|test_capsule_capsule)$'`
  passed.
- Targeted raw-reference rerun for the five previous micro-path reports passed
  with `5 passed, 0 failed, 0 skipped`.
- `pixi run -e collision-reference bm-collision-check-narrow-raw-reference`
  passed with `24 passed, 0 failed, 0 skipped`; the tightest current
  raw-reference ratio is the sphere-box grazing case at 0.973.
- `pixi run lint` passed after the primitive fast-path and documentation
  update.
- `pixi run build` passed.
- `pixi run test-unit` passed with 281/281 tests.
- `git diff --check` passed.
- `pixi run -e collision-reference -- cmake --build build/collision-reference/cpp/Release --target bm_scenarios_stacked_scenes`
  passed.
- `build/collision-reference/cpp/Release/bin/bm_scenarios_stacked_scenes --benchmark_out=.benchmark_results/collision_check_stacked_scenes.json --benchmark_out_format=json --benchmark_min_time=1ms --benchmark_repetitions=3`
  captured the current native-only baseline for the stacked-scene benchmark
  packet.
- Fixed native convex-convex penetration witness reconstruction so signed
  distance reports nearest points separated by the reported penetration depth.
- Added exact convex-convex signed-distance regression coverage for swept,
  skew, long rotated, flat octagonal, hexagonal, pentagonal, triangular, and
  dense-patch convex witness cases.
- `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`
  passed.
- `pixi run python scripts/inventory_upstream_collision_coverage.py` updated
  the generated inventory and row map.
- `pixi run lint` passed.
- `pixi run build` passed.
- `git diff --check` passed.
- Post-optimization native coverage check has started. A full-suite
  `pixi run coverage-report` attempt was interrupted after
  `UNIT_simulation_World` ran too long under instrumentation, then native-only
  extraction was captured from `build/default/cpp/Debug/dart/collision/native`.
  Initial full native scope was 84.6% line / 90.8% function coverage.
- Added targeted tests for SDF field access/query/batch/validation paths,
  collision-world object access, update stats, serial/parallel collide stats,
  filters, batch output, raycast closest/miss, supported collision and distance
  dispatch pairs, AABB-tree no-op/visitor paths, and GJK/EPA/MPR warm-start and
  degenerate cases.
- Focused verification already run:
  `pixi run cmake --build build/default/cpp/Debug --target test_sdf_compare test_collision_world test_narrow_phase`,
  `pixi run ctest --test-dir build/default/cpp/Debug --output-on-failure -R '^(test_sdf_compare|test_collision_world|test_narrow_phase)$'`,
  `pixi run cmake --build build/default/cpp/Debug --target test_aabb_tree test_gjk`, and
  `pixi run ctest --test-dir build/default/cpp/Debug --output-on-failure -R '^(test_aabb_tree|test_gjk)$'`.
- Native-only coverage extraction now reports 90.0% line coverage
  (7,945/8,827) and 96.0% function coverage (627/653). Phase 6 remains open
  until full native line coverage reaches 95%+.
- Added exact box-box signed-distance regression coverage for edge-nearest,
  thin tilted, colinear-edge, thin-plate, offset-asymmetric, expected-touching,
  and near-perpendicular witness cases, plus tilted near-touching box stacks
  across scales.
- Added exact near-boundary sphere-box signed-distance regression coverage.
- `pixi run cmake --build build/default/cpp/Release --target test_distance_core`
  passed.
- `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_distance_core$'`
  passed.
- `pixi run python scripts/inventory_upstream_collision_coverage.py` updated
  the generated inventory and row map.
- `pixi run lint` passed.
- `pixi run build` passed.
- `git diff --check` passed.
