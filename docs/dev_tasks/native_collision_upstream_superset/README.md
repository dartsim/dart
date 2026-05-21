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
      map now has 0 `mapping-needed` rows; fixture/prototype decisions remain
      for follow-up.
- [ ] Phase 3: Build a benchmark coverage map from every in-scope upstream
      benchmark/demo scenario to an existing DART benchmark or new benchmark.
- [ ] Phase 4: Implement correctness coverage expansion without losing existing
      reference/native coverage.
- [ ] Phase 5: Implement benchmark expansion and rerun native-vs-reference
      performance optimization against the expanded benchmark packet.
- [ ] Phase 6: After performance optimization, run native collision detector
      code coverage and raise the native detector implementation to 95%+ line
      coverage.
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
  [`03-case-map.md`](03-case-map.md) currently records 197 covered rows, 25
  fixture-needed rows, 10 new-benchmark-needed rows, 357 not-applicable rows,
  and 0 mapping-needed rows.

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

1. Convert the 25 `fixture-needed` Bullet OpenCL GPU/prototype rows into an
   explicit prototype gate decision or follow-up fixture.
2. Start converting the 10 `new-benchmark-needed` Bullet/ODE benchmark rows
   into deterministic DART benchmark fixtures.
3. Keep new DART test and benchmark names focused on the algorithm, query, and
   edge condition, not on the upstream library that inspired the case.
4. Keep the later GUI collision-pair debugger example scoped around the native
   pair matrix, pose controls, and contact point/depth/normal rendering.
5. Regenerate the inventory and case map after any upstream clone refresh:
   `pixi run python scripts/inventory_upstream_collision_coverage.py`.

## Verification Log

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
  rows now classify as `fixture-needed` prototype follow-up rows.
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
