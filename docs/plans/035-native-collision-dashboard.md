# Native Collision Feature And Performance Dashboard

This is the consolidated status view for DART native collision against the FCL,
Bullet, and ODE reference engines. It is intentionally a dashboard, not the
full evidence log:

- [`035-native-collision/coverage-matrix.md`](035-native-collision/coverage-matrix.md)
  owns row-level test and benchmark coverage.
- [`035-native-collision/benchmark-manifest.md`](035-native-collision/benchmark-manifest.md)
  owns the generated local benchmark manifest for the completed performance
  wave.
- [`035-native-collision/upstream-inventory.md`](035-native-collision/upstream-inventory.md)
  owns the generated inventory of collected FCL, Bullet, and ODE source tests,
  demos, kernels, and benchmark scenarios.
- [`035-native-collision/upstream-case-map.md`](035-native-collision/upstream-case-map.md)
  owns the row-level mapping from those collected upstream cases to DART tests,
  benchmarks, fixtures, or explicit non-applicability decisions.
- `dashboard.md` owns this plan's operating state and final validation evidence.
- Local command output, CI artifacts, and the final PR description own
  per-run evidence after the dev-task folders are removed.

## Completion Contract

This plan is complete for the single PR only when all of the following are
true:

- DART native collision implements every DART-owned collision feature exercised
  by DART 6 and DART 7 tests, examples, benchmarks, tutorials, and
  compatibility surfaces that previously selected FCL, Bullet, or ODE.
- Reference engines are allowed only in explicit test and benchmark targets,
  through `createReference` or the `collision-reference` environment. They must
  not be part of the normal runtime, package, wheel, install, or dartpy API.
- Correctness parity is enforced by tests; performance parity and regressions
  are tracked by benchmarks. Any row without current performance evidence must
  be either closed before the PR or marked `DEFERRED` with a feature/performance
  rationale in
  [`035-native-collision/coverage-matrix.md`](035-native-collision/coverage-matrix.md).
- `docs/dev_tasks/native_collision/` and
  `docs/dev_tasks/experimental_collision/` must be absent before the PR is
  ready. This dashboard and the coverage matrix are the durable source of truth
  after those temporary task folders are removed.
- Completed performance-wave task folders must be removed once their durable
  evidence is promoted here and under
  [`035-native-collision/`](035-native-collision/).
- Completed upstream-superset task state must be removed once final validation
  proves the durable upstream inventory and case map are current.

## Reading Rules

- **DART native runtime** is the shipping target. The normal runtime stack must
  be the built-in `dart` detector.
- **FCL, Bullet, and ODE** are comparison references only. They must not be
  selected by normal runtime factory keys, package components, wheels, or
  dartpy.
- **Gap / action** means the remaining native-collision gap or finalization
  gate. A reference-engine limitation is called out separately when it affects
  comparability but not native runtime completeness.
- **Deferred** rows are explicit later-wave scope, mostly deeper performance,
  scale, SIMD, or native shape-specialization work.

## Status Snapshot

- **Feature coverage**
  Current status:
  [`035-native-collision/coverage-matrix.md`](035-native-collision/coverage-matrix.md)
  records 202 DONE, 0 PARTIAL, 0 GAP, and 24 DEFERRED rows.
  Gap signal: no active feature-level GAP/PARTIAL rows for this pass; deferred
  rows belong to later waves.
- **Upstream superset map**
  Current status:
  [`035-native-collision/upstream-case-map.md`](035-native-collision/upstream-case-map.md)
  records 207 covered rows and 382 not-applicable rows from the generated FCL,
  Bullet, and ODE inventory, with no mapping-needed, fixture-needed, or
  new-benchmark-needed rows.
  Gap signal: regenerate the map after any upstream clone refresh before
  accepting a refreshed superset claim.
- **Runtime selection**
  Current status: `dart` is the normal detector; retained old names are
  native-backed compatibility facades.
  Gap signal: final review-head CI and downstream evidence still open.
- **Reference engines**
  Current status: FCL/Bullet/ODE live behind explicit test and benchmark
  reference targets and `createReference`.
  Gap signal: final reference-enabled CI/package evidence still open on the
  completing review surface.
- **Performance guardrails**
  Current status:
  [`035-native-collision/benchmark-manifest.md`](035-native-collision/benchmark-manifest.md)
  records 45 comparable native leads, 0 behind rows, 24 native-only rows, and
  0 rerun-needed rows from the local single-worker benchmark packet.
  Gap signal: host CPU scaling remained enabled for the local packet; deeper
  single-CPU, data-layout, multi-core, and GPU work is later.
- **CI follow-up**
  Current status: pushed Windows reference run passed; pushed Linux reference
  run found one Release ASan leak now locally repaired.
  Gap signal: review-head CI must prove the Atlas Simbicon ownership repair
  after push.
- **Completion packaging**
  Current status: persistent dashboard/matrix and generated upstream inventory
  artifacts are promoted. The upstream-superset working docs have been removed
  after final local validation.
  Gap signal: final PR-state CI evidence remains open until the branch is
  published for review.

## Upstream Superset Completion Evidence

| Requirement                         | Durable evidence                                                                                                                                               | Current result                                                                                                                                    |
| ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| Upstream collection                 | Generated [`upstream-inventory.md`](035-native-collision/upstream-inventory.md) records FCL, Bullet, and ODE origins, revisions, scoped files, and row counts. | FCL test tree, Bullet collision/tests/examples/benchmarks/kernels, and ODE tests/demos/libccd benchmarks are catalogued.                          |
| Row categorization                  | Generated [`upstream-case-map.md`](035-native-collision/upstream-case-map.md) maps every extracted row to DART coverage or explicit non-applicability.         | 207 rows are covered, 382 rows are not applicable to the current CPU native detector or native public API, and 0 rows remain unresolved.          |
| Correctness expansion               | Coverage anchors live in `tests/unit/collision/`, `tests/integration/collision/`, and the DART/reference compatibility tests named in the generated map.       | `pixi run test-all` passed locally after the durable artifacts and status checker were added.                                                     |
| Benchmark expansion and performance | Generated [`benchmark-manifest.md`](035-native-collision/benchmark-manifest.md) owns the latest local native-vs-reference packet summary.                      | 45 comparable rows are strict native leads, 0 rows are behind, 24 rows are native-only profiling coverage, and 0 rows need rerun.                 |
| Native detector coverage            | Native-only `coverage_native.info` extraction is summarized in the coverage matrix and status snapshot.                                                        | Full `dart/collision/native` line coverage is 95.1% and function coverage is 98.0%.                                                               |
| Repeatable status gate              | `pixi run check-native-collision-superset` runs `scripts/check_native_collision_superset_status.py` and is part of `pixi run lint`.                            | The gate validates source revisions, required upstream scopes, row-map counts, unresolved statuses, and zero benchmark rows behind the reference. |

## Later GUI Debugger Scope

A native collision-pair debugger example is now tracked as
[`PLAN-037`](archive.md#plan-037-native-collision-visual-verification), with
durable rendering guidance in
[`gui-rendering.md`](../onboarding/gui-rendering.md#native-collision-visual-checks).
That follow-up keeps this completed feature/performance dashboard focused on
durable coverage and benchmark evidence while the visual verification work owns
pair-by-pair runtime controls, contact/manifold rendering, and broad-phase
overlays.

## Feature Summary

| Feature content                               | Native runtime status                                                                                                                | Comparison/reference scope                                                                                   | Remaining gate                                                                           |
| --------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------- |
| Detector factory and public runtime selection | `dart` is the canonical runtime detector; legacy names route to native-backed facades.                                               | Reference detectors are opt-in test/benchmark fixtures only.                                                 | Final review-head factory/runtime validation.                                            |
| Primitive contacts and signed distance        | Sphere, box, capsule, cylinder, plane, convex, compound, SDF, contact symmetry, and pair-order behavior are covered by native tests. | Reference rows validate parity where the query is comparable.                                                | Keep the generated upstream map and benchmark manifest synchronized before final claims. |
| Broadphase and collision groups               | Native world/group updates, deterministic pair ordering, empty scenes, self-collision, filters, and persistent scenes are covered.   | Reference group behavior remains a comparison surface.                                                       | No active upstream broadphase mapping gap; final validation remains.                     |
| Ray and cast queries                          | Native closest/all/filter raycast behavior is covered and benchmarked.                                                               | Comparable raycast references are benchmark-only.                                                            | Deeper closest-hit traversal tuning is deferred.                                         |
| Mesh, convex, and field-heavy scenes          | Mesh, convex, plane/mesh, heightmap/field adaptation, and mesh-heavy scenes are covered by tests and benchmark guards.               | Mesh-heavy references remain comparison-only.                                                                | Larger dynamic mesh and BVH scale sweeps are deferred.                                   |
| Adapter-only DART shapes                      | Cone, pyramid, ellipsoid, heightmap, multi-sphere, and voxel-grid shapes adapt into explicit native geometry.                        | Comparison engines do not define the runtime contract.                                                       | Direct heightfield/sparse voxel acceleration remains deferred.                           |
| Upstream FCL/Bullet/ODE case map              | Every generated upstream row maps to covered DART behavior or a concrete non-applicability decision.                                 | GPU/prototype kernels and non-collision tests are catalogued but outside current CPU native collision scope. | Regenerate the inventory before final task cleanup.                                      |

## Performance Summary

Ratio is native median divided by the strongest comparable median; values below
`1.0` mean native is faster in the local single-worker benchmark packet.

| Benchmark content                                                             | Comparable rows | Native result                           |     Ratio range | Current caveat                                                       |
| ----------------------------------------------------------------------------- | --------------: | --------------------------------------- | --------------: | -------------------------------------------------------------------- |
| Primitive pair narrowphase, including touching/grazing/deep-penetration cases |              27 | Leads every comparable row              | `0.015`-`0.976` | Keep recurring JSON regression checks on the final CI surface.       |
| Public adapter and package collision paths                                    |               3 | Leads every comparable row              | `0.619`-`0.894` | Keep recurring JSON regression checks on the final CI surface.       |
| Signed distance queries                                                       |               3 | Leads every comparable row              | `0.077`-`0.410` | Ordered broadphase distance traversal remains a later optimization.  |
| Ray and ray-batch queries                                                     |               7 | Leads every comparable row              | `0.104`-`0.606` | Batch/SIMD throughput is deferred.                                   |
| Mixed broadphase and pair-generation scenarios                                |               4 | Leads every comparable row              | `0.041`-`0.957` | Larger stacked/ragdoll/landscape sweeps are deferred.                |
| Mesh, convex, and field-heavy scenarios                                       |               1 | Leads the comparable row                |         `0.144` | Dynamic mesh update and BVH profiling remain later work.             |
| Native-only persistent scene and dirty-subset paths                           |               8 | Non-comparable native guardrails        |             n/a | Retain as native regression guards, not comparison claims.           |
| Native-only stacked and terrain profiling scenes                              |              16 | Non-comparable native workload coverage |             n/a | Add reference variants only if that scope becomes part of the claim. |

## Runtime Dependency Contract

- Normal runtime, package exports, and dartpy expose the native detector only.
  Legacy detector names are native-backed migration facades during the DART 7
  compatibility window.
- `scripts/check_collision_runtime_isolation.py` is the static guard that keeps
  reference-engine implementation source and third-party collision includes out
  of runtime paths.
- `scripts/audit_collision_compat_facades.py` is the static guard that keeps
  retained factory keys, C++ facade names, package components, and dartpy API
  boundaries routed to the native detector.
- Reference implementations live under `tests/dart/test/reference_collision/`
  and are consumed by opt-in test/benchmark targets only.

## DART Use-Case Inventory

This section is the pre-PR confidence view for "is every current DART collision
use case covered?" It complements the taxonomy in
[`035-native-collision/coverage-matrix.md`](035-native-collision/coverage-matrix.md)
by naming the repo surfaces that were inventoried.

- **Native collision unit tests**
  Inventory evidence: `tests/unit/collision/CMakeLists.txt` plus the
  `collision-native` CTest label; recent local validation includes a passing
  `pixi run test-all` after artifact promotion and `pixi run test-unit`
  reporting 281/281 unit tests passing.
  Coverage status: covered by native pair, algorithm, broadphase, filter,
  distance, raycast, CCD, and adapter rows.
  Remaining pre-PR gate: re-run final validation on the completing review head.
- **Legacy/reference detector unit tests**
  Inventory evidence: `test_fcl_collision_detector.cpp`,
  `test_bullet_collision_shapes.cpp`, `test_ode_*`,
  `test_reference_backends.cpp`, and `test_legacy_compat_facades.cpp`.
  Coverage status: covered as explicit reference tests or native-backed
  compatibility-facade behavior.
  Remaining pre-PR gate: reference-enabled CI evidence on the completing review
  head.
- **Collision integration tests**
  Inventory evidence: `tests/integration/collision/*` and
  `INTEGRATION_collision_native_collision_consistency`.
  Coverage status: covered by native consistency, primitive matrix,
  mesh/contact, filtering, false-contact, and ground rows.
  Remaining pre-PR gate: re-run final integration/reference gates on the
  completing review head.
- **Indirect simulation / constraint / IO tests**
  Inventory evidence: `INTEGRATION_simulation_World`, contact constraint, split
  impulse, soft dynamics, skeleton topology, and SDF/SKEL/URDF parser targets
  in the CTest inventory.
  Coverage status: covered by full local `test-all`; Atlas Simbicon ASan
  ownership repair has focused ASan/Release reruns.
  Remaining pre-PR gate: review-head CI must prove the local Atlas repair and
  parser/simulation paths together.
- **Examples and tutorials**
  Inventory evidence: `examples/CMakeLists.txt`, collision/contact examples,
  native collision visualization examples, and collision/biped/domino tutorials
  build in `pixi run test-all`.
  Coverage status: compile/build covered; runtime behavior maps to native
  unit/integration rows and gz/local validation.
  Remaining pre-PR gate: final review-head build/docs evidence; add runtime
  smoke only if a new example-only behavior appears.
- **Benchmarks**
  Inventory evidence: `bm-collision-check`, `bm_native*`, `bm_comparative*`,
  `bm_scenarios_*`, and `bm_boxes`.
  Coverage status: native/reference guardrails exist; legacy `bm_boxes` now has
  native and Bullet-reference rows.
  Remaining pre-PR gate: final PR-state benchmark artifact remains required.
- **Downstream/gz compatibility**
  Inventory evidence: local full gz-physics `ctest` inventory passed 203/203;
  `pixi run -e gazebo test-gz` is the persistent full-suite gate and the
  compatibility-facade audit keeps old names native-backed.
  Coverage status: covered locally for the current DART/gz use cases and all
  pinned gz-physics tests.
  Remaining pre-PR gate: final downstream CI evidence on the chosen review
  surface.

## Reference Superset Survey

Scope: DART-owned DART 6 and DART 7 collision surfaces that previously used
FCL, Bullet, or ODE in tests, examples, benchmarks, tutorials, downstream
compatibility, or reference-only harnesses. The matrix converts those inputs
into DART-owned feature/performance rows.

- **FCL survey**
  Inventory evidence: `test_fcl_collision_detector.cpp`,
  `test_fcl_collision_object.cpp`, `test_distance.cpp`,
  `test_collision_group.cpp`, `test_reference_backends.cpp`,
  `test_fcl_primitive_contact_matrix.cpp`,
  `test_fcl_primitive_contact_matrix_fcl.cpp`,
  `tests/dart/test/reference_collision/fcl/*`, comparative narrowphase,
  distance, and mesh-heavy benchmark rows.
  Superset categories: primitive and mesh contacts, convex/mesh adaptation,
  distance, two-group queries, filters, self-collision, object lifecycle,
  primitive contact matrix coverage, and mesh-heavy performance comparison.
  Status: comparable correctness rows are covered by native tests or explicit
  reference parity; current performance rows are present in the benchmark
  dashboard, with later scale/SIMD work marked deferred.
- **Bullet survey**
  Inventory evidence: `test_bullet_collision_shapes.cpp`,
  `test_bullet_contact.cpp`, `test_bullet_box_stack.cpp`,
  `test_bullet_cylinder_rolling.cpp`, `test_bullet_ellipsoid_rolling.cpp`,
  `test_raycast.cpp`, `test_reference_backends.cpp`,
  `tests/dart/test/reference_collision/bullet/*`, `bm_boxes.cpp`, raycast,
  raycast-batch, mixed-primitive, and mesh-heavy benchmark rows.
  Superset categories: primitive/compound/convex/mesh/heightmap adaptation,
  contact manifolds, filters, raycast closest/all/null/filter behavior,
  two-group behavior, shape cache lifecycle, stack/rolling regressions, and
  scenario performance comparison.
  Status: comparable correctness rows are covered by native tests or explicit
  reference parity; box-stack and raycast benchmark comparison rows are
  restored, with deeper scale work deferred.
- **ODE survey**
  Inventory evidence: `test_ode_cylinder_mesh.cpp`,
  `test_ode_heightmap.cpp`, `test_collision_group.cpp`,
  `test_reference_backends.cpp`, `tests/dart/test/reference_collision/ode/*`,
  and comparable integration rows that exercise old detector naming and
  unsupported-raycast compatibility.
  Superset categories: primitive contacts, cylinder/mesh regressions,
  heightmap/terrain adaptation, collision spaces/groups, filter behavior, and
  unsupported-query compatibility semantics where downstream users rely on
  them.
  Status: comparable correctness rows are covered by native tests or explicit
  reference parity; unsupported reference behavior is preserved only where it is
  an API compatibility contract, not as native runtime scope.

## Feature Matrix

- **Detector factory and default selection**
  DART native runtime: `dart` is the canonical default; `experimental`, `fcl`,
  `fcl_mesh`, `bullet`, and `ode` route to native facades.
  FCL/Bullet/ODE references: reference only through explicit test/benchmark
  APIs.
  Gap/action: final PR-head factory/runtime validation still required.
- **C++ legacy detector and package component names**
  DART native runtime: retained C++ detector/group names and `collision-*`
  package components are native-backed migration facades.
  FCL reference: old implementation stays under
  `tests/dart/test/reference_collision/fcl`.
  Bullet reference: old implementation stays under
  `tests/dart/test/reference_collision/bullet`.
  ODE reference: old implementation stays under
  `tests/dart/test/reference_collision/ode`.
  Gap/action: keep downstream migration evidence aligned with the facade split.
- **dartpy API**
  DART native runtime: clean DART 7 API exposes `DartCollisionDetector`; legacy
  detector aliases are intentionally absent.
  FCL/Bullet/ODE references: no Python runtime alias.
  Gap/action: final wheel-matrix evidence still required on the completing
  review surface.
- **Primitive narrowphase**
  DART native runtime: sphere, box, capsule, cylinder, plane, convex, compound,
  SDF, and pair-order contracts are covered.
  FCL/Bullet/ODE references: used for opt-in parity and benchmark comparison
  where available.
  Gap/action: no active feature gap; keep parity tests and JSON benchmark
  guards current.
- **Mesh and convex geometry**
  DART native runtime: mesh, convex, mesh/plane, large flat box/mesh contact
  patches, Atlas foot mesh, and mesh-heavy scenes are covered.
  FCL/Bullet references: mesh-heavy benchmark comparison is available.
  ODE reference: test/benchmark-only where comparable.
  Gap/action: smarter large-patch reduction remains a watch item if solver or
  benchmark evidence requires.
- **DART adapter-only shapes**
  DART native runtime: cone, pyramid, ellipsoid, heightmap, multi-sphere, and
  voxel grid adapt through explicit native representations.
  FCL/Bullet/ODE references: comparison-only, not the runtime contract.
  Gap/action: direct heightfield or sparse voxel acceleration is deferred until
  benchmarks require it.
- **Point clouds**
  DART native runtime: explicitly non-collidable in this feature pass, with
  adapter coverage.
  FCL/Bullet/ODE references: reference behavior does not define native runtime
  scope.
  Gap/action: direct point BVH support is deferred and tracked as a scope
  decision.
- **Persistent scene and broadphase**
  DART native runtime: `DartCollisionGroup` owns native scene state, dirty sync,
  object IDs, broadphase snapshots, and public query pruning.
  FCL/Bullet/ODE references: manager behavior is preserved only for explicit
  comparison.
  Gap/action: ordered distance/raycast traversal and broader invalidation
  coverage are performance-wave work.
- **Filtering**
  DART native runtime: DART collision filters adapt into native pair checks
  before narrowphase; distance/raycast filters stay query-specific.
  FCL/Bullet/ODE references: callback/filter semantics are comparison
  baselines.
  Gap/action: extend native filter hooks to distance/raycast if those native
  APIs grow filter callbacks.
- **Contact identity and manifolds**
  DART native runtime: scene-issued cache IDs, deterministic ordering,
  pair-order normal semantics, and warm-start flushing are covered.
  FCL/Bullet/ODE references: reference behavior informs parity tests only.
  Gap/action: consider moving remaining cache ownership fully into scene/query
  context if cleanup warrants.
- **Distance queries**
  DART native runtime: public distance uses native scene AABB lower-bound
  pruning and native narrowphase.
  FCL/Bullet/ODE references: native/reference distance benchmark coverage exists
  where available.
  Gap/action: ordered AABB-distance traversal is deferred unless benchmarks show
  the current pruning is weak.
- **Raycast queries**
  DART native runtime: native and Bullet-compatible raycast support is covered,
  including public broadphase segment-AABB pruning and double-sided
  mesh/heightmap adapter hits.
  FCL/ODE references: compatibility facades intentionally preserve unsupported
  gz legacy behavior.
  Bullet reference: comparison exists for raycast and raycast-batch benchmarks.
  Gap/action: no native raycast feature gap; FCL/ODE raycast comparison
  coverage is intentionally limited.
- **CCD and sweeps**
  DART native runtime: sphere/capsule casts plus conservative advancement have
  unit and benchmark coverage.
  FCL/Bullet/ODE references: reference engines are not normal runtime sweep
  providers.
  Gap/action: non-linear and bilateral CCD variants are deferred scope
  decisions.
- **gz-physics compatibility**
  DART native runtime: local gz passes are recorded through the built-in
  detector and compatibility facades.
  FCL reference: legacy naming behavior is preserved only where gz requires it.
  Bullet reference: Bullet/native raycast behavior stays supported.
  ODE reference: legacy unsupported raycast behavior is preserved where gz
  requires it.
  Gap/action: final downstream CI evidence on the chosen review surface remains
  open.
- **Source, package, and wheel isolation**
  DART native runtime: lint, package smoke, link scans, wheel verifier, and
  CMake exports keep old engines out of normal runtime artifacts.
  FCL/Bullet/ODE references: old-engine implementation stays
  test/benchmark-only.
  Gap/action: final package/wheel artifact evidence remains open on the
  completing review surface.

## Performance Dashboard

- **Broad collision benchmark guard**
  DART native evidence: `bm-collision-check` aggregates core and public adapter
  JSON guards, promoted through
  [`035-native-collision/benchmark-manifest.md`](035-native-collision/benchmark-manifest.md).
  Reference comparison: FCL/Bullet/ODE where the query is comparable.
  Current bar: the local single-worker packet has 45 comparable native leads,
  0 behind rows, 24 native-only rows, and 0 rerun-needed rows.
  Open performance gap: rerun the guard before accepting future performance
  claims, ideally with CPU scaling controlled.
- **Raw narrowphase**
  DART native evidence: timed rows consume outputs/contact counts and reject
  trivial no-op wins.
  Reference comparison: native raw plus reference adapter/fixed raw rows.
  Current bar: high-margin raw wins are audited with the Round 18 protocol.
  Open performance gap: keep extending raw rows when new native pair kernels
  land.
- **Public adapter collision**
  DART native evidence: `DartCollisionDetector` adapter JSON covers collision
  and dirty-world collision paths.
  Reference comparison: native/reference adapter comparison.
  Current bar: guards the user-facing path, not only native-core
  microbenchmarks.
  Open performance gap: add recurring JSON regression checks when final CI
  surface exists.
- **Distance**
  DART native evidence: `collision_check_distance.json` covers primitive
  distance and edge cases through native/reference paths.
  Reference comparison: FCL/Bullet/ODE where available.
  Current bar: public DART distance uses scene AABB lower-bound pruning before
  narrowphase.
  Open performance gap: add ordered broadphase distance traversal if current
  pruning loses scale benchmarks.
- **Raycast**
  DART native evidence: `collision_check_raycast.json` covers public native
  raycast behavior.
  Reference comparison: Bullet only in the current benchmark matrix.
  Current bar: public DART raycast uses broadphase segment-AABB pruning.
  Open performance gap: add ordered closest-hit traversal/profiling if
  segment-AABB pruning is too broad.
- **Raycast batch**
  DART native evidence: `collision_check_raycast_batch.json` covers batch-style
  raycast guardrails.
  Reference comparison: Bullet comparison.
  Current bar: batch guard exists for the current feature-level pass.
  Open performance gap: SIMD/batch throughput work remains a later performance
  wave.
- **Mixed primitive scenarios**
  DART native evidence: `collision_check_mixed.json` covers dense and sparse
  mixed primitive scenes.
  Reference comparison: native/reference comparison where comparable.
  Current bar: scenario guard catches broad integration regressions beyond
  single-pair rows.
  Open performance gap: larger stacked/ragdoll/landscape scale sweeps are
  deferred.
- **Stacked box simulation**
  DART native evidence: `bm_boxes.cpp` now contains native and Bullet-reference
  `BM_RunBoxes` rows for 8/64/512-box cube stacks.
  Reference comparison: Bullet reference row preserved for explicit comparison.
  Current bar: equal benchmark surface to the legacy box-stack benchmark.
  Open performance gap: larger ragdoll/landscape/1000+ scale sweeps remain
  later performance-wave work.
- **Mesh-heavy scenarios**
  DART native evidence: `collision_check_mesh.json` covers mesh-heavy
  native/reference scenarios with audited output consumption.
  Reference comparison: Bullet and FCL comparison.
  Current bar: mesh-heavy guard is part of the broad performance packet.
  Open performance gap: smarter dynamic mesh updates and BVH build/traversal
  metrics are deferred unless evidence demands.
- **Native broadphase implementations**
  DART native evidence: `bm_broadphase.cpp` covers add, update, overlap, and
  pair query across BruteForce, AABB tree, SAP, hash.
  Reference comparison: no direct old-engine runtime dependency required.
  Current bar: native broadphase choices are benchmarked as native
  implementation details.
  Open performance gap: N=100/1000/10000 scaling sweeps and SIMD BVH work are
  deferred.
- **Pipeline breakdown**
  DART native evidence: `bm_pipeline_breakdown.cpp` reports per-stage native
  pipeline timing; profile builds may also use `dart/common/profile.hpp`
  scopes on the public collision adapter path.
  Reference comparison: native instrumentation, not old-engine comparison.
  Current bar: provides visibility into sync, broadphase, narrowphase, and
  merge costs.
  Open performance gap: use the benchmark breakdown and profile scopes to pick
  later single-CPU and multi-core optimization work.
- **CCD / sweep benchmarks**
  DART native evidence: `bm_ccd.cpp` covers sphere casts, capsule casts, and
  conservative advancement.
  Reference comparison: reference engines are not normal runtime sweep targets.
  Current bar: native sweep paths have audited benchmark coverage.
  Open performance gap: non-linear, screw-motion, and bilateral CCD are
  deferred scope decisions.
- **Batch narrowphase**
  DART native evidence: focused batch JSON covers BoxBox, SphereSphere,
  CapsuleCapsule, CylinderCylinder, ConvexConvex, MeshMesh.
  Reference comparison: native batch vs scalar/native dispatcher rows.
  Current bar: determinism and malformed-input tests pin batch behavior.
  Open performance gap: SIMD SAT/clip speedups and "no naked single-pair loops"
  lint are future-wave rows.

## Performance Follow-Up Plan

Current PR bar: correctness parity for every comparable DART-owned collision
surface, plus best-effort benchmark coverage that makes performance visible.
Future performance work should use feature and metric names, not
reference-engine names, when setting goals.

1. Single-core CPU: reduce broadphase sync cost, ordered traversal overhead,
   narrowphase pair cost, contact manifold reduction cost, and public-adapter
   overhead across primitive, convex, mesh, distance, raycast, batch, and
   scenario rows. Prefer `dart/common/profile.hpp` scopes and benchmark JSON to
   identify the next slice before changing data structures.
2. Scenario scale: extend box-stack, mixed primitive, ragdoll/capsule,
   mesh-heavy, heightmap, and raycast-batch sweeps from smoke sizes to sustained
   large-scene sizes, with JSON guardrails for regression detection.
3. Multi-core CPU: keep deterministic ordering while parallelizing broadphase
   update/query, narrowphase dispatch, contact merge, and batched query paths.
4. GPU: treat acceleration as an optional backend for native data structures
   only, with identical CPU fallback semantics, deterministic result ordering,
   and no runtime dependency on comparison engines.
5. Cache locality and allocation: consider pooled or slab-backed storage for
   frequently recreated contact/manifold data only after profiling shows
   allocation churn dominates. A measured inline first-manifold storage
   experiment regressed one-contact raw paths, so future pool work should
   preserve small `CollisionResult` objects and gz-physics-compatible public
   behavior.

## API And Abstraction Cleanup Roadmap

- **DART 7 compatibility window**
  Keep source compatibility for gz-physics and existing downstream C++ users by
  retaining legacy detector names, factory keys, and package components as
  native-backed facades. Keep deprecation warnings enabled by default, keep
  reference implementations test/benchmark-only, and move hot paths toward
  direct native scene/query data instead of multi-detector virtual boundaries.
- **DART 7 clean-break cleanup window**
  Remove legacy detector names, factory keys, package components, and any
  multi-detector runtime abstraction that exists only to support retired
  backends. Expose a clean native collision API centered on native scenes,
  query contexts, deterministic result ordering, batch/query performance hooks,
  and explicit compatibility notes for downstream migration.

## Gap Queue

1. Final benchmark artifact on the completing review surface.
   Closure path: final `Collision Benchmark Guard` artifact and PR evidence
   packet.
2. Review-head CI proof for the local Atlas Simbicon ASan ownership repair.
   Closure path: final `CI Linux` / `Release Tests` AddressSanitizer step and
   PR evidence packet.
3. Final native-only, reference-enabled, downstream, wheel/package, and
   platform CI.
   Closure path: final PR description, final CI artifacts, and release-gate
   evidence.
4. Deferred feature/scope decisions: point BVH, OBB/RSS/k-DOP, non-linear CCD
   variants.
   Closure path:
   [`035-native-collision/coverage-matrix.md`](035-native-collision/coverage-matrix.md)
   section 2 deferred rows.
5. Deferred performance waves: scale sweeps, scenario scale,
   BVH build/traversal, SIMD.
   Closure path:
   [`035-native-collision/coverage-matrix.md`](035-native-collision/coverage-matrix.md)
   section 4 and section 4 Batch + SIMD deferred rows.
6. Final evidence transfer.
   Closure path: final PR description and final CI artifacts after explicit
   maintainer/user approval.

## Maintenance Rule

Update this dashboard whenever
[`035-native-collision/coverage-matrix.md`](035-native-collision/coverage-matrix.md)
flips a row, final benchmark/CI artifact evidence changes, or the
native-vs-reference gap status changes.
