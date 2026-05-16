# PR Evidence Transfer Draft

This file is a staging packet for the eventual PR description update. It should
be copied into the PR review surface only after the maintainer opens the
successor PR or explicitly chooses another review surface. Do not reopen PR
#2652 or create a PR from this branch just because this packet exists.

Do not treat this file as final completion evidence. The dev-task folder still
must be deleted in the same PR that completes the native-collision migration.

## Summary

- Makes the built-in DART collision detector the normal runtime collision stack.
- Keeps legacy FCL, Bullet, ODE, and `experimental` names as native-backed
  compatibility routes instead of external runtime backend selectors.
- Keeps the compatibility surface intentionally narrow: enough for gz-physics
  and downstream C++/package source compatibility, without preserving old
  backend-selection behavior.
- Treats this PR as feature-level completion: benchmark/profiling guardrails
  remain in scope, while single-CPU optimization, multi-core CPU parallelism,
  and stretch GPU support are follow-up performance work.
- Moves old external-engine access behind explicit reference-only test and
  benchmark surfaces.
- Adds source, package, wheel, downstream, benchmark, and lint guards to keep
  normal builds native-only.
- Fixes the latest native box-ground contact regression, scopes capsule-box
  duplicate filtering to the current pair, makes invalid convex/soft mesh data
  non-collidable with a warning, and adds focused raw, convex, mesh, and
  default-world regression coverage.

## Motivation / Problem

- DART historically exposed multiple collision runtime backends through public
  factory keys, package components, Python bindings, and detector classes.
- That surface carried optional external runtime dependencies into normal
  package and wheel paths.
- The native detector is intended to become the single default runtime stack,
  while FCL, Bullet, and ODE remain useful only as opt-in reference engines for
  correctness and performance comparisons.
- gz-physics and other downstream users still need a migration window where
  legacy names compile and run, but those names must no longer select old
  runtime engines.

## Changes / Key Changes

- Native collision is the default runtime path in normal CMake and Pixi builds.
- FCL, Bullet, ODE, and `experimental` factory keys route to
  `DartCollisionDetector`.
- C++ legacy detector/group surfaces are native-backed compatibility facades.
- dartpy exposes the clean DART 7 `DartCollisionDetector` API and does not
  retain legacy detector aliases.
- Legacy package components `collision-fcl`, `collision-bullet`, and
  `collision-ode` are native-backed interface facades.
- Explicit `dart-test-reference-*` targets retain opt-in access for tests and
  benchmarks.
- Public DART migration docs now state the DART 7 policy: C++ legacy collision
  names are native-backed migration facades, dartpy keeps the clean
  `DartCollisionDetector` API without legacy aliases, and FCL/Bullet/ODE
  comparison engines are opt-in test-only reference targets rather than normal
  runtime build options.
- Source-build prerequisite docs now keep FCL/Bullet/ODE out of the required
  dependency set. They are optional reference-comparison dependencies only.
- `05-downstream-migration.md` defines the downstream/deprecation evidence
  acceptance criteria that must be carried on the final PR/downstream CI
  surface before retained C++ and package facades are removed or
  hard-deprecated further.
- Runtime isolation is guarded by `check-collision-runtime-isolation`, which
  rejects non-reference old-engine includes and now also enforces pure
  top-level legacy facade headers.
- Compatibility routing is guarded by `audit-collision-compat-facades`, which
  audits factory aliases, C++ facades, package components, and absence of
  dartpy legacy detector aliases.
- Wheel/package isolation is guarded by `verify_wheel_collision_isolation.py`
  and package smoke/link checks.
- Native/reference benchmark checks cover primitive, narrow-phase, distance,
  raycast, raycast-batch, mesh-heavy, mixed-primitive, and public adapter
  surfaces.
- Native box-box contact points now stay local to the overlap for upright and
  rotated boxes on large ground boxes; invalid convex/soft mesh data is
  non-collidable with a warning, capsule-box duplicate filtering is pair-local
  for accumulated result objects, and sphere-mesh public-detector coverage is
  focused on contact detection.
- The public OctoMap include is wrapped with DART's warning-suppression helper
  so `hello_world` rebuilds without the third-party `<ciso646>` C++20 warning.
- Durable architecture notes are seeded in onboarding docs; working
  `docs/dev_tasks/native_collision/` notes remain only until final completion.

## Testing

Local validation currently recorded in the dev-task evidence:

- Latest local committed code follow-up, not yet pushed under the current
  no-PR / no-GitHub-mutation scope, contains:
  - `beea226cf8d` (`Stabilize native box-box contact points`)
  - `0839874dffb` (`Unify invalid mesh collision handling`)
  - `5e8a2c67d78` (`Fix native box-box ground contacts`)
  - `3ff258d3bf2` (`Preserve native convex mesh fallback`) — superseded by
    `0839874dffb` for invalid mesh data
  - `6853e86f9a6` (`Suppress OctoMap include warning`)
  - `bb4b48c1eff` (`Refresh native collision evidence notes`)
  - `ea48ea9a30c` (`Add sphere-sphere native batch coverage`)
  - `8bd5dd62b8a` (`Add capsule-capsule native batch coverage`)
  - `8702b83b8d3` (`Add cylinder-cylinder native batch coverage`)
  - `f300507a350` (`Add convex-convex native batch coverage`)
  - `6b81c9a2481` (`Add mesh-mesh native batch coverage`)
  - `4db514cfd22` (`Add native narrow-phase batch dispatcher`)
  - `f9ee1dd28ba` (`Label native collision stability tests`)
  - `9d9e6aeb62c` (`Add native stacked box stability coverage`)
  - `3fe8f9f7fde` (`Add native thin box stability coverage`)
  - `c5bc95e3bcf` (`Stabilize native capsule box line contacts`)
  - `bb188c616fa` (`Record native capsule stability validation`)
  - `84c7084d619` (`Add native mesh BVH build coverage`)
  - `c72447845ea` (`Add native mesh BVH transform coverage`)
  - `8ff535a8a6f` (`Add native degenerate narrow phase stress coverage`)
  - `14da49427bb` (`Add native narrow phase scale stress coverage`)
  - `c7aae13da4f` (`Add native mixed primitive stack coverage`)
  - `07645052a4b` (`Add native raw stress stability coverage`)
  - `fe2759bb1b1` (`Add native convex mesh landscape coverage`)
  - `e65c57a9770` (`Add native convex fragment stability coverage`)
  - `944bd95f874` (`Add hello world native collision regression`)
  - `ca0201e67f4` (`Add Atlas Simbicon native collision regression`)
  - `35578ad2f8a` (`Clean stale legacy collision build artifacts`)
  - `c1f03f23147` (`Scope capsule-box duplicate filtering`)
- `DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-all`
  passed on local head `4db514cfd22` with 6/6 top-level gates: linting, build,
  unit tests, simulation-experimental tests, Python tests, and documentation.
  Release CTest passed 264/264 and Python tests passed 147/147.
- The latest current-state local recheck with the same `pixi run test-all`
  command passed on head `35578ad2f8a` after docs/evidence audit hardening and
  stale legacy collision artifact cleanup on top of `ca0201e67f4`, which
  includes the native stability-label commits, stack/stress/BVH/convex-landscape
  coverage, exact `hello_world`-style no-tunneling coverage, and Atlas Simbicon
  controller-loop no-tunneling coverage. The recheck passed 6/6 top-level gates
  and printed the final `All tests passed!` report. The artifact scan reported
  only `libdart-collision-native.so`, and the package/export scan found no old
  facade-library or FCL/Bullet/ODE/libccd runtime references.
- A newer local recheck with the same `pixi run test-all` command
  passed on head `48c0cc3f90e` after docs/evidence PR-surface audit refreshes.
  The focused `hello_world` no-tunneling regression,
  Atlas Simbicon controller-loop no-tunneling regression, and
  `collision-native-stability` CTest label also passed locally. No PR, push,
  workflow, branch, or GitHub state was mutated by this recheck.
- The latest pre-record local recheck with
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  passed on head `59769b3ee58` after the current audit/evidence refresh and
  before the docs-only validation-record commit. The final report passed 6/6
  top-level gates: linting, build, unit tests, simulation-experimental tests,
  Python tests, and documentation. No PR, push, workflow, branch, or GitHub
  state was mutated by this recheck.
- The newest pre-record local recheck with the same safe-job `pixi run test-all`
  gate passed on head `6d3224426ce`
  (`Clarify native collision validation snapshot`) after the validation snapshot
  doc correction and before this evidence update. The final report passed 6/6
  top-level gates: linting, build, unit tests, simulation-experimental tests,
  Python tests, and documentation. The lint sub-gate reran
  `check-collision-runtime-isolation` and `audit-collision-compat-facades`. No
  PR, push, workflow, branch, or GitHub state was mutated by this recheck.
- The capsule-box fix head `c1f03f23147` passed focused
  `test_capsule_capsule` with the new
  `CapsuleBox.DuplicateFilteringIsPairLocal` regression, the mixed
  collision benchmark guard, the full
  `pixi run -e collision-reference bm-collision-check` guard, `pixi run lint`,
  `git diff --check`,
  `pixi run test-all`, and a fresh `pixi run -e gazebo test-gz` 65/65
  downstream compatibility gate. The gz DART plugin dependency scan again
  reported `libdart-collision-native.so`.
- The no-PR boundary audit head `b5801f6c84c` passed a fresh
  full local `pixi run test-all` using the safe local job cap. The full
  report passed all 6 top-level gates: linting, build, unit tests,
  simulation-experimental tests, Python tests, and documentation, then printed
  `All tests passed!`. The C++ unit-test phase reported 264/264 passing tests,
  including the native collision labels. No PR, push, workflow, branch, or
  GitHub state was mutated by this recheck.
- The latest branch-local full validation head `5f8c9b0204a` passed
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  with all 6 top-level gates green: linting, build, unit tests,
  simulation-experimental tests, Python tests, and documentation. The C++
  unit-test phase reported 264/264 passing tests, including
  `collision-native` and `collision-native-stability`; simulation-experimental
  reported 13/13, Python reported 147/147, documentation built successfully,
  and the report printed `All tests passed!`. No PR, push, workflow, branch,
  or GitHub state was mutated by this recheck.
- The latest current-head gz-physics refresh on pre-record head `9cc3fe99008`
  passed `pixi run -e gazebo test-gz` 65/65 against a fresh
  `gz-physics9_9.0.0` checkout. The gazebo DART install configured reference
  collision tests and benchmarks `OFF`, the DART plugin built successfully, and
  direct `readelf` inspection of the plugin showed
  `libdart-collision-native.so` without old DART collision reference,
  FCL, Bullet, ODE, or libccd runtime dependencies. No PR, push, workflow,
  branch, or GitHub state was mutated by this recheck.
- Focused local regression validation for the latest follow-up passed:
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_box_box|UNIT_collision_DartCollisionDetector|test_convex|test_mesh_mesh)$' --repeat until-fail:20`
  - `UNIT_simulation_World --gtest_filter='WorldTests.DefaultNative*BoxRestsOnGround'`
  - `UNIT_simulation_World --gtest_filter=WorldTests.DefaultNativeTenBoxStackDoesNotTunnel`
  - `UNIT_simulation_World --gtest_filter=WorldTests.DefaultNativeThinBoxDoesNotTunnel`
  - `UNIT_simulation_World --gtest_filter=WorldTests.DefaultNativeSlenderCapsuleDoesNotTunnel`
  - `UNIT_simulation_World --gtest_filter=WorldTests.DefaultNativeHelloWorldBoxDoesNotTunnel`
  - `UNIT_simulation_World`
  - `CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target INTEGRATION_simulation_World --parallel 5`
  - `INTEGRATION_simulation_World --gtest_filter=World.AtlasSimbiconControllerFeetStayAboveGroundWithNativeCollision`
  - `INTEGRATION_simulation_World`
  - `test_capsule_capsule`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -L collision-native-stability`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -L collision-native`
  - `UNIT_collision_DartCollisionDetector --gtest_filter='DartCollisionDetector.SphereMeshCollisionDetectsContact:DartCollisionDetector.ConvexMeshCollisionUsesNativeConvexGeometry:DartCollisionDetector.EmptyConvexMeshIsNotCollidable'`
  - `test_convex`
  - `test_mesh_mesh`
  - `ctest -L collision-native`
  - `CMAKE_BUILD_DIR=build/default/cpp/Release python scripts/cmake_build.py --target hello_world`
- `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run test-all`
  passed on pushed branch head `64abc65a032`
  (`Clarify native collision progress gates`)
  with 6/6 top-level gates:
  - linting
  - build
  - unit tests
  - simulation-experimental tests
  - Python tests
  - documentation
- Default configure kept reference tests and reference benchmarks `OFF` and
  exposed no per-engine FCL/Bullet/ODE collision build switches, confirming
  old-engine comparison components are not required by core DART, dartpy,
  gz-physics runtime integration, or native-backed compatibility facades.
- `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run -e collision-reference -- ctest --test-dir build/collision-reference/cpp/Release --output-on-failure -R '^test_reference_backends$' -j 5`
  passed 1/1 after the `collision-reference` configure path reported reference
  tests and reference benchmarks `ON` and configured all three FCL, Bullet, and
  ODE test-only reference targets.
- Current local `collision-reference` validation on head `4db514cfd22` passed:
  focused `test_reference_backends` plus
  `INTEGRATION_collision_native_backend_consistency` 2/2, the non-simulation
  CTest sweep 288/288, the dedicated simulation-experimental task 13/13, and
  the final unfiltered CTest sweep 301/301.
- Local commits after `ca0201e67f4` are docs/evidence-only refreshes. Recent
  audit commits include `19aed78ae04` for current local `test-all` evidence and
  `d1a1dbd9ac9` for read-only review-surface state; later packet refresh commits
  may move `HEAD`. The latest local build-surface cleanup commit
  `35578ad2f8a` removes stale legacy collision build artifacts and export
  snippets from reused build directories. `pixi run lint` has passed after the
  latest code update, including `check-collision-runtime-isolation` and
  `audit-collision-compat-facades`. Use `git log -3 --oneline --decorate` for
  the exact current local head.
- Latest docs-only reference-surface cleanup corrected stale public
  `collision-reference-*` component wording to the current
  `dart-test-reference-*` target/dependency wording. `pixi run lint` passed
  after that cleanup, including runtime-isolation and compatibility-facade
  audits, and current-facing stale wording searches returned no matches. A
  read-only review-surface audit found no workflow runs and only closed PR #2652
  on old head `714d220d82a`.
- `pixi run lint` passed and includes:
  - `check-collision-runtime-isolation`
  - `audit-collision-compat-facades`
- `pixi run check-lint` passed and verifies the same audit guards through the
  non-mutating lint path.
- `python scripts/check_collision_runtime_isolation.py` passed.
- `python scripts/audit_collision_compat_facades.py` passed.
- Current clean-API validation passed locally:
  - `pixi run lint`
  - `pixi run build`
  - `pixi run test-unit` (277/277 CTest tests passed)
  - `pixi run test-py` (147 Python tests passed)
- Current local downstream evidence:
  - fresh `pixi run -e gazebo test-gz` passed 65/65 tests with the DART
    install configured as native-only with reference tests and benchmarks `OFF`
    and no per-engine FCL/Bullet/ODE collision build switches
  - native compatibility package smoke passed against the same native-only
    install, including installed `collision-fcl`, `collision-bullet`, and
    `collision-ode` package component facades
  - `readelf` showed gz/plugin package-smoke binaries depending on
    `libdart-collision-native.so` without old collision/reference runtime
    dependencies
  - earlier combined local baseline `6404f7607be` passed gz-physics, the
    native compatibility package smoke, and direct `readelf` checks after the
    matrix deferral audit; no PR, push, or GitHub state was mutated for that
    refresh
  - current gz-physics refresh head `6742a21ab0f` reran
    `pixi run -e gazebo test-gz` after the current validation evidence commit;
    gz-physics passed 65/65, and an explicit plugin dependency scan reported
    `libdart-collision-native.so` without old DART collision reference/facade,
    FCL, Bullet, ODE, or libccd matches
  - current-head gz-physics refresh head `9cc3fe99008` reran
    `pixi run -e gazebo test-gz`; gz-physics passed 65/65, and direct
    `readelf` inspection showed `libdart-collision-native.so` without old DART
    collision reference, FCL, Bullet, ODE, or libccd runtime dependencies
  - current package-smoke refresh head `dcfc994542f` reran the native
    compatibility package smoke after the gz-physics evidence update; retained
    `collision-fcl`, `collision-bullet`, and `collision-ode` package components
    resolved as native-backed facades, and `readelf` showed
    `libdart-collision-native.so` without old collision/reference runtime
    dependencies
- Prior workflow-dispatch reference evidence on pushed head `1e1faf6feb1`:
  - native-only CI passed
  - gz-physics CI passed
  - full dartpy wheel matrix passed
  - collision benchmark guard uploaded
    `collision-benchmark-guard-25887939088-1`
- Recent local benchmark evidence recorded in `4b155655890`:
  - `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run -e collision-reference bm-collision-check`
    passed.
  - Checked subsets passed for narrow phase, distance, raycast, mixed
    primitives, mesh-heavy, and raycast-batch; the public DART adapter
    benchmark also completed and wrote JSON.
- Latest docs-policy validation on pushed head `d489292bdf1`:
  - `pixi run lint` passed after adding the public migration guide language and
    downstream deprecation evidence acceptance criteria.
  - Lint configured the default build with reference tests and reference
    benchmarks `OFF`, then reran `check-collision-runtime-isolation` and
    `audit-collision-compat-facades`.
- Latest local full-validation refresh on pushed head `64abc65a032`:
  - `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run test-all`
    passed with 6/6 top-level gates: linting, build, unit tests,
    simulation-experimental tests, Python tests, and documentation.
  - The C++ Release CTest pass completed 264/264 tests, and Python tests
    completed 147/147 tests.
- Latest source-build prerequisite docs validation on pushed head
  `621fca5a1fb`:
  - `pixi run lint` passed after moving FCL out of required source-build
    prerequisites and recording FCL/Bullet/ODE as optional reference-comparison
    dependencies.
  - Lint reran `check-collision-runtime-isolation` and
    `audit-collision-compat-facades`.
- Later pushed audit/docs/policy heads have no attached GitHub Actions runs
  unless the maintainer chooses a PR or manual workflow surface, because
  `feature/new_coll` pushes do not match the workflow `push` filters.

Final PR-complete testing still needs whatever CI surface the maintainer chooses
after the maintainer-opened successor PR or another explicit review surface
exists.

Final CI closure map for the maintainer-selected surface:

| Open gate                    | Workflow / job evidence                                                                                                                                     |
| ---------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Lint and docs                | `CI Lint` / `Lint`; `CI Lint` / `Documentation`                                                                                                             |
| Native-only runtime          | `CI Linux` / `Native Collision (no FCL/Bullet/ODE)`; `CI Linux` / `Release Tests`; `CI Linux` / `Debug Tests`; `CI Linux` / `Asserts enabled (no -DNDEBUG)` |
| Collision benchmark artifact | `CI Linux` / `Collision Benchmark Guard` with uploaded `collision-benchmark-guard-*` JSON artifact                                                          |
| gz-physics downstream        | `CI gz-physics` / `GZ Physics Tests`                                                                                                                        |
| dartpy wheels                | `Publish dartpy` / `Wheels \| ${{ matrix.os }} Py${{ matrix.python-version }}`                                                                              |
| Platform coverage            | `CI macOS` / `Release Tests (arm64)`; `CI macOS` / `Debug Tests (arm64)`; `CI Windows` / `Tests (Release)`                                                  |

## Breaking Changes

- [x] Dartpy no longer exposes legacy detector aliases
      `DARTCollisionDetector`, `FCLCollisionDetector`, `BulletCollisionDetector`,
      or `OdeCollisionDetector`. Use `DartCollisionDetector` or the default
      detector.

Compatibility notes:

- Normal runtime backend selection changes to the built-in `dart` detector even
  when retained legacy names are used.
- Retained legacy names are compatibility facades, not external runtime backend
  selectors.
- Downstream C++ runtime code should migrate to `dart`, `DartCollisionDetector`,
  `CollisionDetectorType::Dart`, and the default `dart` package component.
- Downstream Python runtime code should migrate directly to
  `DartCollisionDetector`; DART 7 does not keep dartpy detector aliases.
- Explicit old-engine comparisons should use `dart-test-reference-*` targets
  and `createReference()` APIs.

## Related Issues / PRs (backports)

- PR #2652: `https://github.com/dartsim/dart/pull/2652`
- Latest audited branch evidence:
  - `beea226cf8d` (`Stabilize native box-box contact points`) — local only;
    kept under the current no-push/no-PR scope.
  - `0839874dffb` (`Unify invalid mesh collision handling`) — local only;
    kept under the current no-push/no-PR scope.
  - `bb4b48c1eff` (`Refresh native collision evidence notes`) — local only;
    kept under the current no-push/no-PR scope.
  - `6853e86f9a6` (`Suppress OctoMap include warning`) — local only.
  - `3ff258d3bf2` (`Preserve native convex mesh fallback`) — local only;
    superseded by `0839874dffb` for invalid mesh data.
  - `5e8a2c67d78` (`Fix native box-box ground contacts`) — local only.
  - `ec6f6f43112` (`Clean dartpy collision API and deprecate C++ facades`)
  - `aa3ccce70c7` (`Clarify collision reference build options`)
  - `06cd27d0163` (`Rename collision reference build options`)
  - `376fd5e686d` (`Remove per-engine collision reference build options`)
  - `64abc65a032` (`Clarify native collision progress gates`)
  - `d0e23f7b2f1` (`Clarify native collision audit branch state`)
  - `d790a459850` (`Document collision migration policy`)
  - `d489292bdf1` (`Define collision deprecation evidence criteria`)
  - `930aca64d45` (`Sync native collision PR evidence draft`)
  - `621fca5a1fb` (`Clarify collision reference dependencies`)
  - `944bd95f874` (`Add hello world native collision regression`)
  - `ca0201e67f4` (`Add Atlas Simbicon native collision regression`)
  - `6404f7607be` (`Classify native collision matrix deferrals`)
  - `1bfc9103a6b` (`Refresh current gz compatibility evidence`)
  - `5627a80a0a2` (`Clarify native collision supervisor status`)
  - `19aed78ae04` (`Refresh current native collision test-all evidence`)
  - `d1a1dbd9ac9` (`Refresh native collision review surface audit`)
  - `35578ad2f8a` (`Clean stale legacy collision build artifacts`)
  - `c1f03f23147` (`Scope capsule-box duplicate filtering`)
  - `17f75efeed7` (`Refresh native collision current evidence`)
  - `b5801f6c84c` (`Record native collision no-PR boundary audit`)
  - `be5d23058e6` (`Record native collision current-head validation`)
  - Later docs-only PR-packet refresh commits may exist; use
    `git log --oneline --decorate` for the exact current local head.

## Checklist Notes

- Template checklist mapping:
  - Milestone must be set on the final PR review surface.
  - `CHANGELOG.md` already carries DART 7 native-collision entries; the final
    PR surface can add a PR number reference if needed after the review surface
    is known.
  - Unit/integration/downstream/package/guard evidence is recorded in this
    folder and must be summarized on the final PR surface.
  - Public migration, architecture, and build-system docs carry the DART 7
    collision policy and reference-only old-engine scope.
  - dartpy exposes `DartCollisionDetector` without legacy detector aliases.
- The dev-task folder must be deleted in the same completing PR after evidence
  is transferred.
- C++ downstream deprecation warnings are enabled by default through
  `DART_COLLISION_DEPRECATE_LEGACY_NAMES`; downstreams may turn that option
  off only as a migration aid. Dartpy intentionally takes the clean API path.
- Per-engine FCL/Bullet/ODE collision build switches are gone from the current
  build surface. `DART_BUILD_COLLISION_REFERENCE_TESTS` and
  `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` are explicit comparison gates,
  not compatibility gates for core DART, dartpy, gz-physics runtime
  integration, or native-backed compatibility facades.
