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
- Moves old external-engine access behind explicit reference-only test and
  benchmark surfaces.
- Adds source, package, wheel, downstream, benchmark, and lint guards to keep
  normal builds native-only.

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
- Explicit `collision-reference-*` surfaces retain opt-in access for tests and
  benchmarks.
- Public DART migration docs now state the DART 7 policy: C++ legacy collision
  names are native-backed migration facades, dartpy keeps the clean
  `DartCollisionDetector` API without legacy aliases, and FCL/Bullet/ODE
  comparison engines are opt-in reference components rather than normal runtime
  build options.
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
- Durable architecture notes are seeded in onboarding docs; working
  `docs/dev_tasks/native_collision/` notes remain only until final completion.

## Testing

Local validation currently recorded in the dev-task evidence:

- `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  passed on pushed branch head `930aca64d45` before the source-build
  prerequisite docs cleanup
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
  ODE reference components.
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
  - latest local refresh is tied to pushed validation baseline `376fd5e686d`;
    this documentation refresh is bookkeeping only
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
- Latest local full-validation refresh on pushed head `930aca64d45`:
  - `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
    passed with 6/6 top-level gates: linting, build, unit tests,
    simulation-experimental tests, Python tests, and documentation.
  - The C++ Release CTest pass completed 264/264 tests, and Python tests
    completed 147/147 tests.
- Later pushed audit/docs/policy heads have no attached GitHub Actions runs
  unless the maintainer chooses a PR or manual workflow surface, because
  `feature/new_coll` pushes do not match the workflow `push` filters.

Final PR-complete testing still needs whatever CI surface the maintainer chooses
after the maintainer-opened successor PR or another explicit review surface
exists.

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
- Explicit old-engine comparisons should use `collision-reference-*` targets
  and `createReference()` APIs.

## Related Issues / PRs

- PR #2652: `https://github.com/dartsim/dart/pull/2652`
- Latest audited branch evidence:
  - `ec6f6f43112` (`Clean dartpy collision API and deprecate C++ facades`)
  - `aa3ccce70c7` (`Clarify collision reference build options`)
  - `06cd27d0163` (`Rename collision reference build options`)
  - `376fd5e686d` (`Remove per-engine collision reference build options`)
  - `d0e23f7b2f1` (`Clarify native collision audit branch state`)
  - `d790a459850` (`Document collision migration policy`)
  - `d489292bdf1` (`Define collision deprecation evidence criteria`)
  - `930aca64d45` (`Sync native collision PR evidence draft`)

## Checklist Notes

- Milestone must be set on the final PR review surface.
- `CHANGELOG.md` update should be decided after the final PR number is known.
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
