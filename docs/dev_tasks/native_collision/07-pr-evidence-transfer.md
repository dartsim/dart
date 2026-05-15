# PR Evidence Transfer Draft

This file is a staging packet for the eventual PR description update. It should
be copied into the PR review surface only after the maintainer chooses whether
to reopen PR #2652 or create a successor PR.

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
- dartpy legacy detector names are aliases of `DartCollisionDetector`.
- Legacy package components `collision-fcl`, `collision-bullet`, and
  `collision-ode` are native-backed interface facades.
- Explicit `collision-reference-*` surfaces retain opt-in access for tests and
  benchmarks.
- Runtime isolation is guarded by `check-collision-runtime-isolation`, which
  rejects non-reference old-engine includes and now also enforces pure
  top-level legacy facade headers.
- Compatibility routing is guarded by `audit-collision-compat-facades`, which
  audits factory aliases, C++ facades, dartpy aliases, and package components.
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
  passed after the compatibility-facade audit hardening with 6/6 top-level
  gates:
  - linting
  - build
  - unit tests
  - simulation-experimental tests
  - Python tests
  - documentation
- `pixi run lint` passed and includes:
  - `check-collision-runtime-isolation`
  - `audit-collision-compat-facades`
- `pixi run check-lint` passed and verifies the same audit guards through the
  non-mutating lint path.
- `python scripts/check_collision_runtime_isolation.py` passed.
- `python scripts/audit_collision_compat_facades.py` passed.
- Prior local downstream evidence:
  - fresh `pixi run -e gazebo test-gz` passed 65/65 tests
  - native compatibility package smoke passed
  - `readelf` showed gz/plugin package-smoke binaries depending on
    `libdart-collision-native.so` without old collision/reference runtime
    dependencies
- Prior workflow-dispatch reference evidence on pushed head `1e1faf6feb1`:
  - native-only CI passed
  - gz-physics CI passed
  - full dartpy wheel matrix passed
  - collision benchmark guard uploaded
    `collision-benchmark-guard-25887939088-1`

Final PR-complete testing still needs whatever CI surface the maintainer chooses
after PR #2652 is reopened or a successor PR is created.

## Breaking Changes

- [ ] None

Compatibility notes:

- Normal runtime backend selection changes to the built-in `dart` detector even
  when retained legacy names are used.
- Retained legacy names are compatibility facades, not external runtime backend
  selectors.
- Downstream runtime code should migrate to `dart`, `DartCollisionDetector`,
  `CollisionDetectorType::Dart`, and the default `dart` package component.
- Explicit old-engine comparisons should use `collision-reference-*` targets
  and `createReference()` APIs.

## Related Issues / PRs

- PR #2652: `https://github.com/dartsim/dart/pull/2652`
- Current pushed branch: `feature/new_coll`

## Checklist Notes

- Milestone must be set on the final PR review surface.
- `CHANGELOG.md` update should be decided after the final PR number is known.
- The dev-task folder must be deleted in the same completing PR after evidence
  is transferred.
- Downstream deprecation or hard-removal requires explicit maintainer policy
  direction; do not add deprecation warnings or remove retained facades until
  that decision is made.
