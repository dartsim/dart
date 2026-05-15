# Draft PR Description

This file is for the maintainer-opened successor PR. Do not open or reopen a PR
from this branch just because this draft exists.

## Title

Make native DART collision the default runtime stack

## Summary

- Makes the built-in `DartCollisionDetector` the normal runtime collision stack.
- Keeps C++ factory keys, C++ detector facades, and legacy package components as
  native-backed migration routes instead of external backend selectors.
- Uses a clean dartpy DART 7 API: `DartCollisionDetector` is exposed, legacy
  detector aliases are not.
- Keeps FCL, Bullet, and ODE available only through explicit
  `collision-reference-*` targets and `createReference()` APIs for tests and
  benchmarks.
- Renames the optional old-engine comparison CMake/Pixi switches to
  `DART_BUILD_COLLISION_REFERENCE_FCL`,
  `DART_BUILD_COLLISION_REFERENCE_BULLET`, and
  `DART_BUILD_COLLISION_REFERENCE_ODE` so the build surface no longer presents
  FCL/Bullet/ODE as runtime collision backend options.
- Adds runtime-isolation, compatibility-facade, wheel/package, downstream, and
  benchmark evidence gates.

## Breaking Changes

- dartpy no longer exposes `DARTCollisionDetector`, `FCLCollisionDetector`,
  `BulletCollisionDetector`, or `OdeCollisionDetector`. Use
  `DartCollisionDetector` or the default detector.

## Test Plan

- `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  passed on the working tree after `ab8ad841b9c` with default
  `DART_BUILD_COLLISION_REFERENCE_*` engine options and reference gates `OFF`.
- `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run -e collision-reference -- ctest --test-dir build/collision-reference/cpp/Release --output-on-failure -R '^test_reference_backends$' -j 5`
  passed 1/1 after configuring `collision-reference` with FCL, Bullet, ODE,
  reference tests, and reference benchmarks `ON`.
- `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run -e collision-reference bm-collision-check`
  passed before benchmark-evidence commit `4b155655890`.
- `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run -e gazebo test-gz`
  passed on current downstream evidence head `4b155655890` with
  FCL/Bullet/ODE/reference knobs `OFF`.
- Native compatibility package smoke and direct `readelf` link inspection passed
  on `4b155655890`; gz/plugin package-smoke binaries link
  `libdart-collision-native.so` without old collision/reference runtime
  dependencies.
- `pixi run lint`
- `pixi run check-lint`
- `pixi run build`
- `pixi run test-unit`
- `pixi run test-py`
- `python scripts/check_collision_runtime_isolation.py`
- `python scripts/audit_collision_compat_facades.py`

Final PR evidence still needs the maintainer-selected CI surface after the
manual PR is opened.

## Evidence Transfer

| Area                        | Evidence source                                                     |
| --------------------------- | ------------------------------------------------------------------- |
| Native runtime default      | `docs/dev_tasks/native_collision/README.md`, `03-evidence-gates.md` |
| Downstream migration policy | `05-downstream-migration.md`                                        |
| Completion/open gates       | `06-completion-audit.md`                                            |
| PR body staging             | `07-pr-evidence-transfer.md`                                        |
| Architecture contract       | `01-design.md`, `docs/onboarding/architecture.md`                   |
