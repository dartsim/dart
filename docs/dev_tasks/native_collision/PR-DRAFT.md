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
- Removes per-engine FCL/Bullet/ODE collision build switches from the current
  build surface; explicit reference tests and benchmarks are the only public
  opt-in gates for old-engine comparison components.
- Keeps FCL/Bullet/ODE out of required source-build prerequisites; they are
  optional reference-comparison dependencies, not core runtime dependencies.
- Documents the DART 7 migration policy in public docs and defines the
  downstream/deprecation evidence criteria that must be attached before retained
  C++ and package facades are removed or hard-deprecated further.
- Adds runtime-isolation, compatibility-facade, wheel/package, downstream, and
  benchmark evidence gates.

## Breaking Changes

- dartpy no longer exposes `DARTCollisionDetector`, `FCLCollisionDetector`,
  `BulletCollisionDetector`, or `OdeCollisionDetector`. Use
  `DartCollisionDetector` or the default detector.

## Test Plan

- `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  passed on pushed branch head `930aca64d45` before the source-build
  prerequisite docs cleanup; the current default configure path keeps reference
  tests and benchmarks `OFF` and exposes no per-engine FCL/Bullet/ODE collision
  build switches.
- `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run -e collision-reference -- ctest --test-dir build/collision-reference/cpp/Release --output-on-failure -R '^test_reference_backends$' -j 5`
  passed 1/1 after configuring `collision-reference` with reference tests and
  reference benchmarks `ON` and all FCL, Bullet, and ODE reference components
  enabled internally.
- `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run -e collision-reference bm-collision-check`
  passed before benchmark-evidence commit `4b155655890`.
- `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run -e gazebo test-gz`
  passed on pushed validation baseline `376fd5e686d` with reference tests and
  benchmarks `OFF` and no per-engine FCL/Bullet/ODE collision build switches.
- Native compatibility package smoke and direct `readelf` link inspection passed
  on pushed validation baseline `376fd5e686d`; gz/plugin package-smoke binaries link
  `libdart-collision-native.so` without old collision/reference runtime
  dependencies.
- `pixi run lint` passed on pushed docs-policy head `d489292bdf1` after adding
  the public migration guide language and downstream deprecation evidence
  acceptance criteria. That lint run configured reference tests and benchmarks
  `OFF`, reran `check-collision-runtime-isolation`, and reran
  `audit-collision-compat-facades`.
- `pixi run lint` passed on pushed source-build prerequisite docs head
  `621fca5a1fb`; that run also reran `check-collision-runtime-isolation` and
  `audit-collision-compat-facades`.
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

| Area                        | Evidence source                                                                      |
| --------------------------- | ------------------------------------------------------------------------------------ |
| Native runtime default      | `docs/dev_tasks/native_collision/README.md`, `03-evidence-gates.md`                  |
| Downstream migration policy | `05-downstream-migration.md`, `docs/readthedocs/dart/user_guide/migration_guide.rst` |
| Completion/open gates       | `06-completion-audit.md`                                                             |
| PR body staging             | `07-pr-evidence-transfer.md`                                                         |
| Architecture contract       | `01-design.md`, `docs/onboarding/architecture.md`                                    |
