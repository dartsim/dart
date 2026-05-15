# Draft PR Description

This file is for the maintainer-opened successor PR. Do not open or reopen a PR
from this branch just because this draft exists.

## Title

Make native DART collision the default runtime stack

## Summary

- Makes the built-in `DartCollisionDetector` the normal runtime collision stack.
- Keeps C++ factory keys, C++ detector facades, and legacy package components as
  native-backed migration routes instead of external backend selectors.
- Favors the clean long-term DART collision API while keeping the narrow
  gz-physics/downstream C++ compatibility surface needed for migration.
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
- Treats this pass as feature-level completion: benchmark/profiling guardrails
  stay in place, while single-CPU optimization, multi-core CPU parallelism, and
  stretch GPU support are follow-up performance work.
- Documents the DART 7 migration policy in public docs and defines the
  downstream/deprecation evidence criteria that must be attached before retained
  C++ and package facades are removed or hard-deprecated further.
- Adds runtime-isolation, compatibility-facade, wheel/package, downstream, and
  benchmark evidence gates.

## Motivation / Problem

- DART has historically exposed multiple runtime collision backend names across
  factories, package components, detector classes, and dartpy.
- That surface pulled optional FCL/Bullet/ODE runtime dependencies into normal
  package and wheel paths.
- Native DART collision should be the single normal runtime stack, while
  gz-physics and C++ downstream users get a migration window where legacy names
  still compile and run as native-backed facades.

## Changes / Key Changes

- Native collision is the default runtime path in normal CMake and Pixi builds.
- FCL, Bullet, ODE, and `experimental` factory keys route to
  `DartCollisionDetector`.
- C++ legacy detector/group surfaces and legacy package components are
  native-backed compatibility facades.
- dartpy exposes only the clean DART 7 `DartCollisionDetector` API.
- Explicit `collision-reference-*` targets and `createReference()` APIs retain
  old-engine access for tests and benchmarks only.
- Runtime isolation, compatibility-facade, wheel/package, downstream, and
  benchmark guards keep normal builds native-only.

## Testing

- `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run test-all`
  passed on pushed branch head `64abc65a032`
  (`Clarify native collision progress gates`) with 6/6 top-level gates:
  linting, build, unit tests, simulation-experimental tests, Python tests, and
  documentation. The default configure path kept reference tests and benchmarks
  `OFF` and exposed no per-engine FCL/Bullet/ODE collision build switches.
- `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run -e collision-reference -- ctest --test-dir build/collision-reference/cpp/Release --output-on-failure -R '^test_reference_backends$' -j 5`
  passed 1/1 after configuring `collision-reference` with reference tests and
  reference benchmarks `ON` and all FCL, Bullet, and ODE reference components
  enabled internally.
- `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run -e collision-reference bm-collision-check`
  passed before benchmark-evidence commit `4b155655890`.
- `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run -e gazebo test-gz`
  passed on code head `64abc65a032` with reference tests and benchmarks `OFF`
  and no per-engine FCL/Bullet/ODE collision build switches.
- Native compatibility package smoke and direct `readelf` link inspection passed
  on code head `64abc65a032`; gz/plugin package-smoke binaries link
  `libdart-collision-native.so` without old collision/reference runtime
  dependencies.
- Prior manual workflow-dispatch reference evidence on pushed head
  `1e1faf6feb1` passed native-only CI, gz-physics CI, the full dartpy wheel
  matrix, and collision benchmark guard artifact upload. Treat this as reference
  evidence only until the maintainer-selected final PR/CI surface exists.
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

## Breaking Changes

- [x] dartpy no longer exposes `DARTCollisionDetector`,
      `FCLCollisionDetector`, `BulletCollisionDetector`, or
      `OdeCollisionDetector`. Use `DartCollisionDetector` or the default
      detector.

## Related Issues / PRs

- PR #2652: `https://github.com/dartsim/dart/pull/2652`

---

#### Checklist

- [ ] Milestone set (DART 7.0 for `main`, DART 6.16.x for `release-6.16`) on
      the final PR review surface.
- [x] CHANGELOG.md updated if required: DART 7 native-collision behavior,
      dartpy alias removal, reference-only old engines, benchmark guard, wheel
      isolation, and runtime source isolation are recorded.
- [x] Add unit tests for new functionality: native/default detector,
      compatibility facades, reference backends, gz-focused regressions, and
      guard scripts are recorded in `docs/dev_tasks/native_collision/`.
- [x] Document new methods and classes: public migration, architecture, and
      build-system docs describe the DART 7 collision policy and reference-only
      old-engine scope.
- [x] Add Python bindings (dartpy) if applicable: dartpy exposes
      `DartCollisionDetector`; legacy detector aliases are intentionally absent
      and verified.
- [ ] Final PR/CI evidence transferred after the maintainer-selected review
      surface exists.
- [ ] `docs/dev_tasks/native_collision/` deleted in the completing PR after
      evidence transfer.

## Evidence Transfer

| Area                        | Evidence source                                                                      |
| --------------------------- | ------------------------------------------------------------------------------------ |
| Native runtime default      | `docs/dev_tasks/native_collision/README.md`, `03-evidence-gates.md`                  |
| Downstream migration policy | `05-downstream-migration.md`, `docs/readthedocs/dart/user_guide/migration_guide.rst` |
| Completion/open gates       | `06-completion-audit.md`                                                             |
| PR body staging             | `07-pr-evidence-transfer.md`                                                         |
| Architecture contract       | `01-design.md`, `docs/onboarding/architecture.md`                                    |
