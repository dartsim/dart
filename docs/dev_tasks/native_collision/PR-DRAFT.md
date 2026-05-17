# Draft PR Description

This file is for a maintainer-opened successor PR or another explicitly chosen
review surface. Do not open or reopen a PR, push local commits, trigger
workflows, mutate PR metadata, or delete this dev-task folder just because this
draft exists.

Before posting, refresh the Testing section against the actual review head and
copy the final evidence from `07-pr-evidence-transfer.md`.

## Title

Make native DART collision the default runtime stack

## Summary

- Makes the built-in `DartCollisionDetector` the normal runtime collision stack.
- Keeps C++ factory keys, detector facades, and legacy package components as
  native-backed migration routes instead of external backend selectors.
- Keeps gz-physics and downstream C++ compatibility narrow while steering new
  runtime code to the `dart` detector.
- Uses a clean dartpy DART 7 API: `DartCollisionDetector` is exposed, legacy
  detector aliases are not.
- Keeps FCL, Bullet, and ODE available only through explicit test and benchmark
  reference surfaces: `dart-test-reference-*` targets plus `createReference()`
  APIs.
- Removes old per-engine collision build switches from the normal build
  surface; `DART_BUILD_COLLISION_REFERENCE_TESTS` and
  `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` are the comparison opt-in gates.
- Keeps FCL, Bullet, and ODE out of required source-build prerequisites; they
  are optional reference-comparison dependencies.
- Adds runtime-isolation, compatibility-facade, package/wheel, downstream,
  benchmark, and lint guards that keep normal builds native-only.
- Treats this pass as feature-level completion. Benchmark/profiling guardrails
  stay in scope; deeper single-CPU optimization, multi-core CPU parallelism, and
  stretch GPU support are later performance work.

## Motivation / Problem

DART historically exposed multiple collision runtime backend names across
factories, package components, detector classes, and dartpy. That surface pulled
optional FCL/Bullet/ODE runtime dependencies into normal package and wheel paths.

The native detector should be the single normal runtime stack. Old engine names
remain only as migration facades for C++ and package downstream users, and old
engines remain useful only as opt-in reference implementations for tests and
benchmarks.

## Changes / Key Changes

- Native collision is the default runtime path in normal CMake and Pixi builds.
- FCL, Bullet, ODE, and `experimental` factory keys route to
  `DartCollisionDetector`.
- C++ legacy detector/group surfaces and legacy package components are
  native-backed compatibility facades.
- dartpy exposes only the clean DART 7 `DartCollisionDetector` API.
- Explicit `dart-test-reference-*` targets and `createReference()` APIs retain
  old-engine access for tests and benchmarks only.
- Runtime source isolation is guarded by
  `scripts/check_collision_runtime_isolation.py`.
- Compatibility routing is guarded by
  `scripts/audit_collision_compat_facades.py`.
- Wheel/package isolation is guarded by wheel verification plus native package
  smoke/link checks.
- Native/reference benchmark checks cover primitive, narrow-phase, distance,
  raycast, raycast-batch, mesh-heavy, mixed-primitive, and public adapter
  surfaces.
- Native regression coverage includes raw primitive pairs, convex and mesh
  cases, batch dispatch, box/ground contact stability, capsule-box duplicate
  filtering, invalid mesh handling, `hello_world` no-tunneling behavior, and
  Atlas Simbicon controller-loop no-tunneling behavior.
- The public OctoMap include is wrapped with DART warning suppression so
  `hello_world` rebuilds without the third-party `<ciso646>` C++20 warning.
- Durable architecture and migration notes are seeded in onboarding and public
  docs; this dev-task folder remains only until the completing PR transfers
  evidence and deletes it.

## Testing

Branch-local evidence currently staged in `docs/dev_tasks/native_collision/`:

- Latest feature-code head recorded by the task: `08a3ee5555c`
  (`Fix native capsule mesh CCD`).
- Recorded full local validation on that feature-code head:

  ```bash
  DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all
  ```

  The run passed all 6 top-level gates and printed `All tests passed!`.

- Focused reruns on the same code head passed:
  - `test_ccd` 62/62
  - `test_capsule_capsule` 18/18
  - `test_mesh_mesh` 11/11
- Local gz-physics evidence recorded in `03-evidence-gates.md` includes fresh
  `pixi run -e gazebo test-gz` passes with 65/65 tests and direct link checks
  showing `libdart-collision-native.so` without old collision/reference runtime
  dependencies.
- Local package/wheel evidence recorded in `03-evidence-gates.md` includes
  native compatibility package smoke, wheel isolation verification, dartpy API
  checks, and absence of old FCL/Bullet/ODE/libccd runtime links in normal
  artifacts.
- Local reference/benchmark evidence recorded in `03-evidence-gates.md` includes
  reference-backend CTest coverage and `bm-collision-check` benchmark guard
  output.
- Docs-only cleanup commits after the feature-code head have run `pixi run lint`
  and `git diff --check`; the lint gate reran runtime-isolation and
  compatibility-facade audits.
- Prior manual workflow-dispatch evidence on pushed head `1e1faf6feb1` passed
  native-only CI, gz-physics CI, the full dartpy wheel matrix, and collision
  benchmark artifact upload. Treat this as reference evidence only until the
  final maintainer-selected review surface exists.

Final PR-complete testing still needs evidence on the actual completing review
surface. Use this closure map:

| Open gate                    | Workflow / job evidence                                                                                              |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| Lint and docs                | `CI Lint` / `Lint`; `CI Lint` / `Documentation`                                                                      |
| Native-only runtime          | `CI Linux` / `Native Collision (no FCL/Bullet/ODE)`; `Release Tests`; `Debug Tests`; `Asserts enabled (no -DNDEBUG)` |
| Collision benchmark artifact | `CI Linux` / `Collision Benchmark Guard` with uploaded `collision-benchmark-guard-*` JSON artifact                   |
| gz-physics downstream        | `CI gz-physics` / `GZ Physics Tests`                                                                                 |
| dartpy wheels                | `Publish dartpy` wheel matrix jobs                                                                                   |
| Platform coverage            | `CI macOS` Release/Debug arm64; `CI Windows` Release                                                                 |

## Breaking Changes

- dartpy no longer exposes `DARTCollisionDetector`, `FCLCollisionDetector`,
  `BulletCollisionDetector`, or `OdeCollisionDetector`. Use
  `DartCollisionDetector` or the default detector.
- Normal runtime backend selection changes to the built-in `dart` detector even
  when retained legacy C++ or package names are used.
- Explicit old-engine comparisons should use `dart-test-reference-*` targets
  and `createReference()` APIs.

## Related Issues / PRs

- Supersedes or replaces closed draft PR #2652:
  `https://github.com/dartsim/dart/pull/2652`

## Checklist Notes

- [ ] Milestone set on the final review surface (`DART 7.0` for `main`).
- [x] `CHANGELOG.md` updated for DART 7 native-collision behavior, dartpy alias
      removal, reference-only old engines, benchmark guard, wheel isolation, and
      runtime source isolation.
- [x] Unit/integration/downstream/package/guard evidence is recorded in
      `docs/dev_tasks/native_collision/`.
- [x] Public migration, architecture, build-system, building, and Python docs
      carry the DART 7 collision policy and reference-only old-engine scope.
- [x] dartpy exposes `DartCollisionDetector` without legacy detector aliases.
- [ ] Final PR/CI evidence transferred after the maintainer-selected review
      surface exists.
- [ ] `docs/dev_tasks/native_collision/` deleted in the same completing PR after
      evidence transfer.
