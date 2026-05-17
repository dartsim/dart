# PR Evidence Transfer Draft

This file stages the evidence packet for the eventual PR description update. It
is not final completion evidence. Copy it only after the maintainer opens the
successor PR or explicitly chooses another review surface.

Do not reopen PR #2652, create a new PR, push local commits, trigger workflows,
mutate PR metadata, or delete this dev-task folder because this packet exists.
The folder must be deleted only in the same PR that completes the native
collision migration.

## Transfer Procedure

1. Confirm the chosen review surface and current review-head SHA.
2. Re-run or collect final evidence for every open gate in
   `06-completion-audit.md`.
3. Replace the branch-local evidence below with review-head evidence where a
   final CI artifact exists.
4. Paste the Summary, Key Changes, Testing, Breaking Changes, and Checklist
   Notes sections into the PR body.
5. Delete `docs/dev_tasks/native_collision/` in the same completing PR after the
   evidence transfer.

## Summary

- Makes native DART collision the normal runtime stack.
- Keeps retained FCL, Bullet, ODE, and `experimental` names as native-backed
  compatibility routes, not external runtime backend selectors.
- Keeps old engines only in explicit reference test and benchmark surfaces.
- Keeps dartpy on the clean DART 7 API: `DartCollisionDetector` only.
- Adds guard scripts, tests, package/wheel checks, downstream evidence, and
  benchmark guardrails to keep normal builds native-only.
- Treats this as feature-level completion. More aggressive CPU/GPU performance
  work is a later wave.

## Key Changes

- Default detector and factory routing now select the built-in `dart` stack.
- Legacy C++ detector/group names and package components are native-backed
  migration facades.
- Reference engines live behind explicit `dart-test-reference-*` targets and
  `createReference()` APIs for tests and benchmarks.
- Normal CMake and Pixi paths default
  `DART_BUILD_COLLISION_REFERENCE_TESTS=OFF` and
  `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS=OFF`.
- Per-engine collision build switches for FCL/Bullet/ODE are gone from the
  normal build surface.
- Source-build, migration, architecture, build-system, and Python docs describe
  the DART 7 native-collision policy.
- Runtime isolation, compatibility-facade behavior, wheel isolation, package
  compatibility, gz-physics compatibility, and benchmark guardrails have local
  evidence recorded in this task folder.

## Evidence Sources

Use these files as evidence owners; do not paste their full history into the PR:

| Area                        | Evidence source                                                                      |
| --------------------------- | ------------------------------------------------------------------------------------ |
| Completion decision         | `06-completion-audit.md`                                                             |
| Current status              | `README.md`, `RESUME.md`                                                             |
| Command and artifact record | `03-evidence-gates.md`                                                               |
| Coverage matrix             | `09-test-coverage-matrix.md`                                                         |
| Downstream policy           | `05-downstream-migration.md`, `docs/readthedocs/dart/user_guide/migration_guide.rst` |
| Architecture contract       | `01-design.md`, `docs/onboarding/architecture.md`                                    |
| PR body draft               | `PR-DRAFT.md`                                                                        |

## Branch-Local Evidence Snapshot

Current local branch status at the time this packet was refreshed:

- `feature/new_coll` is ahead of `origin/feature/new_coll` by local feature,
  docs, evidence, and current-main merge commits.
- The latest merged validation head is `f949b7cbbbe`
  (`Merge current main into native collision branch`), which includes upstream
  `main` at `b218b43786c`.
- PR #2652 is closed, draft, based on `main`, and anchored to old head
  `714d220d82a`.
- Read-only run lookup for `f949b7cbbbe` returns no GitHub Actions runs.

Latest current-branch validation recorded by the task:

- Merged validation head: `f949b7cbbbe`
  (`Merge current main into native collision branch`).
- Full local gate:

  ```bash
  DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all
  ```

  Result: all 6 top-level gates passed and the report printed
  `All tests passed!`. The C++ unit-test phase reported 266/266 passing tests,
  including 29 `collision-native` tests and 2 `collision-native-stability`
  tests; the simulation-experimental C++ phase reported 13/13 passing tests.

Latest focused feature-code validation recorded by the task:

- Feature-code head: `08a3ee5555c` (`Fix native capsule mesh CCD`).
- Full local gate:

  ```bash
  DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all
  ```

  Result: all 6 top-level gates passed and the report printed
  `All tests passed!`.

- Focused reruns on the same code state:
  - `test_ccd` 62/62
  - `test_capsule_capsule` 18/18
  - `test_mesh_mesh` 11/11

Additional local evidence recorded in `03-evidence-gates.md`:

- Pre-record docs-packet head `23dbbccc6c0`
  (`Refresh native collision resume after PR packet cleanup`) passed
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  with all 6 top-level gates green. C++ unit tests reported 266/266 passing,
  including the native collision labels, and the final report printed
  `All tests passed!`.
- `pixi run -e gazebo test-gz` passed 65/65 against fresh gz-physics checkouts.
- Native compatibility package smoke passed with retained
  `collision-fcl`/`collision-bullet`/`collision-ode` package components routing
  to the built-in detector.
- Direct link scans found `libdart-collision-native.so` without old DART
  collision reference/facade, FCL, Bullet, ODE, or libccd runtime dependencies.
- dartpy exposes `DartCollisionDetector` and omits legacy detector aliases.
- Wheel isolation verification rejects old collision libraries, old component
  exports, and old runtime links.
- Reference-engine tests and benchmarks use explicit `dart-test-reference-*`
  targets and `createReference()` APIs.
- `bm-collision-check` benchmark guard evidence exists for narrow phase,
  distance, raycast, raycast-batch, mesh-heavy, mixed primitive, and public
  adapter surfaces.
- `check-collision-runtime-isolation` and `audit-collision-compat-facades` are
  part of the lint gate and pass locally.

Prior GitHub reference evidence:

- Manual workflow-dispatch evidence on pushed head `1e1faf6feb1` passed
  native-only CI, gz-physics CI, full dartpy wheel matrix, and collision
  benchmark guard artifact upload.
- Treat that evidence as reference-only until the maintainer-selected final PR
  or workflow surface exists.

## Final CI Closure Map

Replace branch-local rows with final review-surface evidence before completing
the task.

| Open gate                    | Workflow / job evidence                                                                                              | Completion evidence needed                                      |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| Lint and docs                | `CI Lint` / `Lint`; `CI Lint` / `Documentation`                                                                      | Passing checks on review head.                                  |
| Native-only runtime          | `CI Linux` / `Native Collision (no FCL/Bullet/ODE)`; `Release Tests`; `Debug Tests`; `Asserts enabled (no -DNDEBUG)` | Passing checks and no old runtime collision dependency leakage. |
| Collision benchmark artifact | `CI Linux` / `Collision Benchmark Guard`                                                                             | Uploaded `collision-benchmark-guard-*` JSON artifact.           |
| gz-physics downstream        | `CI gz-physics` / `GZ Physics Tests`                                                                                 | Passing downstream CI on review head.                           |
| dartpy wheels                | `Publish dartpy` wheel matrix jobs                                                                                   | Built, repaired, verified, tested, and uploaded wheels.         |
| Platform coverage            | `CI macOS` Release/Debug arm64; `CI Windows` Release                                                                 | Passing platform checks on review head.                         |

## Breaking Changes

- dartpy no longer exposes `DARTCollisionDetector`, `FCLCollisionDetector`,
  `BulletCollisionDetector`, or `OdeCollisionDetector`. Use
  `DartCollisionDetector` or the default detector.

Compatibility notes:

- Normal runtime backend selection changes to the built-in `dart` detector even
  when retained legacy names are used.
- Retained legacy names are compatibility facades, not external runtime backend
  selectors.
- Downstream C++ runtime code should migrate to `dart`,
  `DartCollisionDetector`, `CollisionDetectorType::Dart`, and the default
  `dart` package component.
- Downstream Python runtime code should migrate directly to
  `DartCollisionDetector`.
- Explicit old-engine comparisons should use `dart-test-reference-*` targets and
  `createReference()` APIs.

## Checklist Notes

- [ ] Milestone set on the final review surface.
- [x] `CHANGELOG.md` carries DART 7 native-collision entries.
- [x] Unit, integration, downstream, package, wheel, and guard evidence is
      recorded in this task folder.
- [x] Public migration, architecture, build-system, building, and Python docs
      carry the DART 7 native-collision policy.
- [x] dartpy exposes the clean `DartCollisionDetector` API.
- [ ] Final PR/CI evidence transferred after the maintainer-selected review
      surface exists.
- [ ] `docs/dev_tasks/native_collision/` deleted in the completing PR after
      evidence transfer.
