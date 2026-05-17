# Native Collision Evidence Gates

This file records the current evidence map for the native-collision task. A
gate is not complete until the result is tied to a commit, command, and
observed output. Detailed historical run logs are intentionally compacted out
of this file per Round 20; use git history for the pre-compaction transcript.

## Branch Baseline

- Branch: `feature/new_coll`
- Starting commit for the native-default push: `96436fd2503`
- Latest feature-code evidence head: `08a3ee5555c`
  (`Fix native capsule mesh CCD`)
- Latest current-branch validation head: `f949b7cbbbe`
  (`Merge current main into native collision branch`)
- Current branch-local status: `f949b7cbbbe` merges upstream `main`
  `b218b43786c` and has local `pixi run test-all` evidence. Later commits are
  docs/evidence cleanup unless recent `git log --oneline --decorate` output
  shows otherwise.

## Gate Matrix

| Gate                  | Required evidence                                                                                                | Current state                                                                  |
| --------------------- | ---------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| Native default        | Focused tests show default detector type is `dart`.                                                              | Local pass, final PR validation open.                                          |
| Feature parity        | Native unit/integration collision tests cover the DART surface.                                                  | Local pass, final PR validation open.                                          |
| Reference consistency | Native-vs-FCL/Bullet/ODE consistency tests pass in opt-in reference builds.                                      | Local pass, final reference CI open.                                           |
| gz-physics            | `pixi run -e gazebo test-gz` passes without behavioral downstream patches beyond the intended DART version bump. | Local 65/65 pass, final PR/downstream CI open.                                 |
| Performance guard     | Comparative benchmark guards check native/reference and public adapter paths.                                    | Local pass, final benchmark artifact open.                                     |
| Dependency removal    | Normal builds/packages/wheels avoid old collision runtime deps.                                                  | Local pass plus repaired-head workflow evidence, final PR-state evidence open. |
| Source isolation      | Lint blocks old-engine runtime include/source leakage.                                                           | Local pass.                                                                    |
| Full validation       | `pixi run test-all` passes on the feature-code state and current local packet.                                   | Local pass on latest feature-code head and current merged packet.              |

## North-Star Progress Scale

| Stage | Gate                                              | State                                                                  |
| ----- | ------------------------------------------------- | ---------------------------------------------------------------------- |
| 0     | Baseline native backend exists.                   | Complete before this task.                                             |
| 1     | Native `dart` detector is default.                | Local complete; final PR validation open.                              |
| 2     | DART feature parity is proven.                    | Local complete; final PR validation open.                              |
| 3     | gz-physics compatibility is proven.               | Local complete; final CI open.                                         |
| 4     | Benchmark guardrails exist.                       | Local complete; final artifact open.                                   |
| 5     | Local builds pass with FCL/Bullet/ODE disabled.   | Local complete.                                                        |
| 6     | Native-only and gz-physics CI are permanent.      | Manual reference evidence exists; final PR CI open.                    |
| 7     | Reference engines are test/bench-only opt-in.     | Local target split proven; final CI open.                              |
| 8     | Default packages/wheels have no old runtime deps. | Local/repaired-head evidence exists; final PR wheel evidence open.     |
| 9     | Downstream migration/deprecation path is tested.  | Local package/gz/link evidence exists; final downstream evidence open. |
| 10    | Clean built-in API/scaling/perf layer.            | Local design/artifact evidence exists; final PR evidence open.         |
| 11    | Old runtime backend source is reference-only.     | Local split and lint guard complete; final audit evidence open.        |
| 12    | Final PR evidence complete and task docs removed. | Blocked on maintainer-selected PR/CI surface and final cleanup.        |

## Current Local Validation

Latest current-branch validation:

- Commit: `f949b7cbbbe` (`Merge current main into native collision branch`)
- Upstream included: `main` at `b218b43786c`
- Commands:
  - `pixi run update-api-boundary-inventory`
  - `pixi run check-api-boundary-inventory`
  - `pixi run lint`
  - `git diff --cached --check`
  - `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
- Observed output:
  - API boundary inventory regenerated and checked clean after the upstream
    merge.
  - Lint passed, including `check-collision-runtime-isolation`,
    `audit-collision-compat-facades`, API-boundary checks, and AI command sync.
  - `test-all` passed all 6 top-level gates: linting, build, unit tests,
    simulation-experimental tests, Python tests, and documentation.
  - The C++ unit-test phase reported 266/266 tests passing, including 29
    `collision-native` tests and 2 `collision-native-stability` tests.
  - The simulation-experimental C++ phase reported 13/13 tests passing.
  - Final report printed `All tests passed!`.

Latest focused feature-code validation:

- Commit: `08a3ee5555c` (`Fix native capsule mesh CCD`)
- Command:
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
- Observed output: all 6 top-level gates passed: linting, build, unit tests,
  simulation-experimental tests, Python tests, and documentation. Final report:
  `All tests passed!`.
- Focused affected-test reruns on the same build tree:
  - `test_ccd` passed 62/62.
  - `test_capsule_capsule` passed 18/18.
  - `test_mesh_mesh` passed 11/11.

Prior docs-compaction validation:

- Commands:
  - `pixi run lint`
  - `pixi run check-lint-md`
  - `git diff --check`
  - `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
- Observed output:
  - Markdown formatting passed.
  - `check-collision-runtime-isolation` passed.
  - `audit-collision-compat-facades` passed.
  - API boundary and AI command sync checks passed.
  - `test-all` passed on pre-record docs-packet head `23dbbccc6c0`
    (`Refresh native collision resume after PR packet cleanup`) with all 6
    top-level gates green: linting, build, unit tests,
    simulation-experimental tests, Python tests, and documentation. The C++
    unit-test phase reported 266/266 tests passing, including 29
    `collision-native` tests and 2 `collision-native-stability` tests, and the
    final report printed `All tests passed!`.

## Current Source And Facade Guards

`python scripts/check_collision_runtime_isolation.py`

- Observed output: `Collision runtime isolation check passed.`
- Coverage: rejects old-engine runtime include/source leakage and keeps
  implementation code from depending on FCL/Bullet/ODE except for documented
  DART 7 compatibility facades and test/benchmark reference code.

`python scripts/audit_collision_compat_facades.py`

- Observed output: `Collision compatibility facade audit passed.`
- Coverage:
  - factory aliases `experimental`, `fcl`, `fcl_mesh`, `bullet`, and `ode`
    route to `dart`;
  - legacy C++ names warn by default;
  - C++ facades route to `DartCollisionDetector`;
  - dartpy exposes `DartCollisionDetector` only;
  - retained package components `collision-fcl`, `collision-bullet`, and
    `collision-ode` route to `dart`;
  - reference engines live under `tests/dart/test/reference_collision`.

## Current Downstream And Package Evidence

Local gz-physics evidence:

- Command: `pixi run -e gazebo test-gz`
- Observed output: 65/65 gz-physics tests passed on refreshed local heads.
- Link inspection: the gz DART plugin depends on
  `libdart-collision-native.so` and does not depend on old DART collision
  reference/facade libraries, FCL, Bullet, ODE, or libccd.

Local package-smoke evidence:

- Smoke package:
  `docs/dev_tasks/native_collision/smoke/native_compat_package/`
- Coverage: requests retained `collision-fcl`, `collision-bullet`, and
  `collision-ode` package components, includes installed legacy detector
  headers, verifies legacy factory keys/direct legacy `create()` calls, and
  checks that the executable links `libdart-collision-native.so` without old
  collision/reference runtime deps.

## Current Benchmark Evidence

Broad guard:

- Command:
  `DART_PARALLEL_JOBS=<N> CTEST_PARALLEL_LEVEL=<N> CMAKE_BUILD_PARALLEL_LEVEL=<N> pixi run -e collision-reference bm-collision-check`
- Coverage: checked native/reference benchmark subsets for narrow phase,
  distance, raycast, raycast-batch, mesh-heavy, mixed primitives, and public
  DART adapter paths.

Audit-hardened benchmark evidence:

- Raw narrow-phase rows now consume outputs/contact counts in timed loops.
- Suspicious high-margin raw wins are audited using the Round 18 protocol.
- Convexity and mesh benchmark harnesses consume outputs and reject trivial
  no-op results.
- Single-CPU optimization, multi-core CPU parallelism, and stretch GPU work
  remain the next performance wave; current pass keeps guardrails and
  correctness baselines.

## Current Wheel And Python Evidence

- dartpy exposes the clean `DartCollisionDetector` API.
- dartpy legacy detector aliases are intentionally absent.
- `wheel-verify` runs `scripts/verify_wheel_collision_isolation.py`.
- Repaired-head wheel matrix reference evidence passed; final PR-state wheel
  artifact evidence remains open.

## PR / CI Evidence State

Read-only GitHub checks show:

- PR #2652 is closed, draft, based on `main`, and anchored to old head
  `714d220d82a`.
- Feature-branch pushes after PR closure do not start the main PR workflows.
- `gh run list --repo dartsim/dart --commit f949b7cbbbe...` returned no runs
  for the local-only merged validation head.

Manual workflow-dispatch reference evidence on repaired head `1e1faf6feb1`
exists for native-only CI, gz-physics CI, dartpy wheel matrix, and collision
benchmark artifact upload. Treat it as reference evidence only until a
maintainer-selected final PR/CI surface exists.

## Historical CI Repair Summary

The earlier closed PR cycle repaired:

- asserts-enabled configure flags that still enabled old collision engines;
- Windows Pixi parser handling for collision override defaults;
- Alt Linux Eigen and EnTT source-build readiness;
- MSVC UTF-8 flags for wheel builds;
- macOS include hygiene and distance-filter link/ownership failures;
- wheel collision-isolation policy for native-backed compatibility components.

Those repairs are represented by current local validation plus repaired-head
workflow evidence. Final review-head CI still has to prove they hold on the
successor PR surface.

## Known Risks

- Final PR/CI evidence is missing because PR #2652 is closed.
- Final benchmark JSON artifact evidence is missing on the completing review
  surface.
- Final wheel/package evidence is missing on the completing review surface.
- Downstream migration/deprecation finalization evidence is still open.
- `docs/dev_tasks/native_collision/` must not be deleted until evidence is
  transferred to the completing PR and deletion happens in that same PR.
