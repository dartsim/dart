# Native Collision North-Star PR - Dev Task

## Current Scope

The active scope is branch-local feature/evidence completion. Do not open or
reopen a PR, trigger GitHub workflows, mutate PR metadata, push local commits,
or delete this folder without explicit maintainer/user approval.

The final PR/CI evidence, evidence transfer, and folder deletion are deferred
finalization steps. `06-completion-audit.md` is the deletion-gate checklist.

## Goal

Make DART's built-in/native collision detector the normal runtime stack and the
long-term replacement for FCL, Bullet, and ODE runtime collision backends. The
replacement must be evidence-driven: feature coverage, behavior compatibility,
gz-physics compatibility, native-only package/wheel isolation, and benchmark
guardrails must be verified before final dependency cleanup.

Current pass: feature-level completion. Benchmark/profiling guardrails stay in
scope; deeper single-CPU optimization, multi-core CPU parallelism, and stretch
GPU work are a later performance wave.

## Current Status

Local implementation/evidence status:

- [x] Native collision core exists under `dart/collision/native/`.
- [x] DART adapter exists under `dart/collision/dart/`.
- [x] `dart` is the normal default detector in core world, constraint, SKEL,
      C++ factory, and dartpy-compatible paths.
- [x] Legacy factory keys and C++ detector/component names are native-backed
      migration facades, not external runtime backend selectors.
- [x] dartpy exposes the clean `DartCollisionDetector` API and intentionally
      omits legacy detector aliases.
- [x] FCL, Bullet, and ODE old-engine implementations live only under
      `tests/dart/test/reference_collision/` and are reached through explicit
      `dart-test-reference-*` targets and `createReference()` APIs.
- [x] Normal Pixi/CMake/wheel paths default reference tests and reference
      benchmarks to `OFF`.
- [x] Runtime source isolation and compatibility-facade audits are wired into
      lint.
- [x] Native-only package, wheel, dartpy, gz-physics, reference-gate,
      benchmark, and stability evidence is recorded in `03-evidence-gates.md`.
- [x] `09-test-coverage-matrix.md` records 201 DONE, 0 PARTIAL, 0 GAP, and 25
      DEFERRED rows for the current feature-level pass.
- [x] Public onboarding/build/python docs contain the durable native-collision
      architecture and migration notes.
- [x] Current branch-local merge head `f949b7cbbbe` includes upstream `main`
      `b218b43786c` and passed `pixi run lint` plus `pixi run test-all`.

Open finalization gates:

- [ ] Final benchmark-guard CI artifact evidence on the completing PR surface.
- [ ] Final PR/CI review surface evidence.
- [ ] Final downstream migration/deprecation evidence on the chosen review
      surface.
- [ ] Final compatibility-facade/runtime cleanup policy evidence on the chosen
      review surface.
- [ ] Final validation on the completing code state, including at least
      `pixi run lint` and `pixi run test-all`.
- [ ] Evidence transferred to the PR description and this folder deleted in the
      same completing PR.

## North-Star Progress Scale

| Stage | Progress marker                               | Status                                                                                |
| ----- | --------------------------------------------- | ------------------------------------------------------------------------------------- |
| 0     | Baseline native backend exists.               | Complete before this task.                                                            |
| 1     | Native `dart` detector is the default path.   | Local complete; final PR validation open.                                             |
| 2     | DART feature parity gaps are closed.          | Local complete; final PR validation open.                                             |
| 3     | gz-physics compatibility is proven.           | Local complete; final downstream CI open.                                             |
| 4     | Benchmark guardrails exist.                   | Local complete; final artifact open.                                                  |
| 5     | FCL/Bullet/ODE are optional for local builds. | Local complete.                                                                       |
| 6     | Native-only and gz-physics CI are permanent.  | Manual reference evidence exists; final PR CI open.                                   |
| 7     | Reference engines are test/bench-only.        | Local complete; final reference CI open.                                              |
| 8     | Default packages have no old runtime deps.    | Local/repaired-head evidence exists; final wheel evidence open.                       |
| 9     | Downstream migration/deprecation path exists. | Local package/gz/link evidence exists; final downstream evidence open.                |
| 10    | Clean built-in API/scaling/perf layer.        | Local design/artifact evidence exists; final PR evidence open.                        |
| 11    | Old runtime backend source is reference-only. | Local source split and lint guard complete; final audit evidence open.                |
| 12    | Final one-PR validation and PR packaging.     | Blocked on maintainer-selected PR/CI surface, evidence transfer, and folder deletion. |

## Architecture Summary

`01-design.md` is the canonical design contract. The target layer shape is:

- public DART collision APIs plus temporary compatibility facades at the
  outside;
- `dart/collision/dart/` as the DART shape/result/filter adapter;
- `dart/collision/native/` as the native scene/query core for geometry,
  broadphase, narrowphase, distance, raycast, persistent manifolds, cache
  lifetimes, deterministic results, profiling, and benchmarks.

The source/package split now matches this shape locally. The remaining
architecture gates are final CI, downstream migration/deprecation evidence,
package/wheel artifacts, benchmark artifacts, and final PR cleanup.

## Final CI Closure Map

Use this map after the maintainer opens the successor PR or explicitly chooses
another workflow surface. Feature-branch pushes to `feature/new_coll` do not
start these workflows while PR #2652 is closed.

| Open gate                    | Workflow / job evidence                                                                                              | Notes                                                                                         |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| Lint and docs                | `CI Lint` / `Lint`; `CI Lint` / `Documentation`                                                                      | Confirms lint, docs build, runtime isolation, and compatibility facade audits on review head. |
| Native-only runtime          | `CI Linux` / `Native Collision (no FCL/Bullet/ODE)`; `Release Tests`; `Debug Tests`; `Asserts enabled (no -DNDEBUG)` | Confirms old engines stay out of normal runtime paths.                                        |
| Collision benchmark artifact | `CI Linux` / `Collision Benchmark Guard`                                                                             | Collect uploaded `collision-benchmark-guard-*` JSON artifact.                                 |
| gz-physics downstream        | `CI gz-physics` / `GZ Physics Tests`                                                                                 | Confirms downstream runtime compatibility on review head.                                     |
| dartpy wheels                | `Publish dartpy` wheel matrix jobs                                                                                   | Confirms wheel build, repair, verify, test, upload, and collision isolation.                  |
| Platform coverage            | `CI macOS` Release/Debug arm64; `CI Windows` Release                                                                 | Confirms platform-specific repairs still hold.                                                |

## Key Decisions

- The canonical public detector key is `dart`; `experimental` remains a
  temporary factory alias.
- Retained C++ legacy names and package components are DART 7 migration
  facades and must route to the built-in detector.
- dartpy does not keep Python legacy detector aliases through DART 7.
- FCL/Bullet/ODE comparison engines are opt-in test/benchmark reference
  targets, not normal runtime backends.
- gz-physics compatibility is a release gate, but compatibility code must stay
  narrow and documented.
- The performance bar is feature first: keep benchmark guardrails now, optimize
  single-CPU/multi-core/GPU paths later.

## Evidence And Planning Docs

- `01-design.md` - architecture contract and design review gates.
- `02-milestones.md` - phase-by-phase success criteria.
- `03-evidence-gates.md` - current evidence map and command summary.
- `04-reference-gap-analysis.md` - capability inventory and implemented
  adapter/core checklist.
- `05-downstream-migration.md` - downstream compatibility/deprecation policy.
- `06-completion-audit.md` - prompt-to-artifact completion checklist.
- `07-pr-evidence-transfer.md` - staging packet for the eventual PR body.
- `09-test-coverage-matrix.md` - test/benchmark coverage matrix.
- `PR-DRAFT.md` - maintainer-opened successor PR draft.
- `RESUME.md` - fresh-session handoff.
- `SUPERVISOR.md` - compacted supervisory side-channel and round anchors.

## Completion Rules

Do not mark this task complete until:

1. final PR/CI/artifact evidence closes every open gate in
   `06-completion-audit.md`;
2. final validation runs on the completing code state;
3. evidence is transferred into the PR description;
4. durable insights are already in onboarding docs; and
5. `docs/dev_tasks/native_collision/` is deleted in the same completing PR.
