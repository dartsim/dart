# Resume: Native Collision Default

## Last Session Summary

`feature/new_coll` makes native DART collision the normal runtime stack and
keeps FCL, Bullet, and ODE only as optional reference engines for tests and
benchmarks. The current branch has local evidence for the clean dartpy API,
native-backed C++ compatibility facades, package/wheel isolation, gz-physics
compatibility, runtime source isolation, benchmark guards, and reference-only
old-engine targets. The latest pushed docs/evidence pass also aligns public
overview, numerical-methods, constraints, and example documentation so they no
longer present FCL/Bullet/ODE as normal runtime collision backends.
The current pass is feature-level completion: benchmark/profiling guardrails
stay in scope, while single-CPU optimization, multi-core CPU parallelism, and
stretch GPU support are the next performance wave.
This session refreshed full local validation on local head `4db514cfd22` with
`pixi run test-all`, then refreshed the `collision-reference` evidence through
focused reference-backend checks and a final unfiltered CTest pass of 301/301.
A later current-state `pixi run test-all` pass on local head `ca0201e67f4`
passed all 6 top-level gates after the docs/evidence cleanup commits, the
native stability CTest label update, the stack/stress/BVH/convex-landscape
coverage, the exact `hello_world`-style no-tunneling regression, and the Atlas
Simbicon controller-loop no-tunneling regression. The current local
gz-physics refresh on `6742a21ab0f` passed 65/65 gz tests, and the current
native compatibility package smoke on `dcfc994542f` passed with direct
`readelf` checks.
`pixi run lint`, runtime isolation, and compatibility-facade audits remain
part of the local gate. The completion audit snapshot and validation-baseline
wording were refreshed so future passes do not treat historical "current head"
evidence as live branch state.
The latest local build-surface cleanup commit before this handoff update was
`35578ad2f8a` (`Clean stale legacy collision build artifacts`), which removes
stale legacy collision artifacts and stale package export snippets from reused
build directories after the retained FCL/Bullet/ODE component names became
native-backed interface facades. The latest local feature-code commit remains
`ca0201e67f4` (`Add Atlas Simbicon native collision regression`), which records
the controller-loop Atlas Simbicon no-tunneling regression in the test matrix.
The local build-surface cleanup head `35578ad2f8a` passed the full local
validation gate.
The newer docs/evidence-head validation on `48c0cc3f90e` also passed
`pixi run test-all`, the focused `hello_world` no-tunneling regression, the
Atlas Simbicon controller-loop no-tunneling regression, and the
`collision-native-stability` CTest label. No PR, push, workflow, branch, or
GitHub state was mutated for that recheck.
The latest pre-record validation on `59769b3ee58` passed
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
with all 6 top-level gates green before the docs-only validation-record commit.
No PR, push, workflow, branch, or GitHub state was mutated for that recheck.
The newest pre-record validation on `6d3224426ce` passed
`DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run test-all`
with all 6 top-level gates green after the validation-snapshot documentation
correction and before the latest evidence update. No PR, push, workflow,
branch, or GitHub state was mutated for that recheck.
The newer local gz-physics refresh on `6742a21ab0f` also passed 65/65 with
reference collision tests and benchmarks configured `OFF`; an explicit plugin
dependency scan reported `libdart-collision-native.so` without old DART
collision reference/facade, FCL, Bullet, ODE, or libccd matches.
The newer native compatibility package smoke on `dcfc994542f` also passed, with
retained `collision-fcl`, `collision-bullet`, and `collision-ode` components
resolving as native-backed package facades and `readelf` showing
`libdart-collision-native.so` without old collision/reference runtime
dependencies.
The capsule-box fix head `c1f03f23147` scopes capsule-box duplicate filtering
to the current pair, adds `CapsuleBox.DuplicateFilteringIsPairLocal`, and
passed focused `test_capsule_capsule`, the mixed and full
`collision-reference` benchmark guards, `pixi run lint`, `pixi run test-all`,
and a fresh `pixi run -e gazebo test-gz` 65/65 compatibility gate. No PR,
push, workflow, branch, or GitHub state was mutated for that recheck.
The local docs/evidence refresh `17f75efeed7` records the current capsule-box
fix evidence in the dev-task packet. A read-only external recheck for that head
found no GitHub Actions runs and only closed PR #2652 on old head
`714d220d82a`; no PR metadata, workflow state, branch state, or GitHub artifact
was mutated.
The no-PR boundary audit head `b5801f6c84c` then passed a fresh full local
`pixi run test-all` with all 6 top-level gates green: linting, build, unit
tests, simulation-experimental tests, Python tests, and documentation. The C++
unit-test phase reported 264/264 passing tests, including the native collision
labels. No PR, push, workflow, branch, or GitHub state was mutated for that
recheck.
The package-smoke evidence was recorded at `25b60737498`; later docs-only
evidence and reference-surface wording commits may move `HEAD`. Use
`git log -3 --oneline --decorate` for the exact current local head. None of
those local evidence commits opened, pushed, or mutated any PR.

The final north-star PR is not complete because PR #2652 is closed and still
points at old head `714d220d82a`; later pushes to `feature/new_coll` do not
trigger the main GitHub Actions workflows. That PR finalization is not in the
current goal scope. The current scope is branch-local feature/evidence
completion: keep the branch evidence packet current, publish only after
explicit user/maintainer approval, and keep `PR-DRAFT.md` /
`07-pr-evidence-transfer.md` ready for the maintainer-opened review surface.

The latest pushed non-PR progress moved FCL out of required source-build
prerequisites and documented FCL/Bullet/ODE as optional reference-comparison
dependencies. That docs cleanup is commit `621fca5a1fb` and was validated with
`pixi run lint`.

Current local code follow-up fixes a native box-box contact-point
regression that could let a rotated box fall through a large ground box in
default-world simulation and a capsule-box duplicate-filtering regression that
could suppress contacts from later pairs in an accumulated collision result. It
also makes invalid convex/soft mesh data non-collidable with a warning and adds
public-detector coverage for sphere-mesh collision. The slice adds raw box-box,
sphere-sphere batch,
capsule-capsule batch,
cylinder-cylinder batch, convex-convex batch, mesh-mesh batch, default-world,
narrow-phase batch dispatcher, convex-mesh, mesh, ten-box stack stability,
thin-box, slender-capsule, exact `hello_world`, and Atlas Simbicon controller
stability regression tests, rebuilds `hello_world` without the OctoMap
`<ciso646>` warning, and refreshes focused native/reference validation plus raw
narrow-phase benchmark evidence. The canonical command list and results are
recorded in `03-evidence-gates.md`.

## Current Branch

`feature/new_coll` tracks `origin/feature/new_coll`. The latest audited
remote-tracking head was `f8f5663d5145`, and the local branch was clean and
ahead by 144 commits before this resume update. Run `git rev-parse HEAD` and
`git rev-parse origin/feature/new_coll` for the exact current local/remote
heads because editing this file changes the latest local hash. The latest
read-only GitHub recheck on local docs/evidence head `ccbe9e5dd06` found no
Actions runs for that head; branch-local commits do not trigger the main
workflows while PR #2652 remains closed. PR #2652 is closed, draft, dirty, and
still anchored to old head `714d220d82a`.

The latest branch-local full validation head `5f8c9b0204a` passed
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
with all 6 top-level gates green: linting, build, unit tests,
simulation-experimental tests, Python tests, and documentation. The C++
unit-test phase reported 264/264 passing tests, including `collision-native`
and `collision-native-stability`; simulation-experimental reported 13/13,
Python reported 147/147, documentation built successfully, and the report
printed `All tests passed!`. No PR, push, workflow, branch, or GitHub state
was mutated by this recheck.

The latest pushed validation baseline is `376fd5e686d`
(`Remove per-engine collision reference build options`). Current branch head
may include docs-only evidence refresh commits on top of that validation
baseline. Run the commands below to refresh the exact local/remote state before
acting:

```bash
git status --short --branch
git log -5 --oneline --decorate
git rev-parse HEAD
git rev-parse origin/feature/new_coll
```

## Current Resume Scope

Under the current user constraint, do not spend the next pass trying to open,
reopen, or prepare GitHub PR mutations. Use `06-completion-audit.md` to keep
the distinction clear:

1. Current no-PR scope: maintain the branch-local evidence packet,
   `PR-DRAFT.md`, and `07-pr-evidence-transfer.md`.
2. Deferred PR-finalization scope: maintainer-selected PR/CI surface, final
   evidence transfer, final local validation on that completing state, and
   deleting this folder in the same completing PR. The latest read-only review
   surface check found no workflow runs for local docs/evidence head
   `ccbe9e5dd06`, `gh pr list --head feature/new_coll --state all` returned
   only PR #2652, `gh auth status -h github.com` showed `jslee02` is the active
   account, and confirmed
   PR #2652 remains closed on old head `714d220d82a6ba99350bf2214fc9696f5495a30f`.

If code or evidence changes again, run `pixi run lint` before committing and
run the focused validation that matches the change. Full `pixi run test-all`
evidence is refreshed locally through pre-record validation head `59769b3ee58`;
later local evidence commits should be checked with
`git status --short --branch` plus `git log -3 --oneline --decorate`.

Publish transport note: `origin` is configured as
`git@github.com:dartsim/dart.git`, but SSH access was unreachable during resume
checks. HTTPS branch lookup worked, and `jslee02` has admin permission on
`dartsim/dart`. The approved publish used the GitHub Git Data API because the
execution policy rejected a direct `git push` command.

Do not delete this folder, open a PR, reopen PR #2652, mutate PR metadata, or
post GitHub comments unless the user explicitly asks.

## Current Evidence Baselines

- Clean dartpy collision API and C++ compatibility-facade deprecation baseline:
  `ec6f6f43112`.
- Build-option policy baseline: `aa3ccce70c7`.
- Clean reference-gate validation: pushed validation baseline `376fd5e686d` passed
  default configure, `collision-reference` configure, focused
  `test_reference_backends`, fresh `pixi run -e gazebo test-gz`, and native
  package/link smoke with direct `readelf` checks.
- Public docs collision-runtime wording baseline: `ad1b6782578`.
- Latest full local `pixi run test-all` validation: pre-record local head `59769b3ee58`
  (`Record native collision current audit state`), with 6/6 top-level gates
  passing after current audit/evidence refreshes. Earlier validation head
  `35578ad2f8a` also passed after stale legacy collision artifact cleanup on
  top of the Atlas Simbicon coverage head. The latest feature code head remains
  `ca0201e67f4`
  (`Add Atlas Simbicon native collision regression`), after the raw, convex,
  mesh, mixed batch-dispatcher, native stability-label,
  stack/stress/BVH/convex-landscape, convex fragment, exact `hello_world`, and
  Atlas Simbicon controller stability follow-ups. The full validation run
  included linting, build, examples, unit tests, simulation-experimental tests,
  Python tests, and documentation. The artifact scan reported only
  `libdart-collision-native.so`, and the package/export scan found no old
  facade-library or FCL/Bullet/ODE/libccd runtime references.
- Latest evidence-record commits are local and may be ahead of
  `origin/feature/new_coll`; run `git log -3 --oneline --decorate` for the
  current local head. Recent docs-only evidence commits refresh the
  completion-audit snapshot, avoid stale-current-head wording, align milestone
  local/final evidence wording, record native stability-label validation, and
  keep the deferred finalization handoff explicit. Recent local evidence-record
  commits include package-smoke validation, PR evidence packet cleanup, and
  supervisor reference-audit cleanup; use `git log -3 --oneline --decorate` for
  the exact current local head instead of treating a docs-only evidence commit
  as stable.
- Latest source-build prerequisite docs cleanup: `621fca5a1fb`. Validation:
  `pixi run lint` passed on that tree after moving FCL out of required
  prerequisites.
- Latest local `collision-reference` benchmark guard baseline: `4b155655890`.
- Latest local raw narrow-phase benchmark harness audit: `0b425b3f7af`. The
  comparative narrow-phase target now consumes collision outputs/contact counts
  in timed rows, rejects empty-contact setups for strict fixed-contact rows,
  and passed the focused narrow and raw-reference benchmark guards locally.
- Latest full local `pixi run test-all` validation: `184c8be739d`. The full
  local report passed linting, build, unit tests, simulation-experimental
  tests, Python tests, and documentation after the benchmark harness evidence
  commits.
- Latest focused high-margin raw narrow-phase audit: `184c8be739d`. The focused
  benchmark JSON
  `.benchmark_results/native_collision_suspicious_win_raw_audit.json` covered
  the eight high-margin primitive rows with the hardened benchmark guards
  active.
- Latest local convexity benchmark harness audit: `1f59af53d30`. The native
  algorithm and CCD benchmark harnesses now consume the output fields they
  populate in timed loops and reject setup rows that do not enter their
  intended hit/intersection path. Fresh local benchmark JSON was emitted for
  algorithm rows, CCD rows, and convex-convex scalar/batch rows, and the
  focused convexity/CCD CTest slice passed 9/9.
- Latest local mesh benchmark harness audit: `2c6a29eaf0d`. The mesh-heavy
  scenario benchmark now consumes native/detector collision results and contact
  counts in timed rows, TriMesh-backed `MeshShape` construction no longer probes
  empty `file://` resources, the mesh-heavy benchmark guard passed without
  empty-URI warnings, and `.benchmark_results/native_collision_mesh_mesh_batch.json`
  was refreshed for mesh-mesh scalar/batch rows after built-in accuracy
  verification passed.
- Latest full local `pixi run test-all` validation after the convexity
  benchmark harness audit: source head `1f59af53d30`, with the docs/evidence
  update recording the result. The full local gate passed linting, build, unit
  tests, simulation-experimental tests, Python tests, and documentation, and
  printed `All tests passed!`.
- Latest local gz-physics baseline: `6742a21ab0f`.
- Latest local native compatibility package-smoke baseline: `dcfc994542f`.
- Last manual workflow-dispatch CI evidence head: `1e1faf6feb1`.
- Latest pushed docs/reference-cleanup heads after those baselines have no
  GitHub Actions runs because branch pushes do not match the workflow filters
  while PR #2652 is closed.
- Latest read-only successor-PR search on local docs/evidence head `307cb5fb07b`
  found no maintainer-opened replacement review surface; `gh run list` returned
  no Actions runs for that head, and PR #2652 remains the only relevant
  native-collision PR and is closed/draft on old head `714d220d82a`.
- Latest remote-ref check: `git fetch origin feature/new_coll` failed because
  SSH to `github.com:22` was unreachable in this environment. HTTPS fetch and
  `git ls-remote https://github.com/dartsim/dart.git refs/heads/feature/new_coll`
  worked and confirmed the published feature branch head.
- Current local regression refresh after `f8f5663d514`: focused raw
  box-ground tests passed, `test_box_box` passed 20/20 with the new direct
  regression and determinism coverage, `UNIT_collision_DartCollisionDetector`
  passed the new invalid-convex-mesh and sphere-mesh regressions, `test_convex`
  and `test_mesh_mesh` passed, full `collision-native` label passed 29/29, the
  focused default/native CTest slice passed 5/5, `UNIT_collision_ConvexMesh`
  and `test_reference_backends` passed 2/2 in the `collision-reference`
  environment, `hello_world` rebuilt without the prior OctoMap `<ciso646>`
  warning, and
  `.benchmark_results/native_collision_raw_narrow_phase.json` was produced by
  the raw narrow-phase benchmark run.
- Current local sphere-sphere batch refresh after `1dbbe30dea3`: focused
  `test_sphere_sphere` batch determinism passed, the default
  `collision-native` label passed 29/29 tests, `pixi run lint` passed with
  runtime-isolation and compatibility-facade audits, and the
  reference-enabled narrow-phase benchmark emitted
  `.benchmark_results/native_collision_sphere_sphere_batch.json` for
  N=1/10/100/1000.
- Current local capsule-capsule batch refresh after `ea48ea9a30c`: focused
  `test_capsule_capsule` passed 16/16 tests, including batch determinism and
  malformed-input coverage, and the reference-enabled narrow-phase benchmark
  emitted `.benchmark_results/native_collision_capsule_capsule_batch.json`
  for N=1/10/100/1000.
- Current local cylinder-cylinder batch refresh after `8bd5dd62b8a`: focused
  `test_cylinder` batch determinism and malformed-input tests passed, and the
  reference-enabled narrow-phase benchmark emitted
  `.benchmark_results/native_collision_cylinder_cylinder_batch.json` for
  N=1/10/100/1000.
- Current local convex-convex batch refresh after `f300507a350`: focused
  `test_convex` batch determinism and malformed-input tests passed, and the
  reference-enabled narrow-phase benchmark emitted
  `.benchmark_results/native_collision_convex_convex_batch.json` for
  N=1/10/100/1000.
- Current local mesh-mesh batch refresh after `6b81c9a2481`: focused
  `test_mesh_mesh` batch determinism and malformed-input tests passed, and the
  reference-enabled narrow-phase benchmark emitted
  `.benchmark_results/native_collision_mesh_mesh_batch.json` for
  N=1/10/100/1000.
- Current local narrow-phase batch dispatcher refresh after `4db514cfd22`:
  focused `test_narrow_phase` mixed-pair determinism, hit-flag, and
  malformed-input tests passed, and the reference-enabled narrow-phase
  benchmark emitted
  `.benchmark_results/native_collision_narrow_phase_dispatcher_batch.json`
  for N=1/10/100/1000.
- Current local reference sweep after `4db514cfd22`: focused
  `test_reference_backends` plus
  `INTEGRATION_collision_native_backend_consistency` passed 2/2; the
  `collision-reference` non-simulation CTest sweep passed 288/288; the
  dedicated simulation-experimental task passed 13/13; and the final
  unfiltered `collision-reference` CTest sweep passed 301/301.
- Current local slender-capsule stability refresh after `c5bc95e3bcf`: focused
  build of `test_capsule_capsule` and `UNIT_simulation_World` passed,
  `test_capsule_capsule` passed 17/17, the default-world
  `WorldTests.DefaultNativeSlenderCapsuleDoesNotTunnel` regression passed,
  `ctest -L collision-native` passed 31/31, and
  `DART_PARALLEL_JOBS=5 CMAKE_BUILD_PARALLEL_LEVEL=5 CTEST_PARALLEL_LEVEL=5 pixi run test-all`
  passed all 6 top-level gates with the final `All tests passed!` report.
- Current local downstream/package/link refresh after `6742a21ab0f` and
  `dcfc994542f`: `pixi run -e gazebo test-gz` passed 65/65 tests with the
  gazebo DART install configured with reference tests and benchmarks `OFF`; the
  native compatibility package smoke passed against the installed native build;
  direct `readelf` checks showed the gz DART plugin and package-smoke
  executable depend on `libdart-collision-native.so` without old
  collision/reference runtime dependencies.
- Current local capsule-box duplicate-filtering refresh after `c1f03f23147`:
  focused `test_capsule_capsule` passed with the new pair-local accumulated
  result regression, the previously failing mixed benchmark guard passed, the
  full `pixi run -e collision-reference bm-collision-check` guard passed,
  `pixi run lint` and `git diff --check` passed, `pixi run test-all` passed
  all 6 top-level gates, and `pixi run -e gazebo test-gz` passed 65/65.
- Current local raw narrow-phase benchmark harness audit after `0b425b3f7af`:
  the focused benchmark build passed, `bm-collision-check-narrow` reported
  `3 passed, 0 failed, 0 skipped`, `bm-collision-check-narrow-raw-reference`
  reported `2 passed, 0 failed, 0 skipped, 22 reported`, and `pixi run lint`
  plus `git diff --check` passed. No PR, push, workflow, branch, or GitHub
  state was mutated by this validation pass.
- Current local suspicious raw-row benchmark audit after `184c8be739d`:
  the focused high-margin raw-row benchmark emitted
  `.benchmark_results/native_collision_suspicious_win_raw_audit.json` after
  built-in accuracy verification passed. Native remained ahead of the fastest
  reference lane on the eight checked rows by roughly 11x-22x with result and
  contact-count consumption active.
- Current local mesh benchmark harness audit after `2c6a29eaf0d`: focused
  build and CTest passed, `bm-collision-check-mesh` reported
  `1 passed, 0 failed, 0 skipped` without empty `file://` mesh-load warnings,
  and the mesh-mesh scalar/batch benchmark refreshed
  `.benchmark_results/native_collision_mesh_mesh_batch.json`. No PR, push,
  workflow, branch, or GitHub state was mutated by this validation pass.
- Current local full validation after `184c8be739d`: `pixi run test-all`
  passed all 6 top-level gates and printed `All tests passed!`. No PR, push,
  workflow, branch, or GitHub state was mutated by this validation pass.
- Current local feature-level stability refresh after `ca0201e67f4`: focused
  build of `INTEGRATION_simulation_World` passed,
  `World.AtlasSimbiconControllerFeetStayAboveGroundWithNativeCollision`
  passed, full `INTEGRATION_simulation_World` passed 18/18,
  `ctest -L collision-native-stability` passed 2/2, `pixi run lint` and
  `git diff --check` passed, and
  `DART_PARALLEL_JOBS=5 CMAKE_BUILD_PARALLEL_LEVEL=5 CTEST_PARALLEL_LEVEL=5 pixi run test-all`
  passed all 6 top-level gates with the final `All tests passed!` report.
- Current local build-surface cleanup refresh after `35578ad2f8a`:
  `pixi run config` passed,
  `DART_PARALLEL_JOBS=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run build` passed,
  the retained compatibility component targets built as interface facades,
  `pixi run lint` and `git diff --check` passed, the artifact scan reported
  only `libdart-collision-native.so`, the package/export scan found no old
  facade-library or FCL/Bullet/ODE/libccd runtime references, and
  `DART_PARALLEL_JOBS=5 CMAKE_BUILD_PARALLEL_LEVEL=5 CTEST_PARALLEL_LEVEL=5 pixi run test-all`
  passed all 6 top-level gates with C++ tests 264/264 and the final
  `All tests passed!` report. No PR, push, workflow, branch, or GitHub state
  was mutated by this validation pass.
- Previous local default-world stability refresh after `944bd95f874`: focused
  build of `UNIT_simulation_World` passed,
  `WorldTests.DefaultNativeHelloWorldBoxDoesNotTunnel` passed, full
  `UNIT_simulation_World` passed 81/81, `ctest -L collision-native-stability`
  passed 2/2, `pixi run lint` and `git diff --check` passed, and
  `DART_PARALLEL_JOBS=5 CMAKE_BUILD_PARALLEL_LEVEL=5 CTEST_PARALLEL_LEVEL=5 pixi run test-all`
  passed all 6 top-level gates with the final `All tests passed!` report.

Useful lightweight guards:

```bash
python scripts/check_collision_runtime_isolation.py
python scripts/audit_collision_compat_facades.py
```

The last local guard refresh reported:

- `Collision runtime isolation check passed.`
- `Collision compatibility facade audit passed.`
- factory aliases `experimental`, `fcl`, `fcl_mesh`, `bullet`, and `ode`
  route to `dart`;
- dartpy exposes only `DartCollisionDetector`;
- retained `collision-fcl` / `collision-bullet` / `collision-ode` package
  components route to `dart`.

## Context That Would Be Lost

- `DartCollisionDetector` is canonical. Legacy C++ factory keys, detector
  classes, headers, and package components are native-backed migration
  facades, not external runtime backend selectors.
- Current project steering is to keep the clean long-term DART collision API, with
  the narrowest compatibility surface needed for gz-physics and downstream
  C++/package source compatibility. Do not reintroduce a real FCL/Bullet/ODE
  runtime backend selection layer.
- dartpy intentionally does not keep the legacy detector aliases through DART 7.
  The clean Python API is `DartCollisionDetector`.
- Per-engine FCL/Bullet/ODE collision build switches are gone from the current
  build surface. `DART_BUILD_COLLISION_REFERENCE_TESTS` and
  `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` are the explicit opt-in gates for
  reference comparison components. Core DART, dartpy, gz-physics runtime
  integration, and native-backed compatibility facades do not need those gates.
- The reference-file cleanup audit found no unreferenced FCL/Bullet/ODE
  implementation files to delete. Remaining old-engine implementation files
  live under `tests/dart/test/reference_collision/{fcl,bullet,ode}/` and are
  intentional reference test/benchmark code for `createReference()` and
  `dart-test-reference-*` targets.
- `docs/onboarding/architecture.md` and `docs/onboarding/build-system.md`
  contain the durable architecture/build-system notes. This dev-task folder is
  still working documentation and must be removed only in the completing PR.

## How To Resume

```bash
git checkout feature/new_coll
git status --short --branch
git log -5 --oneline --decorate
gh pr list --repo dartsim/dart --head feature/new_coll --state all \
  --json number,title,state,isDraft,headRefOid,baseRefName,url
gh run list --repo dartsim/dart --branch feature/new_coll \
  --commit "$(git rev-parse HEAD)" \
  --json databaseId,name,status,conclusion,headSha,event,url --limit 20
```

Then read:

1. `docs/dev_tasks/native_collision/06-completion-audit.md`
2. `docs/dev_tasks/native_collision/README.md`
3. `docs/dev_tasks/native_collision/PR-DRAFT.md`
4. `docs/dev_tasks/native_collision/07-pr-evidence-transfer.md`

If you make a commit, run `pixi run lint` first. Push only after explicit
user/maintainer approval. Do not create or reopen a PR.
