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
A later current-state `pixi run test-all` pass on local head `c99b257cf15`
passed all 6 top-level gates after the docs/evidence cleanup commits. The
earlier pushed `64abc65a032` validation still records the latest local
gz-physics/package/link smoke baseline. `pixi run lint`, runtime isolation,
and compatibility-facade audits remain part of the local gate. The completion
audit snapshot and validation-baseline wording were also refreshed so future
passes do not treat historical "current head" evidence as live branch state.

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

Current unpushed code follow-up fixes a native box-box contact-point
regression that could let a rotated box fall through a large ground box in
default-world simulation. It also makes invalid convex/soft mesh data
non-collidable with a warning and adds public-detector coverage for
sphere-mesh collision. The slice adds raw box-box, sphere-sphere batch,
capsule-capsule batch,
cylinder-cylinder batch, convex-convex batch, mesh-mesh batch, default-world,
narrow-phase batch dispatcher, convex-mesh, and mesh regression tests, rebuilds
`hello_world` without the OctoMap `<ciso646>` warning, and refreshes focused
native/reference validation plus raw narrow-phase benchmark evidence. The
canonical command list and results are recorded in `03-evidence-gates.md`.

## Current Branch

`feature/new_coll` tracks `origin/feature/new_coll`. After explicit user
approval, the docs/evidence commits were published to
`origin/feature/new_coll`; the latest published pre-note head was
`f8f5663d514` (`Avoid stale native collision publish head`). Run
`git rev-parse HEAD` and `git rev-parse origin/feature/new_coll` for the exact
current local/remote heads because amending this note changes the latest hash.
A read-only GitHub recheck after publishing found no Actions runs for the
published evidence head; branch pushes do not trigger the main workflows while
PR #2652 remains closed. PR #2652 is closed, draft, dirty, and still anchored
to old head `714d220d82a`.

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
   deleting this folder in the same completing PR. Read-only checks on local
   head `ed451c25829` found no workflow runs for that head and confirmed
   PR #2652 remains closed on old head `714d220d82a6ba99350bf2214fc9696f5495a30f`.

If code or evidence changes again, run `pixi run lint` before committing and
run the focused validation that matches the change. Full `pixi run test-all`
evidence is refreshed locally for current-state validation head `c99b257cf15`;
later local evidence commits are docs-only and should be checked with
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
- Latest full local `pixi run test-all` validation: local head `c99b257cf15`
  (`Clarify native collision resume scope heading`), with 6/6 top-level gates
  passing after the raw, convex, mesh, mixed batch-dispatcher, and
  docs/evidence cleanup follow-ups. That run included linting, build, unit
  tests, simulation-experimental tests, Python tests, and documentation.
- Latest evidence-record commits: current local commits may be ahead of
  `origin/feature/new_coll`; run `git log -3 --oneline --decorate` for the
  current head. Recent docs-only evidence commits refresh the completion-audit
  snapshot, avoid stale-current-head wording, and keep the deferred
  finalization handoff explicit.
- Latest source-build prerequisite docs cleanup: `621fca5a1fb`. Validation:
  `pixi run lint` passed on that tree after moving FCL out of required
  prerequisites.
- Latest local `collision-reference` benchmark guard baseline: `4b155655890`.
- Latest local downstream/package/link smoke baseline: `64abc65a032`.
- Last manual workflow-dispatch CI evidence head: `1e1faf6feb1`.
- Latest pushed docs/reference-cleanup heads after those baselines have no
  GitHub Actions runs because branch pushes do not match the workflow filters
  while PR #2652 is closed.
- Latest read-only successor-PR search found no maintainer-opened replacement
  review surface; PR #2652 remains the only relevant native-collision PR and is
  closed/draft on old head `714d220d82a`.
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
  implementation files to delete. Remaining files under
  `dart/collision/{fcl,bullet,ode}/reference/` are intentional reference
  test/benchmark code for `createReference()` and `collision-reference-*`
  targets.
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
