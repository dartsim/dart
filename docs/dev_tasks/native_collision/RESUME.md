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

The task is not complete. PR #2652 is closed and still points at old head
`714d220d82a`; later pushes to `feature/new_coll` do not trigger the main
GitHub Actions workflows. The user has repeatedly directed agents not to open
or reopen a PR. Continue by pushing focused commits only when there is real
non-PR progress, and keep `PR-DRAFT.md` / `07-pr-evidence-transfer.md` ready
for the maintainer-opened review surface.

## Current Branch

`feature/new_coll` tracks `origin/feature/new_coll`.

The latest pushed validation baseline is `376fd5e686d`
(`Remove per-engine collision reference build options`), where local and remote
matched before this documentation bookkeeping refresh. If another audit-record
commit has been added after this note, run the
commands below to refresh that fact instead of trusting this timestamp:

```bash
git status --short --branch
git log -5 --oneline --decorate
git rev-parse HEAD
git rev-parse origin/feature/new_coll
```

## Immediate Next Step

Start with `06-completion-audit.md`. It is the checklist for deciding whether
the north star is complete. Under the current user constraint, the known open
items are PR/review-surface bound:

1. Final maintainer-selected PR or manual CI surface.
2. Final PR-state evidence transfer from this folder to the review body.
3. Final local validation after the completing code state.
4. Deleting `docs/dev_tasks/native_collision/` in the same completing PR.

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
- Latest full local `pixi run test-all` validation baseline:
  `376fd5e686d`, with 6/6 top-level gates passing.
- Latest local `collision-reference` benchmark guard baseline: `4b155655890`.
- Latest local downstream/package/link smoke baseline: `376fd5e686d`.
- Last manual workflow-dispatch CI evidence head: `1e1faf6feb1`.
- Latest pushed docs/reference-cleanup heads after those baselines have no
  GitHub Actions runs because branch pushes do not match the workflow filters
  while PR #2652 is closed.

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

If you make a commit, run `pixi run lint` first, then push the branch only.
Do not create or reopen a PR.
