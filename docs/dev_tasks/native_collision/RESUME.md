# Resume: Native Collision Default

## Last Session Summary

`feature/new_coll` makes native DART collision the normal runtime stack and
keeps FCL, Bullet, and ODE only as optional reference engines for tests and
benchmarks. The latest merged validation head is `f949b7cbbbe`
(`Merge current main into native collision branch`), which includes upstream
`main` at `b218b43786c` and passed `pixi run lint` plus `pixi run test-all`.
The latest focused feature-code head is `08a3ee5555c`
(`Fix native capsule mesh CCD`), which passed the focused CCD/capsule/mesh
reruns recorded in `03-evidence-gates.md`.

Recent local-only history refreshed the Round 20 documentation-hygiene slice
for the largest native-collision side-channel files, merged current upstream
`main`, recorded the merged validation evidence, and aligned the PR staging
packet with `.github/PULL_REQUEST_TEMPLATE.md`:

- `21ea43cdc12` — `Compact native collision supervisor notes`
- `01adba243b1` — `Compact native collision completion audit`
- `f8e251ee1c8` — `Compact native collision evidence gates`
- `de5a1695ea8` — `Refresh native collision resume handoff`
- `d875b95e0a5` — `Compact native collision README`
- `9d2017f73e7` — `Refresh native collision resume after README cleanup`
- `b16165f5859` — `Compact native collision PR packet`
- `23dbbccc6c0` — `Refresh native collision resume after PR packet cleanup`
- `aea78dc3d61` — `Record native collision current packet validation`
- `a9816dcfe06` — `Clarify native collision local CI audit wording`
- `f949b7cbbbe` — `Merge current main into native collision branch`
- `7246d1d59ff` — `Record native collision merged validation evidence`
- `2f128a6664e` — `Align native collision PR packet with template`
- `f544a72ae12` — `Complete native collision PR transfer packet`
- `d2a0d842a3f` — `Match native collision transfer packet to PR sections`
- this `RESUME.md` update may move `HEAD`; run `git log -10 --oneline` for the
  exact current hash.

The compaction preserved the active no-PR boundary, current completion
checklist, evidence map, round/question anchors, and finalization blockers
while removing detailed historical transcripts now available through git
history.

## Current Branch

`feature/new_coll` tracks `origin/feature/new_coll`. Before this resume update,
local `HEAD` was `d2a0d842a3f` and `origin/feature/new_coll` was
`e0304211446`. After committing this file, the local branch should be ahead by
one additional docs-only handoff commit. Run `git log -10 --oneline --decorate`
for the exact current local head. Do not push without explicit
maintainer/user approval.

PR #2652 remains closed, draft, based on `main`, and anchored to old head
`714d220d82a`. Feature-branch pushes do not start the main PR workflows while
that PR is closed.

## Immediate Next Step

Within the current no-PR scope, the branch-local evidence packet is current.
Stop at the completion boundary unless another small local docs issue is found:
the remaining work requires explicit maintainer/user approval for the final
review surface.

To finish the north-star task, the maintainer/user must choose a final PR/CI
surface. After that, collect final CI/artifact evidence, transfer the evidence
packet into the PR description, run final validation on the completing state,
and delete `docs/dev_tasks/native_collision/` in that same completing PR.

## Current Evidence

Latest merged-branch validation:

- `f949b7cbbbe` includes upstream `main` at `b218b43786c`.
- `f949b7cbbbe` passed `pixi run lint`.
- `f949b7cbbbe` passed
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  with all 6 top-level gates green and `All tests passed!`.
- The C++ unit-test phase reported 266/266 tests passing, including 29
  `collision-native` tests and 2 `collision-native-stability` tests.
- The simulation-experimental C++ phase reported 13/13 tests passing.

Latest focused feature-code validation:

- `08a3ee5555c` passed
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  with all 6 top-level gates green and `All tests passed!`.
- Focused reruns passed `test_ccd` 62/62, `test_capsule_capsule` 18/18, and
  `test_mesh_mesh` 11/11.

Latest docs/PR-packet validation:

- `pixi run lint` passed after each committed compaction slice.
- `git diff --check` passed before each commit.
- The lint gate reran `check-collision-runtime-isolation` and
  `audit-collision-compat-facades`; both passed.
- `pixi run check-lint-md` passed during the compaction checks.
- The PR staging packet now points to evidence-owner files instead of copying
  long historical run lists.
- `PR-DRAFT.md` and `07-pr-evidence-transfer.md` now use PR-template section
  names for Summary, Changes / Key Changes, Testing, Breaking Changes,
  Related Issues / PRs (backports), and Checklist.
- Pre-record docs-packet head `23dbbccc6c0` passed
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  with all 6 top-level gates green and `All tests passed!`.

## Context That Would Be Lost

- `DartCollisionDetector` is canonical. Legacy C++ factory keys, detector
  classes, headers, and package components are native-backed migration facades,
  not external runtime backend selectors.
- dartpy intentionally exposes only the clean `DartCollisionDetector` API.
- Per-engine FCL/Bullet/ODE collision build switches are gone from the normal
  build surface. `DART_BUILD_COLLISION_REFERENCE_TESTS` and
  `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` are the explicit opt-in gates for
  reference comparison components.
- Remaining old-engine implementation files live under
  `tests/dart/test/reference_collision/{fcl,bullet,ode}/` and are intentional
  reference test/benchmark code.
- `06-completion-audit.md` is the current deletion-gate checklist.
- `PR-DRAFT.md` and `07-pr-evidence-transfer.md` stage the successor PR body.

## How To Resume

```bash
git checkout feature/new_coll
git status --short --branch
git log -10 --oneline --decorate
git rev-parse HEAD
git rev-parse origin/feature/new_coll
gh pr list --repo dartsim/dart --head feature/new_coll --state all \
  --json number,title,state,isDraft,headRefOid,baseRefName,url
gh run list --repo dartsim/dart --branch feature/new_coll \
  --commit "$(git rev-parse HEAD)" \
  --json databaseId,workflowName,status,conclusion,headSha,event,url --limit 20
```

Then read:

1. `docs/dev_tasks/native_collision/06-completion-audit.md`
2. `docs/dev_tasks/native_collision/README.md`
3. `docs/dev_tasks/native_collision/PR-DRAFT.md`
4. `docs/dev_tasks/native_collision/07-pr-evidence-transfer.md`

If you make another local commit, run `pixi run lint` first. Push only after
explicit user/maintainer approval. Do not create or reopen a PR.
