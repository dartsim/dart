# Native Collision Completion Audit

This audit is the checkpoint to read before deciding whether
`docs/dev_tasks/native_collision/` can be deleted. It maps the north-star
requirements to concrete artifacts and remaining evidence gaps. Historical
run-by-run details live in git history and `03-evidence-gates.md`; this file
keeps only the current decision surface.

## Objective Restatement

The native-collision task is complete only when DART's normal collision runtime
is one built-in `dart` detector stack, FCL/Bullet/ODE are absent from normal
runtime builds, retained old names are native-backed compatibility facades, and
the built-in layer is proven feature-complete, correct, scalable, and
performance-ready through tests, benchmark guardrails, package/wheel checks,
downstream compatibility, PR/CI artifacts, and final dev-task cleanup.

Current user-scoped pass remains branch-local and no-PR: keep the
implementation/evidence packet current on `feature/new_coll`, keep
`PR-DRAFT.md` and `07-pr-evidence-transfer.md` ready, and do not open/reopen a
PR, trigger workflows, mutate PR metadata, or delete this folder without
explicit maintainer/user approval.

## Completion Decision

Status: **not complete**.

Local branch evidence is strong, but the north-star completion bar still has
external finalization gates. This folder must remain until final PR/CI evidence
exists, evidence is transferred to the completing PR, and this working-doc
folder is deleted in that same PR.

## Prompt-To-Artifact Checklist

| Requirement / gate                                                                  | Evidence                                                                                                                                                                                                            | Result                                                                                                         |
| ----------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| Native `dart` detector is the normal runtime path.                                  | `README.md`, `03-evidence-gates.md`, default-detector tests, and recorded full validation on the latest feature-code heads.                                                                                         | Local satisfied; final PR validation still required.                                                           |
| Legacy FCL/Bullet/ODE names are compatibility facades, not runtime backend choices. | `audit-collision-compat-facades`, `check-collision-runtime-isolation`, C++ facade tests, package smoke, and link scans.                                                                                             | Local satisfied; final downstream/PR evidence still required.                                                  |
| Feature-level collision correctness covers the current DART surface.                | `09-test-coverage-matrix.md` records no active feature-level GAP/PARTIAL rows; focused raw, convex, mesh, batch, `hello_world`, Atlas Simbicon, and capsule-box validations are recorded in `03-evidence-gates.md`. | Local satisfied for this feature-level pass; explicitly deferred rows belong to later performance/scope waves. |
| gz-physics compatibility remains green.                                             | Fresh local `pixi run -e gazebo test-gz` passes are recorded, including 65/65 tests and `readelf` checks showing `libdart-collision-native.so` without old collision/reference runtime deps.                        | Local satisfied; final PR/downstream CI evidence open.                                                         |
| Benchmark/profiling guardrails exist without making optimization part of this pass. | `bm-collision-check`, raw narrow-phase audit JSON, convexity/mesh harness audits, and benchmark-output-consumption checks are recorded in `03-evidence-gates.md` and `09-test-coverage-matrix.md`.                  | Local satisfied for guardrails; final benchmark artifact evidence on the PR surface open.                      |
| Clean package, wheel, and Python surfaces are preserved.                            | Wheel verifier wiring, clean dartpy API audit, package metadata checks, native compatibility package smoke, and repaired-head wheel-matrix reference evidence.                                                      | Local satisfied; final PR-state wheel/package evidence open.                                                   |
| Key task insights are extracted to durable onboarding docs.                         | `docs/onboarding/architecture.md`, `docs/onboarding/build-system.md`, `docs/onboarding/python-bindings.md`, and `docs/onboarding/building.md` carry the durable native-collision decisions.                         | Local satisfied; dev-task folder deletion still waits for completing PR.                                       |
| Completion packaging is done in the same PR.                                        | `PR-DRAFT.md` and `07-pr-evidence-transfer.md` stage the PR packet; `docs/dev_tasks/README.md` requires deleting this folder in the completing PR.                                                                  | Open; requires maintainer-selected PR/CI surface and explicit approval.                                        |

## Current Evidence Snapshot

Latest branch-local code evidence:

- Latest current-branch validation head after merging current upstream `main`:
  `f949b7cbbbe` (`Merge current main into native collision branch`).
- Upstream `main` included at `b218b43786c`.
- Recorded validation on that merged head: `pixi run lint` plus
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  passed all 6 top-level gates and printed `All tests passed!`.
- The C++ unit-test phase reported 266/266 tests passing, including 29
  `collision-native` tests and 2 `collision-native-stability` tests.
- The simulation-experimental C++ phase reported 13/13 tests passing.
- Focused feature-code reruns against `08a3ee5555c`
  (`Fix native capsule mesh CCD`) passed `test_ccd` 62/62,
  `test_capsule_capsule` 18/18, and `test_mesh_mesh` 11/11.
- Later local changes after `f949b7cbbbe`, if any, should be treated as
  docs/evidence-only unless recent git history shows otherwise.

Latest branch-local evidence packet:

- `PR-DRAFT.md` and `07-pr-evidence-transfer.md` stage the successor PR body
  using the repository PR-template sections.
- `RESUME.md` records the no-PR boundary, exact commands for resuming, and the
  local PR-packet alignment handoff. Run `git log -10 --oneline` for the exact
  current local head.
- `SUPERVISOR.md` has been compacted per Round 20 so current guidance is not
  buried under historical side-channel detail.
- Pre-record docs-packet head `23dbbccc6c0`
  (`Refresh native collision resume after PR packet cleanup`) passed
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  with all 6 top-level gates green and `All tests passed!`. The C++ unit-test
  phase reported 266/266 passing tests, including `collision-native` and
  `collision-native-stability`.

Latest local guard evidence from the compaction pass:

- `pixi run lint` passed.
- `git diff --cached --check` passed before the `f949b7cbbbe` merge commit.
- `check-collision-runtime-isolation` passed.
- `audit-collision-compat-facades` passed and confirmed factory aliases,
  C++ facades, dartpy API, package components, and reference-engine locations.

Latest read-only review-surface evidence:

- `gh pr list --repo dartsim/dart --head feature/new_coll --state all`
  returns only PR #2652.
- PR #2652 is `CLOSED`, draft, based on `main`, and anchored to old head
  `714d220d82a`.
- Read-only `gh run list --repo dartsim/dart --commit <current-local-head>`
  lookups for local-only heads returned no runs. Rerun the command from
  `RESUME.md` for the exact current local head.
- No PR metadata, workflow state, branch state, push, or GitHub artifact was
  mutated by read-only audits.

## Current Scope And Deferred Finalization

Current no-PR scope:

1. Keep the branch-local evidence packet current.
2. Keep `PR-DRAFT.md` and `07-pr-evidence-transfer.md` ready.
3. Use local validation as the working gate while PR #2652 remains closed.

Deferred finalization scope:

1. Maintainer-selected PR/CI surface exists.
2. Final native-only, gz-physics, wheel/package, benchmark, platform, and
   downstream/deprecation evidence is collected on that surface.
3. Final validation runs on the completing code state, including at least
   `pixi run lint` and `pixi run test-all`.
4. Evidence is transferred into the PR description.
5. `docs/dev_tasks/native_collision/` is deleted in the same completing PR.

## Completion Bar

Do not mark this dev task complete until every checklist row is either `Done`
with durable docs evidence or backed by PR/CI artifacts that close the remaining
local/open statuses. Until then, `docs/dev_tasks/native_collision/` remains
active working documentation.
