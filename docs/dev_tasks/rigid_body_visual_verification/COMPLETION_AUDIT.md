# Rigid Body Visual Verification Completion Audit

Date: 2026-06-12

This audit maps the active rigid-body GUI visual-verification objective to
current repository evidence. It is intentionally conservative: the dev task is
locally review-ready, but not complete until maintainer acceptance and the
completion PR cleanup happen.

Latest validation evidence: after the stop-only handoff, current `HEAD` was
validated with `DART_SAFE_JOBS=5`. The default command
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run test-all`
completed successfully with all six wrapper gates passing: linting, build, unit
tests, simulation tests, Python tests, and documentation. Because the host
exposes `GPU 0: NVIDIA GeForce RTX 4080 Laptop GPU`, the CUDA command
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run -e cuda test-all`
was also run and exited successfully. The prior stopped wrapper attempt remains
recorded below as historical context, not as current verification state.

Latest remote publication state: after the explicit `push to origin`
instruction and the final local handoff amend,
`origin/feature/rigid-body-gui-visual-verification` points to
`d44031e7040 Refresh rigid workflow review handoff`. Fresh
`gh pr list --head "$(git branch --show-current)"` and `gh pr status` checks
still report no PR for the branch. The latest pre-push
`git fetch origin main && git merge --no-edit origin/main` reported
`Already up to date`, so the branch remained aligned with the PR #2986 DART 7
architecture/work-packet harness at publication time. This current-state docs
correction may be an unpublished local commit on top of that remote checkpoint;
rerun `git status -sb` before acting. The approved push did not approve PR
creation, milestone mutation, CI reruns, review comments, thread resolution, or
other GitHub review-state changes.

Latest local artifact audit: a read-only audit rechecked both current review
packet manifests and static review indexes. The rows 01-36 packet still reports
`status=complete`, `capture_count=36`, `failed_count=0`,
`guidance_complete=true`, `resolved_solver_identity_complete=true`,
`resolved_solver_identity_count=36`, and 181/181 local review-index assets
present. The optional rows 37-52 packet still reports `status=complete`,
`capture_count=16`, `failed_count=0`, `guidance_complete=true`,
`resolved_solver_identity_complete=true`,
`resolved_solver_identity_count=16`, and 81/81 local review-index assets
present.

Latest publication-readiness audit: `git fetch origin main && git merge
--no-edit origin/main` still reports `Already up to date`; `gh pr status` still
reports no PR for `feature/rigid-body-gui-visual-verification`; the open
milestone list includes `DART 7.0`; and `PR_DRAFT.md` is safe to pass as a body
file because the visible body starts at `## Summary` after the HTML comment and
contains Testing, Breaking Changes, Related Issues / PRs, milestone, and
completion-audit references.

Latest deferred-API audit: runtime `dartpy.simulation` symbol introspection and
targeted binding/stub reads still find only `RigidBody.apply_force()`,
`RigidBody.apply_torque()`, force/torque properties, rigid-body
linear/angular-momentum accessors, `Link.apply_force(...)`, and
`Multibody.compute_impulse_response(...)` for the relevant public surfaces.
No public `RigidBody` or `World` symbol exposes a direct rigid-body impulse,
sleep/wake, island activation, or loop-closure compliance/stiffness/damping
control, so the current workflow should keep those queries routed to caveated
nearest rows instead of adding speculative numbered rows.

Fresh focused guard for that audit:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_routes_deferred_api_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_deferred_api_gaps_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `6 passed`.

Latest stop/push handoff: the current session was redirected to stop code
changes and stop further verification, update only this handoff state, merge
latest `origin/main`, push to `origin`, and stop. `git fetch origin main &&
git merge --no-edit origin/main` reported `Already up to date`. The running
`test-all` wrapper was terminated by request during the simulation-labeled ctest
stage; it is not a completed validation pass. Visible progress before
termination included a successful direct `pixi run build-tests ON Release`
re-run, 219/219 C++ unit tests passed, and simulation output through 52/65
visible tests. CUDA validation was not run. The handoff docs refresh therefore
intentionally records no new green validation after the user stop instruction.

## Objective Requirements

| Requirement                                                                                    | Evidence inspected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Status                                        |
| ---------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------- |
| Curate rigid-body `py-demos` as the first DART 7 visual debugging path.                        | PLAN-103 sidecar owns a 36-row World Rigid Body workflow from `rigid_body` through `rigid_loop_closure`; `python/examples/demos/README.md` documents the same ordered path; branch default py-demos front door is guarded by tests.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Proven locally                                |
| Cover key rigid-body features, performance, corner cases, and deal-breaker behavior.           | The sidecar rows cover baseline, modes, free flight, frames, external and point loads, timestep sensitivity, step diagnostics, contact budget, restitution/material/friction/spin/stack behavior, contact inspection, query options, casts, solver family comparison, executor equivalence, contact-policy comparison, multibody contact, manipulation, kinematic drivers, fixed/breakable/distance/limited joints, motor/limit/passive/screw-joint behavior, dynamics terms, COM offsets, Jacobians, multibody solver routing, and loop-closure family selection.                                                                                                                                                    | Proven locally for current scoped public APIs |
| Let users switch or compare solvers/backends/parameters where it answers a practical question. | Rows label comparison axes and held-fixed controls in the panel and capture metrics; examples include solver family, executor, contact policy, timestep, contact workload, passive parameter family, multibody solver family, and loop-closure family/policy rows.                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Proven locally                                |
| Follow the DART 7 harness solver-identity evidence rule for generated packets.                 | `py-demo-capture` now promotes `resolved_solver_identity` from latest scene metrics into successful per-scene manifests, attaches it to captured workflow rows, summarizes identity completeness in workflow manifests, and shows it in `review_index.html`; focused unit/integration guards reported `4 passed`. The native `SIGBUS` was traced to Filament's Linux `/tmp`-backed CircularBuffer path under tmpfs user-quota pressure, and Linux headless engine creation now forces Filament's anonymous soft CircularBuffer fallback. The regenerated rows 01-36 packet and optional rows 37-52 packet both report `resolved_solver_identity_complete=true`, zero missing solver identities, and zero failed rows. | Proven locally                                |
| Keep unsupported or misleading GUI claims out of the workflow.                                 | PLAN-103 public API audit records no public direct `RigidBody` impulse, no sleep/wake or island activation, and no loop-closure compliance/stiffness/damping surfaces; the workflow search routes those terms to nearest rows with explicit caveats instead of adding speculative rows.                                                                                                                                                                                                                                                                                                                                                                                                                               | Proven locally                                |
| Provide reviewer-facing visual evidence.                                                       | `build/captures/rigid_workflow_rows_01_36_1781312968/manifest.json` reports `status=complete`, `capture_count=36`, `failed_count=0`, `guidance_complete=true`, `resolved_solver_identity_complete=true`, and 2388 PNG frames. `build/captures/rigid_workflow_optional_rows_37_52_1781313357/manifest.json` reports `status=complete`, `capture_count=16`, `failed_count=0`, `guidance_complete=true`, `resolved_solver_identity_complete=true`, related/IPC-shelf/packet groups enabled, and 1004 PNG frames.                                                                                                                                                                                                         | Proven locally                                |
| Make static review artifacts self-contained enough for maintainer scan.                        | Current review-index asset audit found 181/181 local assets present for rows 01-36 and 81/81 present for optional rows 37-52; manifests and review headers record the exact top-level workflow commands.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Proven locally                                |
| Keep durable docs current.                                                                     | PLAN-103 sidecar, `python/examples/demos/README.md`, `CHANGELOG.md`, `docs/dev_tasks/rigid_body_visual_verification/README.md`, `RESUME.md`, and `PR_DRAFT.md` record the current workflow, evidence, caveats, and PR-ready state.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Proven locally                                |
| Provide automated drift guards.                                                                | Focused docs/API drift guards and capture/review-index guards are recorded in the PR draft and recent local commits. Current broad validation also ran default `pixi run test-all` and CUDA `pixi run -e cuda test-all` successfully under `DART_SAFE_JOBS=5`.                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Proven locally                                |
| Record the AI principle audit for substantial AI-assisted work.                                | This file and `PR_DRAFT.md` record the objective, assumptions, source-of-truth docs, public `pixi run ...` paths, evidence packets, and shared-state safety status.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Proven locally                                |
| Open/publish maintainer review with DART 7.0 milestone.                                        | `gh pr list --head "$(git branch --show-current)"` and `gh pr status` still return no PR after the latest approved push through `d44031e7040`. The explicit push request covered publishing the branch only, not PR creation, milestone mutation, CI rerun, or review-thread mutation.                                                                                                                                                                                                                                                                                                                                                                                                                                | PR/milestone external action still missing    |
| Maintainer accepts the 36-row workflow plus optional 52-row packet as the completed scope.     | `PR_DRAFT.md` includes the explicit maintainer acceptance checkbox. No maintainer acceptance has been recorded in the current repository state.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Missing external decision                     |
| Retire the dev-task folder in the completing PR.                                               | `docs/dev_tasks/README.md` requires deletion in the completing PR after durable artifacts are promoted. This folder remains active because acceptance is not recorded yet.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Not yet done by design                        |

## Current Local Evidence

- Branch: `feature/rigid-body-gui-visual-verification`
- Latest merge check: `git fetch origin main && git merge --no-edit origin/main`
  reported `Already up to date`.
- Current remote publication state: origin branch is pushed through
  `d44031e7040`; this docs correction may be an unpublished local commit.
  No PR for this branch from `gh pr list --head "$(git branch --show-current)"`
  or `gh pr status`.
- Current broad validation: default `pixi run test-all` passed all wrapper
  gates, and CUDA `pixi run -e cuda test-all` exited successfully on the visible
  RTX 4080 Laptop GPU host.
- Historical stopped validation state: the interrupted `test-all` wrapper is
  not a pass and has been superseded by current green runs.
- Full review packet:
  `build/captures/rigid_workflow_rows_01_36_1781312968/review_index.html`
- Optional review packet:
  `build/captures/rigid_workflow_optional_rows_37_52_1781313357/review_index.html`
- Current refresh state: `build/captures/rigid_workflow_rows_01_36_1781311276`
  failed before metrics and remains failure evidence only. The underlying
  native `SIGBUS` was traced to Filament's `/tmp`-backed Linux CircularBuffer
  path under tmpfs user-quota pressure. Linux headless engine creation now
  forces Filament's soft CircularBuffer fallback, and the current full plus
  optional packets regenerated successfully with `/dev/shm` output before being
  mirrored into `build/captures/`.
- PR body seed:
  `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

## AI Principle Audit

- Objective: the current scoped deliverable is the rigid-body `py-demos`
  visual-verification workflow, its capture/review packet path, and PR-ready
  handoff for maintainer acceptance.
- Assumptions: unsupported direct rigid-body impulse, sleep/wake or island
  activation, and loop-closure compliance rows stay deferred until public
  `dartpy` APIs exist.
- Simplicity and scope: the workflow uses PLAN-103, py-demos, capture helper,
  and dev-task handoff surfaces already established on this branch; no new
  speculative row family is added by this audit.
- Source of truth: PLAN-103 owns the durable workflow definition, the py-demos
  README owns user-facing usage, generated manifests own capture evidence, and
  dev-task docs only own active handoff/review-prep state.
- Public path: all reviewer and user actions map to tracked docs and
  `pixi run ...` commands, including `py-demos` and `py-demo-capture`.
- Evidence: current full and optional workflow manifests, review-index asset
  audits, focused pytest guards, and `pixi run lint` are recorded in the PR
  draft and this audit.
- Shared-state safety: the branch push through `d44031e7040` was explicitly
  approved by the user. PR creation, milestone mutation, review comments, CI
  re-triggers, thread resolution, and merges still require explicit approval.

## Completion Decision

Do not mark the thread goal complete yet. The local rigid-body visual workflow
is review-ready for the scoped public API surface, but final completion still
requires:

1. explicit approval to create/update the PR;
2. the PR milestone set to `DART 7.0`;
3. maintainer acceptance that the maintained 36-row workflow plus optional
   52-row packet is the completed scope for this dev task; and
4. the completing PR cleanup that removes
   `docs/dev_tasks/rigid_body_visual_verification` after any final durable
   close-out note is promoted.

Until those happen, keep future local work limited to evidence repair,
review-prep, or a newly unblocked public API gap. Do not add speculative
numbered rows merely to keep the branch moving.
