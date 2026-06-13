# Rigid Body Visual Verification Completion Audit

Date: 2026-06-12

This audit maps the active rigid-body GUI visual-verification objective to
current repository evidence. It is intentionally conservative: the dev task is
locally review-ready, but not complete until maintainer acceptance and the
completion PR cleanup happen.

Latest local implementation slice: row 30,
`rigid_joint_passive_parameters`, now exposes the passive-joint force controls
that were already modeled in capture and replay state. The GUI adds
`Hold force` for the stiction lane and `Armature drive force` for the
direct-versus-armature lanes, applies both through the existing reset path, and
keeps capture metrics recording `hold_force` and `armature_force`. Focused
passive-row guards reported `5 passed`. Fresh single-scene visual evidence
lives at `build/captures/rigid_joint_passive_controls_1781325827`: 120
requested UI frames, 119 PNG frames written, screenshot written,
`scene_metrics.event_count=120`, and latest scene metrics containing
`hold_force=3.0` plus `armature_force=6.0` under the contact-free
passive-parameter row scope.

Latest local implementation slice: row 2, `rigid_body_modes`, now exposes the
same explicit comparison-axis and held-fixed context pattern used by the later
rigid workflow rows. The panel and capture metrics label
`comparison_axis=rigid_body_mode_semantics` and record the solver, executor,
gravity scale, force magnitude, body mass, and time step held fixed across the
dynamic/static/kinematic lanes. Focused row/docs guards reported `4 passed`.
Fresh single-scene visual evidence lives at
`build/captures/rigid_body_modes_comparison_axis_1781325323`: 72 requested UI
frames, 71 PNG frames written, screenshot written, `scene_metrics.event_count=72`,
and latest scene metrics containing the comparison axis plus held-fixed
context.

Latest local implementation slice: row 26, `rigid_joint_breakage`, now exposes
the public `Joint.break_force` parameter directly through a log-scale
`Break force log10(N)` GUI slider. Capture metrics and replay state record the
active break threshold, and the panel can reset with the current threshold
without losing the existing locked-reset and weak-rearm paths. The workflow
scope now describes this as an AVBD-pinned editable-threshold breakage row, not
a fixed-threshold caveat. Focused breakage-row guards reported `4 passed`, and
adjacent docs/workflow consistency guards reported `3 passed`. Fresh
single-scene visual evidence for the changed row lives at
`build/captures/rigid_joint_breakage_editable_threshold_1781324921`: 48
requested UI frames, 47 PNG frames written, screenshot written, and
`scene_metrics.event_count=48` with workflow guidance scope matching the
editable-threshold row.

Latest validation evidence: after merging latest `origin/main` (`Already up to
date`), the default command
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run test-all`
passed all six wrapper gates on current branch state: linting, build, unit
tests, simulation tests, Python tests, and documentation. The same run detected
a CUDA runtime and warned to run `pixi run -e cuda test-all` after default
validation. The current full rows 01-36 packet was then regenerated at
`build/captures/rigid_workflow_rows_01_36_1781323428` and now reports complete
scene metrics plus complete resolved solver identity under the DART 7
work-packet harness. The prior stopped wrapper attempt remains recorded below
as historical context, not as current verification state.

Latest remote publication state: use `git status -sb` as the source of truth
for local/remote parity. The last recorded remote checkpoint before this
validation-language correction was
`6bafd605907 Add pre-contact surrogate visual demo`; later handoff/docs commits
may be present when the branch matches
`origin/feature/rigid-body-gui-visual-verification`. Fresh
`gh pr list --head "$(git branch --show-current)"` and `gh pr status` checks
still reported no PR for the branch. The latest pre-push
`git fetch origin main && git merge --no-edit origin/main` reported
`Already up to date`, so the branch remained aligned with the PR #2986 DART 7
architecture/work-packet harness at publication time. The approved push did not
approve PR creation, milestone mutation, CI reruns, review comments, thread
resolution, or other GitHub review-state changes.

Latest local artifact audit: a read-only audit rechecked both current review
packet manifests and static review indexes. The rows 01-36 packet still reports
`status=complete`, `capture_count=36`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`scene_metrics_count=36`, `resolved_solver_identity_complete=true`,
`resolved_solver_identity_count=36`, and 181/181 local review-index assets
present. The refreshed optional rows 37-53 packet reports
`status=complete`, `capture_count=17`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`scene_metrics_count=17`, `resolved_solver_identity_complete=true`,
`resolved_solver_identity_count=17`, and 86/86 local review-index assets
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

| Requirement                                                                                      | Evidence inspected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Status                                        |
| ------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------- |
| Curate rigid-body `py-demos` as the first DART 7 visual debugging path.                          | PLAN-103 sidecar owns a 36-row World Rigid Body workflow from `rigid_body` through `rigid_loop_closure`; `python/examples/demos/README.md` documents the same ordered path; branch default py-demos front door is guarded by tests.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Proven locally                                |
| Cover key rigid-body features, performance, corner cases, and deal-breaker behavior.             | The sidecar rows cover baseline, modes, free flight, frames, external and point loads, timestep sensitivity, step diagnostics, contact budget, restitution/material/friction/spin/stack behavior, contact inspection, query options, casts, solver family comparison, executor equivalence, contact-policy comparison, multibody contact, manipulation, kinematic drivers, fixed/breakable/distance/limited joints, motor/limit/passive/screw-joint behavior, dynamics terms, COM offsets, Jacobians, multibody solver routing, and loop-closure family selection.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Proven locally for current scoped public APIs |
| Let users switch or compare solvers/backends/parameters where it answers a practical question.   | Rows label comparison axes and held-fixed controls in the panel and capture metrics; examples include solver family, executor, contact policy, timestep, contact workload, passive parameter family, multibody solver family, and loop-closure family/policy rows.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Proven locally                                |
| Follow the DART 7 harness solver-identity and scene-metrics evidence rule for generated packets. | `py-demo-capture` now promotes validated `resolved_solver_identity` from latest scene metrics into successful per-scene manifests, requires solver-family plus context fields for workflow identity completeness, attaches identity and scene-metrics evidence to captured workflow rows, summarizes both completeness checks in workflow manifests, shows warning blocks in `review_index.html`, and returns failure status for non-dry-run workflow packets that capture rows without latest metrics or valid solver identity. Focused capture guards reported `11 passed`. The native `SIGBUS` was traced to Filament's Linux `/tmp`-backed CircularBuffer path under tmpfs user-quota pressure, and Linux headless engine creation now forces Filament's anonymous soft CircularBuffer fallback. The regenerated rows 01-36 packet and refreshed optional rows 37-53 packet both report `resolved_solver_identity_complete=true`, zero missing solver identities, zero scene-metrics gaps, and zero failed rows; a read-only per-scene audit found latest scene metrics for every captured row in both packet directories. | Proven locally                                |
| Keep unsupported or misleading GUI claims out of the workflow.                                   | PLAN-103 public API audit records no public direct `RigidBody` impulse, no sleep/wake or island activation, and no loop-closure compliance/stiffness/damping surfaces; the workflow search routes those terms to nearest rows with explicit caveats instead of adding speculative rows.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Proven locally                                |
| Provide reviewer-facing visual evidence.                                                         | `build/captures/rigid_workflow_rows_01_36_1781323428/manifest.json` reports `status=complete`, `capture_count=36`, `failed_count=0`, `guidance_complete=true`, `scene_metrics_complete=true`, `resolved_solver_identity_complete=true`, and 2388 PNG frames. `build/captures/rigid_workflow_optional_rows_37_53_1781321474/manifest.json` reports `status=complete`, `capture_count=17`, `failed_count=0`, `guidance_complete=true`, `scene_metrics_complete=true`, `resolved_solver_identity_complete=true`, related/IPC-shelf/packet groups enabled, and 1027 PNG frames across the current 53-row optional span including `diff_pre_contact_surrogate`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Proven locally                                |
| Make static review artifacts self-contained enough for maintainer scan.                          | Current review-index asset audit found 181/181 local assets present for rows 01-36 and 86/86 present for optional rows 37-53; manifests and review headers record the exact top-level workflow commands.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Proven locally                                |
| Keep durable docs current.                                                                       | PLAN-103 sidecar, `python/examples/demos/README.md`, `CHANGELOG.md`, `docs/dev_tasks/rigid_body_visual_verification/README.md`, `RESUME.md`, and `PR_DRAFT.md` record the current workflow, evidence, caveats, and PR-ready state. PLAN-103 now includes a formal `WP-103.1` work-packet section with objective, scope, non-goals, acceptance evidence, gates, dependencies, and evidence pointers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Proven locally                                |
| Provide automated drift guards.                                                                  | Focused docs/API drift guards, scene-metrics route guards, capture/review-index guards, current-head `pixi run lint`, and the current-head focused capture-plan/docs drift guard (`6 passed`) are recorded in the PR draft and recent local commits. Current-head broad default `pixi run test-all` passed under `DART_SAFE_JOBS=5`; CUDA validation is pending after this evidence refresh until `pixi run -e cuda test-all` is rerun.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Proven locally; CUDA rerun pending            |
| Record the AI principle audit for substantial AI-assisted work.                                  | This file and `PR_DRAFT.md` record the objective, assumptions, source-of-truth docs, public `pixi run ...` paths, evidence packets, and shared-state safety status.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Proven locally                                |
| Open/publish maintainer review with DART 7.0 milestone.                                          | `gh pr list --head "$(git branch --show-current)"` and `gh pr status` still reported no PR in the latest recorded checks. The explicit push requests covered publishing the branch only, not PR creation, milestone mutation, CI rerun, or review-thread mutation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | PR/milestone external action still missing    |
| Maintainer accepts the 36-row workflow plus optional 53-row packet as the completed scope.       | `PR_DRAFT.md` includes the explicit maintainer acceptance checkbox. No maintainer acceptance has been recorded in the current repository state.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Missing external decision                     |
| Retire the dev-task folder in the completing PR.                                                 | `docs/dev_tasks/README.md` requires deletion in the completing PR after durable artifacts are promoted. This folder remains active because acceptance is not recorded yet.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Not yet done by design                        |

## Current Local Evidence

- Branch: `feature/rigid-body-gui-visual-verification`
- Latest merge check: `git fetch origin main && git merge --no-edit origin/main`
  reported `Already up to date`.
- Current remote publication state: verify with `git status -sb`. The last
  recorded remote checkpoint before this validation-language correction was
  `6bafd605907`; later handoff/docs commits may be present when the branch
  matches `origin/feature/rigid-body-gui-visual-verification`. No PR for this
  branch from the latest recorded
  `gh pr list --head "$(git branch --show-current)"` or `gh pr status` checks.
- Current-head broad default validation: `pixi run test-all` passed all six
  wrapper gates with `DART_SAFE_JOBS=5`; the simulation stage included
  `test_world` and `test_rigid_ipc_paper_experiments` as green.
- Current-head focused validation: the focused capture-plan/docs drift guard
  reported `6 passed`; the full rows 01-36 packet and optional rows 37-53
  packet regenerated successfully with complete scene metrics and solver
  identity.
- CUDA validation: a CUDA runtime is visible on this host; rerun
  `pixi run -e cuda test-all` after this evidence refresh is committed before
  publication if exact current-head CUDA evidence is required.
- Historical stopped validation state: the interrupted `test-all` wrapper is
  not a pass and has been superseded by current green runs.
- Full review packet:
  `build/captures/rigid_workflow_rows_01_36_1781323428/review_index.html`
- Optional review packet:
  `build/captures/rigid_workflow_optional_rows_37_53_1781321474/review_index.html`
- Current refresh state: `build/captures/rigid_workflow_rows_01_36_1781311276`
  failed before metrics and remains failure evidence only. The underlying
  native `SIGBUS` was traced to Filament's `/tmp`-backed Linux CircularBuffer
  path under tmpfs user-quota pressure. Linux headless engine creation now
  forces Filament's soft CircularBuffer fallback, and the current full plus
  optional packets regenerated successfully under `build/captures/`.
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
  audits, focused pytest guards, solver-identity/scene-metrics enforcement
  guards, and `pixi run lint` are recorded in the PR draft and this audit.
- Shared-state safety: branch pushes were explicitly approved by the user. PR
  creation, milestone mutation, review comments, CI re-triggers, thread
  resolution, and merges still require explicit approval.

## Completion Decision

Do not mark the thread goal complete yet. The local rigid-body visual workflow
is review-ready for the scoped public API surface, but final completion still
requires:

1. explicit approval to create/update the PR;
2. the PR milestone set to `DART 7.0`;
3. maintainer acceptance that the maintained 36-row workflow plus optional
   53-row packet is the completed scope for this dev task; and
4. the completing PR cleanup that removes
   `docs/dev_tasks/rigid_body_visual_verification` after any final durable
   close-out note is promoted.

Until those happen, keep future local work limited to evidence repair,
review-prep, or a newly unblocked public API gap. Do not add speculative
numbered rows merely to keep the branch moving.
