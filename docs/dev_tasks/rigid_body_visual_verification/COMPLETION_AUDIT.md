# Rigid Body Visual Verification Completion Audit

Date: 2026-06-12

This audit maps the active rigid-body GUI visual-verification objective to
current repository evidence. It is intentionally conservative: the dev task is
locally review-ready, but not complete until maintainer acceptance and the
completion PR cleanup happen.

## Objective Requirements

| Requirement                                                                                    | Evidence inspected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Status                                        |
| ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------- |
| Curate rigid-body `py-demos` as the first DART 7 visual debugging path.                        | PLAN-103 sidecar owns a 36-row World Rigid Body workflow from `rigid_body` through `rigid_loop_closure`; `python/examples/demos/README.md` documents the same ordered path; branch default py-demos front door is guarded by tests.                                                                                                                                                                                                                                                                                                                                | Proven locally                                |
| Cover key rigid-body features, performance, corner cases, and deal-breaker behavior.           | The sidecar rows cover baseline, modes, free flight, frames, external and point loads, timestep sensitivity, step diagnostics, contact budget, restitution/material/friction/spin/stack behavior, contact inspection, query options, casts, solver family comparison, executor equivalence, contact-policy comparison, multibody contact, manipulation, kinematic drivers, fixed/breakable/distance/limited joints, motor/limit/passive/screw-joint behavior, dynamics terms, COM offsets, Jacobians, multibody solver routing, and loop-closure family selection. | Proven locally for current scoped public APIs |
| Let users switch or compare solvers/backends/parameters where it answers a practical question. | Rows label comparison axes and held-fixed controls in the panel and capture metrics; examples include solver family, executor, contact policy, timestep, contact workload, passive parameter family, multibody solver family, and loop-closure family/policy rows.                                                                                                                                                                                                                                                                                                 | Proven locally                                |
| Keep unsupported or misleading GUI claims out of the workflow.                                 | PLAN-103 public API audit records no public direct `RigidBody` impulse, no sleep/wake or island activation, and no loop-closure compliance/stiffness/damping surfaces; the workflow search routes those terms to nearest rows with explicit caveats instead of adding speculative rows.                                                                                                                                                                                                                                                                            | Proven locally                                |
| Provide reviewer-facing visual evidence.                                                       | `build/captures/rigid_workflow_rows_01_36_1781309127/manifest.json` reports `status=complete`, `capture_count=36`, `failed_count=0`, `guidance_complete=true`, and 2388 PNG frames. `build/captures/rigid_workflow_optional_rows_37_52_1781309448/manifest.json` reports `status=complete`, `capture_count=16`, `failed_count=0`, `guidance_complete=true`, related/IPC-shelf/packet groups enabled, and 1004 PNG frames.                                                                                                                                          | Proven locally                                |
| Make static review artifacts self-contained enough for maintainer scan.                        | Current review-index asset audit found 181/181 local assets present for rows 01-36 and 81/81 present for optional rows 37-52; manifests and review headers record the exact top-level workflow commands.                                                                                                                                                                                                                                                                                                                                                           | Proven locally                                |
| Keep durable docs current.                                                                     | PLAN-103 sidecar, `python/examples/demos/README.md`, `CHANGELOG.md`, `docs/dev_tasks/rigid_body_visual_verification/README.md`, `RESUME.md`, and `PR_DRAFT.md` record the current workflow, evidence, caveats, and PR-ready state.                                                                                                                                                                                                                                                                                                                                 | Proven locally                                |
| Provide automated drift guards.                                                                | Focused docs/API drift guard, capture/review-index guard, and mandatory `pixi run lint` are recorded in the PR draft and recent local commits.                                                                                                                                                                                                                                                                                                                                                                                                                     | Proven locally for focused scope              |
| Open/publish maintainer review with DART 7.0 milestone.                                        | `gh pr list --head "$(git branch --show-current)"` currently returns no PR; branch is local-only and ahead of origin. Push and PR creation require explicit approval.                                                                                                                                                                                                                                                                                                                                                                                              | Missing external action                       |
| Maintainer accepts the 36-row workflow plus optional 52-row packet as the completed scope.     | `PR_DRAFT.md` includes the explicit maintainer acceptance checkbox. No maintainer acceptance has been recorded in the current repository state.                                                                                                                                                                                                                                                                                                                                                                                                                    | Missing external decision                     |
| Retire the dev-task folder in the completing PR.                                               | `docs/dev_tasks/README.md` requires deletion in the completing PR after durable artifacts are promoted. This folder remains active because acceptance is not recorded yet.                                                                                                                                                                                                                                                                                                                                                                                         | Not yet done by design                        |

## Current Local Evidence

- Branch: `feature/rigid-body-gui-visual-verification`
- Current local checkpoint before this audit: `df721612138 Mark rigid workflow changelog readiness`
- Current PR state: no PR for this branch from `gh pr list --head "$(git branch --show-current)"`
- Full review packet:
  `build/captures/rigid_workflow_rows_01_36_1781309127/review_index.html`
- Optional review packet:
  `build/captures/rigid_workflow_optional_rows_37_52_1781309448/review_index.html`
- PR body seed:
  `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

## Completion Decision

Do not mark the thread goal complete yet. The local rigid-body visual workflow
is review-ready for the scoped public API surface, but final completion still
requires:

1. explicit approval to push and create/update the PR;
2. the PR milestone set to `DART 7.0`;
3. maintainer acceptance that the maintained 36-row workflow plus optional
   52-row packet is the completed scope for this dev task; and
4. the completing PR cleanup that removes
   `docs/dev_tasks/rigid_body_visual_verification` after any final durable
   close-out note is promoted.

Until those happen, keep future local work limited to evidence repair,
review-prep, or a newly unblocked public API gap. Do not add speculative
numbered rows merely to keep the branch moving.
