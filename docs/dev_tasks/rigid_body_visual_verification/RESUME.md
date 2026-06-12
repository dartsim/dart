# Resume: Rigid-Body Visual Verification

## Current Checkpoint Snapshot - 2026-06-12 README Workflow Order

The latest continuation starts after local checkpoint `325280d4483`
(`Harden rigid workflow search and evidence docs`). It does not push or mutate
GitHub state.

Current local slice: README quick-workflow order labels. A read-only audit found
no major remaining numbered-row implementation gap, but did find that the
Python demo README quick table was ordered without visibly showing the same
`01/36` through `36/36` labels used by the interactive navigator and PLAN-103
sidecar. The README table now includes an `Order` column, and
`test_rigid_visual_verification_readme_matches_sidecar_order` now verifies the
README labels against the sidecar row numbers.

Resume-state handoff:

- The previous stop-only handoff has been superseded by a goal-continuation
  turn. Continue the local README-order slice unless the user explicitly
  redirects.
- Pushes, PR creation, comments, CI re-triggers, and other GitHub mutations
  still require explicit approval.
- A parallel read-only audit found no concrete remaining implementation gap in
  the non-numbered related/capture-first route surface. Its recommendation was
  PR readiness and dev-task cleanup, not more route implementation.

Validation collected before this resume-state docs edit:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q`
- reported `3 passed`.
- `pixi run lint` passed before the previous stop-only handoff docs edit.
- `git diff --check` passed before the previous stop-only handoff docs edit.

Fresh-session resume instructions:

1. Inspect the worktree and preserve the current five-file README-order slice
   unless the user explicitly redirects.
2. Re-run focused README-order tests, `pixi run lint`, and `git diff --check`;
   then locally commit the slice if clean.
3. After that checkpoint, either select the next bounded rigid GUI verification
   improvement or, if the user asks to publish/finish, get explicit approval
   before any push or GitHub mutation and settle the dev-task cleanup decision
   in the completing PR.

## Current Checkpoint Snapshot - 2026-06-12 PR-Readiness And Workflow Search

The latest continuation resumed from pushed checkpoint `f22db3b7751`
(`Record rigid capture ownership visual evidence`). It does not push or mutate
GitHub state.

Current local slice: PR-readiness/doc-drift and workflow-search hardening.
Fresh audits found no obvious next implementation row in the curated 36-row
rigid workflow, but did find two bounded issues:

- The durable PLAN-103 evidence matrix no longer explicitly named
  capture-metrics coverage for three rows that already expose
  `SceneSetup.info["capture_metrics"]` and have focused assertions:
  `rigid_step_diagnostics`, `rigid_contact_scale_budget`, and
  `rigid_collision_query_options`.
- The `Rigid Workflow` search could rank related-route negative caveats above
  intended positive solver rows and did not understand visible shorthand or
  backend/profile phrases such as `SI`, `step profile`, or
  `accelerated backend`.

What changed:

- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md` now
  names capture metrics for those three rows in the workflow table.
- `python/examples/demos/README.md` now explains the manifest-sidecar metric
  families captured for those rows.
- `python/tests/integration/test_demos_cycle.py` adds
  `test_rigid_visual_workflow_capture_metric_docs_match_hooks`, which verifies
  every numbered workflow row has a capture hook and PLAN-103 capture-metrics
  wording in the controls/evidence cells.
- `python/examples/demos/runner.py` now uses explicit search aliases for solver
  and backend vocabulary, treats one- and two-character search terms as exact
  words, and ranks related-evidence scene id/shelf/label matches without using
  caveat reasons as high-priority search fields.
- `python/tests/unit/test_py_demo_panels.py` now covers `sequential impulse`,
  `SI`, `step profile`, `accelerated backend`, and `memory diagnostics`
  searches.
- `CHANGELOG.md` records the workflow-search hardening.
- The dev-task folder remains active because this PR has not yet been opened or
  accepted and the final publication/cleanup decision still needs maintainer
  approval. The durable evidence owner is already the PLAN-103 sidecar; delete
  this folder in the completing PR only after final scope and cleanup are
  explicitly settled.

Validation collected so far:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks python/tests/integration/test_demos_cycle.py::test_rigid_visual_routes_publish_self_describing_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
- reported `4 passed`.
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_labels_related_evidence_search_matches -q`
- initially exposed the `contact`/alias ranking regressions while the fix was
  being tuned, then reported `4 passed`.
- Combined focused guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks python/tests/integration/test_demos_cycle.py::test_rigid_visual_routes_publish_self_describing_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_labels_related_evidence_search_matches -q`
- reported `8 passed`.
- `pixi run lint` passed after the final evidence update.
- Bounded default `pixi run build` passed with `DART safe jobs: 3` and
  `ninja: no work to do`.
- Full `pixi run test-py` passed with `953 passed, 10 skipped` using
  `DART safe jobs: 3`.
- `git diff --check` passed after the final evidence update.

Immediate next step:

1. Commit the current local PR-readiness/search-hardening slice if the working
   tree still shows only the files listed above.
2. After that local checkpoint, either continue the next bounded rigid
   visual-verification slice or publish/open the PR only after explicit
   maintainer/user approval.
3. Do not push, open a PR, comment, re-trigger CI, or mutate GitHub state
   unless the user explicitly approves it in the current turn.

## Current Checkpoint Snapshot - 2026-06-12 Capture Metric Ownership

The latest continuation resumed after the pushed full-stop handoff
`c552ce83e2b` because the active goal was explicitly continued. It does not
push or mutate GitHub state.

Code checkpoint before this evidence-refresh docs edit:
`601845c4197` (`Harden rigid visual capture metric ownership`).

Current local slice: self-describing capture metrics for rigid visual
verification routes. The continuation fixed three payload gaps:

- `rigid_ipc_tunnel` now records
  `related_source_row: rigid_solver_compare`.
- `avbd_rigid_breakable_joint` now records
  `related_source_row: rigid_joint_breakage`; the numbered
  `rigid_joint_breakage` wrapper passes `related_source_row=None` so the
  numbered row remains directly identified by `row: rigid_joint_breakage`.
- `rigid_ipc_stack_packet` now records `row: rigid_ipc_stack_packet`.

The new
`test_rigid_visual_routes_publish_self_describing_capture_metrics` invariant
builds the current sidecar-defined numbered workflow rows, related routes, and
capture-first IPC packets from `make_demo_scenes()` and verifies that capture
metrics remain self-identifying.

Validation collected so far:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_routes_publish_self_describing_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_ipc_tunnel_reports_no_tunneling_metrics python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/integration/test_demos_cycle.py::test_avbd_breakable_joint_demo_marks_and_resets_joint -q`
- reported `6 passed` before and after lint.
- Real docked captures under `/tmp/dart_capture_metric_ownership_1781249077`
  verified the manifest path for all three changed payloads:
  `rigid_ipc_tunnel` wrote 23 PNG frames and 24 scene-metrics events with row
  `rigid_ipc_tunnel` and related source row `rigid_solver_compare`;
  `rigid_ipc_stack_packet` wrote 23 PNG frames and 24 scene-metrics events
  with row `rigid_ipc_stack_packet` and `capture_first=true`; and
  `avbd_rigid_breakable_joint` wrote 71 PNG frames and 72 scene-metrics events
  with row `avbd_rigid_breakable_joint`, related source row
  `rigid_joint_breakage`, and broken-state evidence. All three screenshots
  were docked and nonblank.
- `pixi run lint` passed.
- `git diff --check` passed.

Immediate next step:

1. Continue the next bounded rigid visual-verification slice, or push/open the
   PR only after explicit user approval. Do not comment, re-trigger CI, or
   otherwise mutate GitHub state without that approval.

## Critical Stop Handoff - 2026-06-12 Full Stop

The user explicitly stopped all further work and requested handoff docs only,
with no additional verification. Treat this section as the fresh-session entry
point and do not continue implementation, validation, pushing, PR creation, or
other GitHub mutation unless the user explicitly resumes or asks for that exact
action.

State at the start of this handoff-only docs edit:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Worktree: clean before the docs handoff edit.
- Local checkpoint before this docs-only handoff: `5127bf7e6c73`
  (`Refresh rigid visual verification PR readiness`).
- Upstream checkpoint before this docs-only handoff: `4a2fb7a0714e`
  (`Refresh rigid visual verification stop handoff`).
- The local branch was ahead of origin by three commits:
  - `5127bf7e6c7` `Refresh rigid visual verification PR readiness`
  - `22d1a02de55` `Attach shared replay to related rigid scenes`
  - `7c853bee1af` `Expose fundamental rigid workflow capture metrics`
- Latest checked GitHub state still reported no PR associated with this
  branch.
- No tests, lint, build, captures, `git diff --check`, or other verification
  were run after this handoff-only docs edit.

Immediate next step:

1. Stop. If a future session is explicitly asked to publish, push the branch
   and use `PR_DRAFT.md` as the PR body source. If a future session is asked to
   continue implementation, resume from the local checkpoint above and keep
   GitHub mutations gated on explicit approval.

## Current Checkpoint Snapshot - 2026-06-11 Local PR-Readiness Refresh

After the shared Replay follow-up checkpoint, the branch was refreshed against
origin for local PR readiness without pushing or mutating GitHub state.

State and validation:

- `git fetch origin` completed.
- `origin/main` is already an ancestor of the local branch; no merge commit was
  needed.
- Latest checked GitHub state still reports no PR associated with
  `feature/rigid-body-gui-visual-verification`.
- Bounded `pixi run build` passed with `DART safe jobs: 2` and
  `ninja: no work to do`.
- Full `pixi run test-py` passed with `950 passed, 10 skipped` using
  `DART safe jobs: 1`.
- No push, PR creation, PR comment, review reply, CI retrigger, or other
  external mutation was performed.

Immediate next step:

1. If the maintainer approves publishing, push the branch and open/update the
   PR using `PR_DRAFT.md`; otherwise continue with the next bounded
   rigid-body visual-verification slice from the task state. Keep GitHub
   mutations gated on explicit approval.

## Current Checkpoint Snapshot - 2026-06-11 Shared Replay Follow-Up

The latest continuation lands a local checkpoint named
`Attach shared replay to related rigid scenes` after starting from
`7c853bee1af8` (`Expose fundamental rigid workflow capture metrics`). It closes
the shared Replay panel gap exposed by the broader Python demo suite. The
branch still has no associated GitHub PR in the latest checked state, and no
push or GitHub mutation has been performed for this follow-up.

Current local slice: shared Replay panel attachment for replay-capable related
rigid scenes with custom `pre_step` callbacks. The checkpoint touches:

- `python/examples/demos/scenes/articulated.py`
- `python/examples/demos/scenes/floating_base.py`
- `python/examples/demos/scenes/avbd_rigid_revolute_motor.py`
- `python/examples/demos/scenes/avbd_rigid_prismatic_motor.py`
- `python/examples/demos/scenes/rigid_ipc_tunnel.py`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code:

- `articulated`, `floating_base`, `avbd_rigid_revolute_motor`,
  `avbd_rigid_prismatic_motor`, and `rigid_ipc_tunnel` now publish
  `SceneSetup.info["replay_sync"] = bridge.sync` and
  `SceneSetup.info["replay_live_step_is_stateless"] = True`.
- These scenes use custom `pre_step` callbacks to run the bridge step and
  update derived metrics/histories, but they do not need separate
  replay-owned scene state to scrub saved DART 7 World frames. Marking them as
  stateless lets the runner attach the shared bottom `Replay` panel instead of
  silently skipping it.
- The triggering failure was
  `python/tests/unit/test_py_demo_panels.py::test_registered_world_scenes_receive_shared_replay_controls`,
  which first reported `articulated` and then a replay-gap audit identified
  the full affected set above.

Validation collected for this slice:

- `python -m py_compile` over the five edited scene files passed.
- The focused
  `python/tests/unit/test_py_demo_panels.py::test_registered_world_scenes_receive_shared_replay_controls`
  guard passed with `1 passed`.
- A replay-gap audit over `make_demo_scenes()` printed `[]`.
- A broader `pixi run test-py` rerun reported `950 passed, 10 skipped` before
  this docs evidence refresh.
- `pixi run lint` passed.
- `git diff --check` passed.

Immediate next step:

1. Continue the next bounded rigid visual-verification slice from the dev-task
   state, or push/open PR only after explicit user approval. Do not comment,
   re-trigger CI, or otherwise mutate GitHub state without that approval.

## Current Checkpoint Snapshot - 2026-06-11 Fundamental Workflow Metrics

The continuation resumed from pushed handoff commit `4a2fb7a0714` and closed
the remaining live capture-metrics gap in the early numbered World Rigid Body
workflow rows. All related-evidence routes were already covered; this slice
adds or normalizes the numbered fundamentals.

Current local slice: fundamental numbered-row capture metrics. The checkpoint
touches:

- `python/examples/demos/scenes/rigid_body_modes.py`
- `python/examples/demos/scenes/rigid_free_flight.py`
- `python/examples/demos/scenes/rigid_frame_hierarchy.py`
- `python/examples/demos/scenes/rigid_external_loads.py`
- `python/examples/demos/scenes/rigid_link_point_loads.py`
- `python/examples/demos/scenes/rigid_timestep_sensitivity.py`
- `python/examples/demos/scenes/rigid_restitution_ladder.py`
- `python/examples/demos/scenes/rigid_material_mixing.py`
- `python/examples/demos/scenes/rigid_kinematic_normal_push.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test:

- `rigid_body_modes`, `rigid_free_flight`, `rigid_frame_hierarchy`,
  `rigid_external_loads`, `rigid_link_point_loads`,
  `rigid_timestep_sensitivity`, `rigid_restitution_ladder`, and
  `rigid_material_mixing` now expose
  `SceneSetup.info["capture_metrics"]` payloads with row identity,
  solver/executor or scope, controls, per-lane metrics, compact history
  extrema, and top-level manifest-friendly fields.
- `rigid_kinematic_normal_push` now uses the shared
  `CAPTURE_METRICS_INFO_KEY` constant instead of a literal string.
- `rigid_joint_breakage` remains covered through
  `build_breakable_joint_scene(row_id="rigid_joint_breakage")`; no wrapper
  code change was needed beyond the focused test coverage that already asserts
  its capture hook.
- Existing focused integration tests for the affected rows now assert the new
  capture payload identity and representative finite top-level fields.

Validation collected so far for this slice:

- `python -m py_compile` over the changed scene files and
  `python/tests/integration/test_demos_cycle.py` passed.
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics python/tests/integration/test_demos_cycle.py::test_rigid_external_loads_scale_force_and_torque_response python/tests/integration/test_demos_cycle.py::test_rigid_free_flight_preserves_initial_state_diagnostics python/tests/integration/test_demos_cycle.py::test_rigid_frame_hierarchy_tracks_body_fixed_frame python/tests/integration/test_demos_cycle.py::test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size python/tests/integration/test_demos_cycle.py::test_rigid_restitution_ladder_orders_rebound_height python/tests/integration/test_demos_cycle.py::test_rigid_material_mixing_applies_pair_rules python/tests/integration/test_demos_cycle.py::test_rigid_link_point_loads_show_lever_arm_and_frame_semantics python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage -q`
  reported `9 passed`.
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets -q`
  reported `10 passed`.
- Real docked captures under `/tmp/dart_capture_*_fundamental_metrics_1781245202`
  wrote scene-metrics sidecars and docked first-frame visual evidence:
  `rigid_body_modes` 72 events with dynamic speed about `1.861`, static drift
  `0.0`, and kinematic error `0.0`; `rigid_free_flight` 96 events with
  near-zero drift residual, arc error about `0.00414`, and spin momentum ratio
  `4.0`; `rigid_frame_hierarchy` 72 events with zero world/relative/orientation
  residuals; `rigid_external_loads` 72 events with light/heavy acceleration
  about `4.0`/`1.0` and static drift `0.0`; `rigid_link_point_loads` 72 events
  with centered acceleration about `2.0` and off-center yaw acceleration about
  `-4.896`; `rigid_material_mixing` 72 events with effective restitution
  `0.82`, effective friction about `0.1897`, and slide speed loss about
  `0.525`; `rigid_restitution_ladder` 96 events with ordered dead/middle/high
  rebound heights about `0.0792`/`0.1183`/`0.1526`; and
  `rigid_timestep_sensitivity` 96 events with fine/coarse freefall errors about
  `1.508` and coarse/fine ratio about `1.0`.
- `pixi run lint` passed.
- Bounded default `pixi run build` passed with `DART safe jobs: 4` and
  `ninja: no work to do`.
- `git diff --check` passed.

Immediate next step:

1. Do not push, open a PR, comment, or mutate GitHub state
   unless the user explicitly approves it in the current turn.

## Critical Stop Handoff - 2026-06-11 World Related Metrics

The user explicitly stopped further work and requested handoff-only docs with
no further verification. Treat this section as the fresh-session entry point
and do not continue implementation unless the user explicitly resumes it.

State at the start of this handoff-only docs edit:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Worktree: clean before the docs handoff edit.
- Local code checkpoint: `7fb9163794d`.
- Local code checkpoint title: `Expose World related route capture metrics`.
- Origin at that moment: `550b37d8b68`
  (`Refresh rigid visual verification handoff`).
- The local branch was ahead of origin by one code/docs checkpoint before this
  handoff-only docs commit.
- Last checked GitHub state: no PR associated with this branch.
- This docs-only handoff commit is intended to be pushed immediately at the
  user's request, without a new verification pass.

No tests, lint, build, captures, or `git diff --check` were run after this
handoff-only docs edit. Do not treat the handoff docs commit itself as freshly
verified. The validated code checkpoint immediately below remains the current
implementation context.

## Current Handoff Snapshot - 2026-06-11 World Related Metrics

The continuation resumed from pushed handoff commit `550b37d8b68` and hardened
the remaining broad World related-evidence routes that were still
screenshot-only evidence: `floating_base` and `articulated`. These remain
non-numbered World Rigid Body shelf routes, not new numbered workflow rows.

Current local slice: broader World related-route capture metrics. The
checkpoint touches:

- `python/examples/demos/scenes/floating_base.py`
- `python/examples/demos/scenes/articulated.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test:

- `floating_base.py` now samples floating-joint SE(3) drift/spin metrics during
  `pre_step` and exposes them through `SceneSetup.info["capture_metrics"]`:
  related source row `rigid_free_flight`, linear speed, angular speed, body
  position, spin command, time step, world time, and compact history extrema.
- `articulated.py` now samples compact two-link arm metrics during `pre_step`
  and exposes them through the same capture hook: related source row
  `rigid_multibody_dynamics_terms`, DOFs, link count, shoulder/wrist speeds,
  forearm height, damping controls, joint positions, world time, and compact
  history extrema.
- `python/tests/integration/test_demos_cycle.py::test_world_related_evidence_routes_report_capture_metrics`
  asserts both related routes publish callable capture hooks, preserve
  related-source identities, expose finite manifest-friendly top-level fields,
  and accumulate history samples while stepping through the render bridge.

Validation collected so far for this slice:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_related_evidence_routes_report_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_labels_related_evidence_search_matches -q`
  reported `7 passed`.
- Real docked captures:
  - `pixi run py-demo-capture -- --scene floating_base --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_floating_base_metrics_1781250001`
    wrote 71 PNG frames, 72 scene-metrics events, and latest metrics with row
    `floating_base`, related source row `rigid_free_flight`, linear speed about
    `1.0145`, angular speed `2.0`, body x about `0.725`, and world time about
    `0.72`.
  - `pixi run py-demo-capture -- --scene articulated --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_articulated_metrics_1781250002`
    wrote 71 PNG frames, 72 scene-metrics events, and latest metrics with row
    `articulated`, related source row `rigid_multibody_dynamics_terms`,
    shoulder speed about `0.643`, wrist speed about `0.583`, max joint speed
    about `0.643`, forearm height about `-0.0257`, and world time about
    `0.072`.
- `pixi run lint` passed.
- Bounded default `pixi run build` passed with `DART safe jobs: 4` and
  `ninja: no work to do`.
- `git diff --check` passed.

## Critical Stop Handoff - 2026-06-11 AVBD Related Metrics

The user explicitly stopped further work and requested handoff-only docs with
no further verification. Treat this section as the fresh-session entry point.

State at the start of this handoff-only docs edit:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Worktree: clean before the docs handoff edit.
- Local code checkpoint: `4d63d2b24b0c`.
- Local code checkpoint title: `Expose AVBD related route capture metrics`.
- Origin at that moment: `5f794e65d3f8`
  (`Document rigid visual verification handoff`).
- The local branch was ahead of origin by one code/docs checkpoint before this
  handoff-only docs commit.
- Last checked GitHub state: no PR associated with this branch.

No tests, lint, build, captures, or `git diff --check` were run after this
handoff-only docs edit. Do not treat the handoff docs commit itself as freshly
verified.

## Current Checkpoint Snapshot - 2026-06-11 AVBD Related Metrics

The continuation resumed from pushed handoff commit `5f794e65d3f` and finished
the AVBD related-route metrics slice that had been partially edited in the
dirty worktree. These remain non-numbered related shelf routes, not new World
Rigid Body workflow rows.

Current local slice: AVBD related-route capture metrics. The checkpoint touches:

- `python/examples/demos/scenes/avbd_rigid_fixed_joint_contact.py`
- `python/examples/demos/scenes/avbd_rigid_spherical_breakable_joint.py`
- `python/examples/demos/scenes/avbd_rigid_revolute_motor.py`
- `python/examples/demos/scenes/avbd_rigid_prismatic_motor.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test:

- `avbd_rigid_fixed_joint_contact.py` now samples fixed-joint contact metrics
  into shared histories during `pre_step` and exposes them through
  `SceneSetup.info["capture_metrics"]`: captured-offset error,
  payload-ground clearance, payload speed, contact count, base x, payload x,
  and related source row `contact`.
- `avbd_rigid_spherical_breakable_joint.py` now samples spherical breakage
  lifecycle metrics: anchor-offset error, orientation drift, payload speed,
  payload height, broken state, break force, status, and related source row
  `rigid_joint_breakage`.
- `avbd_rigid_revolute_motor.py` now samples revolute velocity motor metrics:
  measured speed, target-speed error, absolute speed error, world time, and
  related source row `rigid_joint_motor_limits`.
- `avbd_rigid_prismatic_motor.py` now samples prismatic velocity motor metrics:
  measured speed, speed error, axis position, orthogonal drift, world time, and
  related source row `rigid_joint_motor_limits`.
- The four existing AVBD integration tests now assert the new capture hooks,
  related-source identities, compact histories, and finite manifest-friendly
  top-level fields.

Validation collected before the stop request for this slice:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_avbd_fixed_joint_contact_demo_exercises_contact_path python/tests/integration/test_demos_cycle.py::test_avbd_revolute_motor_demo_drives_hinge python/tests/integration/test_demos_cycle.py::test_avbd_prismatic_motor_demo_drives_slider python/tests/integration/test_demos_cycle.py::test_avbd_rigid_spherical_breakable_joint_demo_resets_anchor_only -q`
  reported `4 passed`.
- Real docked captures:
  - `pixi run py-demo-capture -- --scene avbd_rigid_fixed_joint_contact --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_avbd_fixed_joint_contact_metrics_1781242847`
    wrote 71 PNG frames, 72 scene-metrics events, and latest metrics with
    contact count `3`, max contact count `4`, captured-offset error about
    `0.05649`, and payload speed about `0.1239`.
  - `pixi run py-demo-capture -- --scene avbd_rigid_spherical_breakable_joint --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_avbd_spherical_breakable_metrics_1781242878`
    wrote 71 PNG frames, 72 scene-metrics events, and latest metrics with
    status `broken`, `saw_broken=1`, anchor-offset error about `0.4608`,
    orientation drift about `2.3637`, and payload speed about `2.923`.
  - `pixi run py-demo-capture -- --scene avbd_rigid_revolute_motor --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_avbd_revolute_motor_metrics_1781242920`
    wrote 71 PNG frames, 72 scene-metrics events, and latest metrics with
    measured speed about `1.2` rad/s and near-zero speed error.
  - `pixi run py-demo-capture -- --scene avbd_rigid_prismatic_motor --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_avbd_prismatic_motor_metrics_1781242958`
    wrote 71 PNG frames, 72 scene-metrics events, and latest metrics with
    measured speed about `0.8` m/s, axis position about `0.708`, and zero
    orthogonal drift.
- The focused AVBD plus related-route/docs guard with panel related-route
  coverage reported `15 passed`.
- `pixi run lint` passed.
- Bounded default `pixi run build` passed and reported `ninja: no work to do`.
- `git diff --check` passed.

## Critical Stop Handoff - 2026-06-11

The user explicitly stopped the continuation and requested handoff only for all
current work with no further verification. This is historical context; the
current fresh-session entry point is the top AVBD related-metrics section.

State at the start of this handoff edit:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Worktree: clean.
- Local `HEAD`: `949d08083a8` (`Expose contact-gradient route capture
metrics`).
- Origin: `origin/feature/rigid-body-gui-visual-verification` still pointed at
  `377e55ce5cb` (`Document rigid visual verification stop handoff`).
- Local branch was ahead of origin by one code/docs checkpoint before this
  handoff-only docs commit.
- Last checked GitHub state: no PR associated with this branch.

What is complete in the local checkpoint:

- `diff_drone_liftoff` now has scene-owned capture metrics for the
  non-numbered Differentiable contact-gradient route linked from
  `rigid_contact_solver_compare`.
- The checkpoint includes code, test, CHANGELOG, Python demo README, PLAN-103
  sidecar, dev-task README, this resume file, and PR draft updates.
- Validation for that checkpoint was already collected before the stop request:
  focused contact-gradient metrics test `1 passed`; docked default-build
  capture under `/tmp/dart_capture_diff_drone_liftoff_metrics_1781241455`
  wrote 95 PNG frames, 96 scene-metrics events, and a nonblank screenshot;
  related route/docs drift guard `17 passed`; `pixi run lint` passed; bounded
  default `pixi run build` passed with `ninja: no work to do`; and
  `git diff --check` passed.
- The default capture reported status `fallback` and `optimized=false` because
  the default build has `DART_BUILD_DIFF=OFF`; that was expected by the test.

No tests, lint, build, captures, or `git diff --check` were run after this
handoff-only docs edit. Do not treat the handoff docs commit itself as freshly
verified.

What was interrupted:

- The next continuation had only inspected and planned the next bounded slice.
  It did not successfully edit code.
- A broad `apply_patch` attempt for the next slice failed due to context
  mismatch. No partial changes were applied by that failed patch.
- The next candidate slice was: add scene-owned capture metrics to the missing
  AVBD related-evidence routes while keeping them non-numbered:
  `avbd_rigid_fixed_joint_contact`,
  `avbd_rigid_spherical_breakable_joint`,
  `avbd_rigid_revolute_motor`, and `avbd_rigid_prismatic_motor`.
- `avbd_rigid_breakable_joint` already has capture metrics and should be used
  as the local pattern.

Relevant related-evidence mapping:

- `rigid_free_flight` links to `floating_base`.
- `rigid_multibody_dynamics_terms` links to `articulated`.
- `rigid_solver_compare` links to `rigid_ipc_tunnel`, which now has capture
  metrics.
- `rigid_contact_solver_compare` links to `diff_drone_liftoff`, which now has
  capture metrics.
- `contact` links to `avbd_rigid_fixed_joint_contact`, which still lacks
  capture metrics.
- `rigid_joint_breakage` links to `avbd_rigid_breakable_joint`, which already
  has capture metrics, and `avbd_rigid_spherical_breakable_joint`, which still
  lacks capture metrics.
- `rigid_joint_motor_limits` links to `avbd_rigid_revolute_motor` and
  `avbd_rigid_prismatic_motor`, which still lack capture metrics.

Recommended resume path:

1. Confirm branch state with `git status -sb` and
   `git log --oneline --decorate -5`.
2. Read `avbd_rigid_breakable_joint.py` as the capture-metrics pattern before
   editing the four missing AVBD scenes.
3. Add `CAPTURE_METRICS_INFO_KEY` hooks to the four missing AVBD related
   routes, keeping payloads summary-oriented: row id, related source row,
   solver/scope or actuator label, current metrics, compact history extrema,
   and manifest-friendly top-level numeric fields.
4. Extend the existing focused AVBD integration tests in
   `python/tests/integration/test_demos_cycle.py` rather than adding broad new
   test surfaces.
5. Update `CHANGELOG.md`, `python/examples/demos/README.md`, PLAN-103, this
   task README, this resume, and `PR_DRAFT.md` after code changes.
6. Run focused tests/captures/lint/build only after the user explicitly resumes
   implementation or asks for verification.

Scene details gathered before the stop:

- `avbd_rigid_fixed_joint_contact.py` has fixed-joint base/payload/ground
  contact, connector sync, and panel-only histories for captured-offset error,
  payload-ground clearance, and payload speed. Suggested payload fields:
  `row`, `solver=avbd_rigid_joints`, `constraint=fixed_joint_contact_path`,
  `related_source_row=contact`, fixed-joint count, contact count, offset error,
  clearance, speed, base/payload x positions, and history extrema.
- `avbd_rigid_spherical_breakable_joint.py` has a weak spherical break-force
  joint, reset/rearm helpers, connector color changes, captured offset and
  captured rotation, and panel-only histories for anchor error, orientation
  drift, and broken state. Suggested payload fields: `row`,
  `constraint=spherical_break_force_anchor_lifecycle`,
  `related_source_row=rigid_joint_breakage`, break/reset forces, anchor error,
  orientation drift, payload speed/height, broken/status, and history extrema.
- `avbd_rigid_revolute_motor.py` has a zero-gravity revolute velocity motor
  with target speed `1.2` and max torque `800.0`, plus panel-only speed/error
  histories. Suggested payload fields: `row`,
  `actuator=revolute_velocity_motor`,
  `related_source_row=rigid_joint_motor_limits`, target speed, max torque,
  measured speed, speed error, absolute speed error, and history extrema.
- `avbd_rigid_prismatic_motor.py` has a zero-gravity prismatic velocity motor
  with target speed `0.8` and max force `800.0`, plus panel-only speed,
  position, and orthogonal-drift histories. Suggested payload fields: `row`,
  `actuator=prismatic_velocity_motor`,
  `related_source_row=rigid_joint_motor_limits`, target speed, max force,
  measured speed, speed error, axis position, orthogonal drift, and history
  extrema.

Future pushes, PR creation, comments, review replies, CI retriggers, merges, or
other GitHub mutations require fresh explicit maintainer/user approval unless
the user has explicitly requested that exact mutation in the active turn.

## Current Handoff Snapshot - 2026-06-11 Continuation

The post-stop continuation resumed from pushed handoff commit `377e55ce5cb`
and implemented the remaining non-numbered related-shelf metrics candidate:
`diff_drone_liftoff`. This is a Differentiable shelf route from
`rigid_contact_solver_compare`, not a new numbered World Rigid Body row.

Current local slice: `diff_drone_liftoff` related-shelf capture metrics. The
checkpoint touches:

- `python/examples/demos/scenes/diff_drone_liftoff.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test:

- `diff_drone_liftoff.py` now imports `CAPTURE_METRICS_INFO_KEY` and registers
  `SceneSetup.info["capture_metrics"]`.
- The payload exports row identity, category, related source row, boxed-LCP
  contact-gradient scope, optimized/fallback status, gradient modes, time step,
  horizon, iteration/learning-rate constants, target/rest height,
  playhead/current height, analytic and complementarity-aware
  thrust/final-height/loss values, height/target-error/thrust gaps, compact
  history summaries, and top-level numeric fields for manifest ranges.
- `python/tests/integration/test_demos_cycle.py::test_diff_drone_liftoff_reports_contact_gradient_metrics`
  asserts the hook is callable, mirrors scene state, exports finite
  manifest-friendly fields, verifies aware-mode escape when diff bindings are
  enabled, and accepts the explicit fallback payload when the default build has
  `DART_BUILD_DIFF=OFF`.

Validation collected so far for this slice:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_diff_drone_liftoff_reports_contact_gradient_metrics -q`
  reported `1 passed`.
- `pixi run py-demo-capture -- --scene diff_drone_liftoff --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_diff_drone_liftoff_metrics_1781241455`
  wrote a nonblank docked capture with 95 PNG frames, 96 scene-metrics events,
  and screenshot
  `/tmp/dart_capture_diff_drone_liftoff_metrics_1781241455/diff_drone_liftoff.png`.
- Latest captured payload details from the default build: row
  `diff_drone_liftoff`, solver `boxed_lcp_contact_gradient_modes`, scope
  `rigid_contact_gradient_saddle_escape`, status `fallback`, `optimized=false`,
  target height `1.5`, rest/current/final heights about `0.19995`, target error
  `1.30005`, zero height/thrust gaps, horizon `150`, max iterations `400`,
  learning rate `4.0`, and top-level numeric ranges for the manifest. The
  fallback status is expected because the default capture was configured with
  `DART_BUILD_DIFF=OFF`.
- The focused related-route/docs guard with row ordering, viewer-title
  numbering, sidecar/README/capture-command drift checks,
  no-tunneling/stack-packet metrics, contact-gradient metrics, and panel
  related-route coverage reported `17 passed`.
- `pixi run lint` passed.
- Bounded default `pixi run build` passed and reported `ninja: no work to do`.
- `git diff --check` passed.

Future pushes, PR creation, comments, review replies, CI retriggers, merges, or
other GitHub mutations require fresh explicit maintainer/user approval.

## Historical Critical Stop Handoff - 2026-06-11

This section records the prior handoff-only stop before the continuation above.
The user explicitly requested a handoff-only stop for all current work with no
further verification.

State at the start of this handoff edit:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Worktree: clean.
- Local `HEAD`: `5f5445b6826` (`Expose rigid IPC tunnel capture metrics`).
- Origin: `origin/feature/rigid-body-gui-visual-verification` still pointed at
  `c69ee66ace9` (`Hand off rigid loop closure metrics`).
- Local branch was ahead of origin by one code/docs checkpoint before this
  handoff-only docs commit.
- There was still no associated GitHub PR in the last checked state.

What is complete in the local checkpoint:

- `rigid_ipc_tunnel` has scene-owned capture metrics for the non-numbered
  focused Rigid IPC no-tunneling route.
- The checkpoint includes code, test, CHANGELOG, Python demo README,
  PLAN-103 sidecar, dev-task README, this resume file, and PR draft updates.
- Validation for that checkpoint was already collected before the stop request:
  focused no-tunneling metrics test `1 passed`, docked 24-frame capture with
  24 scene-metrics events under
  `/tmp/dart_capture_rigid_ipc_tunnel_metrics_1781240644`, related route/docs
  drift guard `12 passed`, `pixi run lint` passed, bounded `pixi run build`
  passed with `ninja: no work to do`, and `git diff --check` passed.

What was interrupted at that time:

- The next continuation only inspected the remaining related-shelf gap. It did
  not edit code.
- The likely next candidate was `diff_drone_liftoff`, reached from
  `rigid_contact_solver_compare` as the Differentiable shelf contact-gradient
  route.
- Keep this route non-numbered. Do not add another numbered rigid row unless a
  distinct user question appears beyond the existing contact-solver comparison.

Context gathered for the interrupted `diff_drone_liftoff` audit:

- File: `python/examples/demos/scenes/diff_drone_liftoff.py`.
- The scene compares `ANALYTIC` and `COMPLEMENTARITY_AWARE` optimization modes
  for a clamping-contact saddle-escape example.
- Constants observed during the audit include `_TIME_STEP=1e-2`,
  `_HORIZON=150`, `_MAX_ITERS=400`, `_LEARNING_RATE=4.0`,
  `_DRONE_RADIUS=0.2`, `_DRONE_MASS=1.0`, `_TARGET_Z=1.5`,
  `_REST_Z=0.19995`, `_THRUST_COLUMN=2`, and `_Z_STATE_ROW=2`.
- Current `build()` info already exposes `optimized`, `steps`, `note`,
  `gradient_modes`, `target_z`, `naive_thrust`, `naive_height`,
  `aware_thrust`, `aware_height`, `aware_history_count`, and
  `naive_history_count`.
- The panel already exposes mode, optimized status, frame/current z, target z,
  playback stride/reset, analytic thrust/final z/loss, aware thrust/final
  z/loss, and compact plots for aware height/thrust/loss/thrust gradient plus
  analytic loss.
- No `CAPTURE_METRICS_INFO_KEY` hook exists in that scene yet.

The continuation used this guidance for the contact-gradient metrics payload:
keep it summary-oriented with row id, scope, status, time step, horizon,
learning rate, target/rest height, playhead/current height, ANALYTIC versus
COMPLEMENTARITY_AWARE thrust/final-height/loss values, height gap, target
error, thrust gap, compact history sample counts/ranges, and finite top-level
numeric fields for manifest summaries.

No tests, lint, build, captures, or `git diff --check` were run after that
handoff-only docs edit. The continuation above supersedes the old "not
implemented" state.

## Current Handoff Snapshot

Latest continuation resumed from pushed handoff commit `c69ee66ace9`, audited
the next non-numbered related-shelf gap, and added capture metrics to the
focused Rigid IPC no-tunneling route `rigid_ipc_tunnel`. This is the first
post-numbered-workflow shelf hardening slice, not a new numbered row. The branch
had no associated GitHub PR at resume time.

Expected branch/worktree state after this local checkpoint:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Origin and local `HEAD` were `c69ee66ace9` before this local continuation.
- The current local checkpoint should include `rigid_ipc_tunnel` capture
  metrics, focused assertions, docs, docked capture evidence, and this refreshed
  handoff state.
- Future pushes, PR creation, comments, review replies, CI retriggers, merges,
  or other GitHub mutations require fresh explicit maintainer/user approval.

Recent checkpoints:

- `7ef7a96dda0` (`Expose rigid collision cast capture metrics`)
- `b54c4f5dba3` (`Expose rigid executor equivalence capture metrics`)
- `a47d52b7aea` (`Hand off rigid friction threshold capture metrics`)
- `d16c6200850` (`Complete rigid friction threshold capture docs`)
- `c2beab542e9` (`Expose rigid spin roll capture metrics`)
- `dfef5306aac` (`Expose rigid stack stability capture metrics`)
- `b68f216ebe9` (`Expose rigid contact manipulation capture metrics`)
- `9947ab1285a` (`Expose rigid kinematic driver capture metrics`)
- `30d65d4229c` (`Document rigid visual verification handoff`)
- `8f2119d7cf1` (`Expose rigid joint motor limit capture metrics`)
- `544d2b44a62` (`Hand off rigid passive joint parameter metrics`)
- `44354ed24c7` (`Expose rigid screw joint pitch capture metrics`)
- `9ea310b2c8b` (`Document rigid visual verification stop handoff`)
- `bbf8a8006b7` (`Expose rigid multibody dynamics term capture metrics`)
- `7452a0f22d4` (`Hand off rigid link center of mass metrics`)
- `3c6d5fee5a6` (`Complete rigid link center of mass capture docs`)
- `3864dbf331a` (`Hand off rigid link Jacobian metrics`)
- `d0532eeb560` (`Expose rigid multibody solver family capture metrics`)
- `c69ee66ace9` (`Hand off rigid loop closure metrics`)
- the local checkpoint containing this file

Current local slice: `rigid_ipc_tunnel` related-shelf capture metrics. The checkpoint
touches:

- `python/examples/demos/scenes/rigid_ipc_tunnel.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test:

- `python/examples/demos/scenes/rigid_ipc_tunnel.py` now imports
  `CAPTURE_METRICS_INFO_KEY` and registers
  `SceneSetup.info["capture_metrics"]`.
- The scene now wraps `bridge.pre_step()` so it samples no-tunneling metrics on
  each viewer step: wall clearance, through-wall margin, leading/trailing face
  positions, box velocity/speed, contact count, step timing, barrier status,
  and history extrema.
- The payload exports row identity, solver `rigid_ipc`, scope
  `focused_no_tunneling_capability`, time step, world time, launch speed,
  wall/box extents, current and min clearance, min tunnel margin, max wall
  crossing, min box velocity, max contact count, max step timing, and sample
  count.
- `python/tests/integration/test_demos_cycle.py::test_rigid_ipc_tunnel_reports_no_tunneling_metrics`
  now asserts the capture hook mirrors live IPC state, keeps positive
  through-wall margin, and exposes finite manifest-friendly top-level fields.

Validation status for the no-tunneling related-shelf checkpoint:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_tunnel_reports_no_tunneling_metrics -q`
  reported `1 passed`.
- `pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_tunnel_metrics_1781240644`
  wrote a nonblank docked capture with 23 PNG frames, 24 scene-metrics events,
  screenshot
  `/tmp/dart_capture_rigid_ipc_tunnel_metrics_1781240644/rigid_ipc_tunnel.png`,
  and final contacts `0`.
- Latest captured payload details: row `rigid_ipc_tunnel`, solver `rigid_ipc`,
  scope `focused_no_tunneling_capability`, status `barrier-held`, launch speed
  `8.0`, time step `10.0` ms, world time `0.24`, latest clearance about
  `1.23e-6` m, minimum clearance about `1.22e-6` m, minimum tunnel margin about
  `0.500001` m, max wall crossing `0.0`, box-vx range about
  `[-1.01e-6, 8.0]`, 24 event payloads, and finite manifest ranges for
  clearance, tunnel margin, box position/velocity, wall faces, step timing, and
  world time.
- The focused route/docs drift guard with row ordering, viewer-title numbering,
  docs-count, sidecar, related-evidence route/capture-command, README,
  capture-command, stack-packet, tunnel-metrics, and high-value panel coverage
  reported `12 passed`.
- `pixi run lint` passed.
- Bounded
  `DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 pixi run build`
  passed and reported `ninja: no work to do`.
- `git diff --check` passed.

Completed local checkpoint immediately before this slice:

- Commit `c69ee66ace9` (`Hand off rigid loop closure metrics`) completes the
  row-36 loop-closure handoff and pushes row-35 plus row-36 checkpoint state to
  origin.
- That row publishes POINT/DISTANCE/RIGID loop-closure capture metrics for
  residual-only versus solved policies, residuals, tip/distance/orientation
  errors, residual ratios, joint speed, and step timing. Focused test and real
  docked capture evidence were recorded; post-stop lint/diff refresh was
  intentionally skipped in that handoff because the user explicitly requested
  no further verification.

- Commit `d0532eeb560` (`Expose rigid multibody solver family capture metrics`)
  completes row 35, `rigid_multibody_solver_family`.
- That row publishes solver-family capture metrics for executor/gravity
  controls, case order, integration family and closure policy labels,
  residuals, tip errors/heights, joint speeds, step timing, residual solve
  ratio, and compact histories.
- Validation collected for row 35: focused test reported `1 passed`; real
  docked capture under
  `/tmp/dart_capture_multibody_solver_family_metrics_1781239036` wrote 71 PNG
  frames and 72 scene-metrics events; broader workflow/doc drift guard reported
  `15 passed`; `pixi run lint`, bounded default `pixi run build`, and
  `git diff --check` passed.

Previous completed checkpoint:

- Commit `3864dbf331a` (`Hand off rigid link Jacobian metrics`) completes row
  34, `rigid_link_jacobian`.
- That row publishes link-origin Jacobian capture metrics for motion/wrench
  controls, joint names, link-origin position, twist components,
  finite-difference error, wrench force, transpose-mapped torques,
  joint-versus-wrench power, world/body Jacobian gap, and compact histories.
- Validation collected for row 34: focused test reported `1 passed`; real
  docked capture under `/tmp/dart_capture_link_jacobian_metrics_1781238268`
  wrote 95 PNG frames and 96 scene-metrics events; broader workflow/doc drift
  guard reported `14 passed`. The skipped stop-state gates were refreshed from
  the clean pushed checkpoint: `pixi run lint`, bounded default
  `pixi run build`, and `git diff --check` passed.

- Commit `3c6d5fee5a6` (`Complete rigid link center of mass capture docs`)
  completes row 33, `rigid_link_center_of_mass`.
- That row publishes inertial-offset capture metrics for COM/gravity/mass/
  inertia controls, lane order/count, link/joint/local-COM metadata, serialized
  lane metrics, mirrored torque/angle/acceleration sums, reflected mass-matrix
  and acceleration ratios, COM marker coordinates, energy, step timing, and
  compact history fields.
- Validation already collected for row 33: focused test reported `1 passed`;
  real docked capture under
  `/tmp/dart_capture_link_center_of_mass_metrics_1781237623` wrote 71 PNG
  frames and 72 scene-metrics events; broader workflow/doc drift guard reported
  `14 passed`; `pixi run lint`, bounded default `pixi run build`, and
  `git diff --check` passed.

- Commit `bbf8a8006b7` (`Expose rigid multibody dynamics term capture metrics`)
  completes row 32, `rigid_multibody_dynamics_terms`.
- That row publishes joint-space dynamics capture metrics for controls, lane
  order/count, joint names, target/impulse patterns, mass/inverse-mass,
  coupling, conditioning, inverse-dynamics residuals, impulse residuals, torque
  norm, response norm, heavy-versus-coupled ratios, step timing, and compact
  history fields.
- Validation already collected for row 32: focused test reported `1 passed`;
  real docked capture under
  `/tmp/dart_capture_multibody_dynamics_terms_metrics_1781236627` wrote 95 PNG
  frames and 96 scene-metrics events; broader workflow/doc drift guard reported
  `13 passed`; `pixi run lint`, bounded default `pixi run build`, and
  `git diff --check` passed.

Previous local checkpoint: `rigid_joint_motor_limits` row capture metrics. The
checkpoint touches:

- `python/examples/demos/scenes/rigid_joint_motor_limits.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test:

- `CAPTURE_METRICS_INFO_KEY` wiring in
  `python/examples/demos/scenes/rigid_joint_motor_limits.py`.
- Scene-owned payload fields for row identity, solver
  `world_multibody_joint_actuators`, constraint
  `velocity_motor_position_limit_effort_cap`, time step, world time,
  user-facing controls, joint names, motor speed/position clamp metrics,
  position-limit angle/error/speed, capped-vs-reference force travel and
  acceleration gaps, step timing, the latest metrics snapshot, and compact
  history ranges.
- `_step_ms_history` replay capture/restore coverage and a panel line showing
  force travel gap, step timing, and world time.
- Focused assertions in
  `python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort`
  that the capture hook mirrors the controller state and exposes history maxima.

Validation collected for the motor-limit slice:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort -q`
  reported `1 passed` after fixing the assertions to compare latest top-level
  fields against latest metrics and history fields against max histories.
- `pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_motor_limits_metrics_1781234533`
  wrote a nonblank docked capture with 71 PNG frames, 72 scene-metrics events,
  screenshot
  `/tmp/dart_capture_joint_motor_limits_metrics_1781234533/rigid_joint_motor_limits.png`,
  and manifest ranges for motor speed/position/error, position-limit angle and
  error, capped/open force acceleration, acceleration gap, force-position gap,
  step timing, time step, and world time.
- Latest captured payload details: row `rigid_joint_motor_limits`, solver
  `world_multibody_joint_actuators`, constraint
  `velocity_motor_position_limit_effort_cap`, command speed `0.55`, velocity
  limit `0.3`, position limit `0.35`, requested force `16.0`, effort limit
  `4.0`, motor speed `0.3`, motor speed error `0`, latest position-limit angle
  about `0.2633`, latest force-position gap about `0.3942`, latest
  limited/open force acceleration about `2.0/8.0`, acceleration gap about `6.0`,
  latest step time about `0.0720 ms`, and history sample count `73`.
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  reported `11 passed`.
- `pixi run lint` passed.
- Bounded `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run build`
  passed and reported `ninja: no work to do`.
- `git diff --check` passed.

Previous local checkpoint: `rigid_joint_passive_parameters` row capture metrics. The
handoff checkpoint touches:

- `python/examples/demos/scenes/rigid_joint_passive_parameters.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test:

- `CAPTURE_METRICS_INFO_KEY` wiring in
  `python/examples/demos/scenes/rigid_joint_passive_parameters.py`.
- Scene-owned payload fields for row identity, solver
  `world_multibody_passive_joint_parameters`, scope
  `contact_free_prismatic_lanes`, executor, time step, world time, controls,
  lane order/count, serialized lane metrics, latest spring/damped energy,
  damping ratio, stiction/slip position/speed/acceleration/error, armature
  acceleration and position gaps, step timing, flattened per-lane metric
  fields, and compact history maxima.
- Focused assertions in
  `python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response`
  that the capture hook mirrors controller controls, lane metadata, latest
  metrics, and history maxima.

Validation collected for the passive-parameter slice before the stop request:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q`
  reported `1 passed`.
- `pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_passive_parameters_metrics_1781235045`
  wrote a nonblank docked capture with 119 PNG frames, 120 scene-metrics
  events, and screenshot
  `/tmp/dart_capture_joint_passive_parameters_metrics_1781235045/rigid_joint_passive_parameters.png`.
- Latest captured payload details: row `rigid_joint_passive_parameters`,
  solver `world_multibody_passive_joint_parameters`, scope
  `contact_free_prismatic_lanes`, executor `Sequential`, lane count `6`,
  latest spring energy about `2.1583`, damped energy about `1.3403`,
  damped-energy ratio about `0.6210`, stiction position `0`, slip position
  about `0.17424`, slip speed about `0.72`, armature reference/heavy
  acceleration about `3.0/0.75`, armature acceleration gap about `2.25`,
  armature position gap about `0.26136`, latest step time about `0.0534 ms`,
  history samples `121`, and max observed step time about `0.1003 ms`.
- The skipped post-handoff gates were run at the start of the screw-pitch
  continuation: the broader workflow/doc drift guard reported `11 passed`,
  `pixi run lint` passed, bounded
  `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run build`
  passed with `ninja: no work to do`, and `git diff --check` passed.

Current local slice: `rigid_screw_joint_pitch` row capture metrics. The local
checkpoint touches:

- `python/examples/demos/scenes/rigid_screw_joint_pitch.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test:

- `CAPTURE_METRICS_INFO_KEY` wiring in
  `python/examples/demos/scenes/rigid_screw_joint_pitch.py`.
- Scene-owned payload fields for row identity, solver
  `world_multibody_screw_joint_pitch`, scope
  `contact_free_screw_pitch_lanes`, executor, time step, world time, controls,
  lane order/count, serialized lane metrics, pitch ratios, zero/fine/coarse/
  reverse pitch angle and axial travel, acceleration residuals, effective mass,
  mass matrix, step timing, and compact history maxima.
- Focused assertions in
  `python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation`
  that the capture hook mirrors controller controls, lane metadata, latest
  metrics, and residual/history fields.

Validation collected for the screw-pitch slice so far:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation -q`
  reported `1 passed`.
- `pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_screw_joint_pitch_metrics_1781235714`
  wrote a nonblank docked capture with 95 PNG frames, 96 scene-metrics events,
  screenshot
  `/tmp/dart_capture_screw_joint_pitch_metrics_1781235714/rigid_screw_joint_pitch.png`,
  and final contacts `0`.
- Latest captured payload details: row `rigid_screw_joint_pitch`, solver
  `world_multibody_screw_joint_pitch`, scope
  `contact_free_screw_pitch_lanes`, executor `Sequential`, lane count `4`,
  fine/coarse/reverse pitch `0.28/0.56/-0.28`, coarse/fine pitch ratio `2`,
  reverse/fine pitch ratio `-1`, latest fine/coarse/reverse angle about
  `-0.8317/-0.6162/0.8317`, latest fine/coarse/reverse axial travel about
  `-0.2329/-0.3451/-0.2329`, near-zero acceleration residuals on all nonzero
  pitch lanes, latest step time about `0.0705 ms`, history samples `97`, and
  max observed step time about `0.1055 ms`.
- The broader workflow/doc drift guard with row ordering, viewer-title
  numbering, sidecar/README/capture-command drift checks, motor-limit coverage,
  passive-parameter coverage, replay snapshot coverage, and high-value panel
  coverage reported `12 passed`.
- `pixi run lint` passed.
- Bounded `DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run build`
  passed and reported `ninja: no work to do`.
- `git diff --check` passed.

Current local slice: `rigid_multibody_dynamics_terms` row capture metrics. The
local checkpoint touches:

- `python/examples/demos/scenes/rigid_multibody_dynamics_terms.py`
- `python/tests/integration/test_demos_cycle.py`
- `CHANGELOG.md`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`

What changed in code/test:

- `CAPTURE_METRICS_INFO_KEY` wiring in
  `python/examples/demos/scenes/rigid_multibody_dynamics_terms.py`.
- Scene-owned payload fields for row identity, solver
  `world_multibody_dynamics_terms`, scope
  `contact_free_joint_space_dynamics`, executor, time step, world time,
  top-level and nested controls, lane order/count, joint names,
  target/impulse patterns, per-lane mass/inverse-mass, coupling, conditioning,
  residual, torque, response, heavy-versus-coupled ratios, step timing, and
  compact history maxima.
- Focused assertions in
  `python/tests/integration/test_demos_cycle.py::test_rigid_multibody_dynamics_terms_expose_generalized_terms`
  that the capture hook mirrors controller controls, lane metadata, latest
  metrics, ratio fields, and history maxima.

Validation collected for the dynamics-terms slice so far:

- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_dynamics_terms_expose_generalized_terms -q`
  reported `1 passed`.
- `pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_dynamics_terms_metrics_1781236627`
  wrote a nonblank docked capture with 95 PNG frames, 96 scene-metrics events,
  screenshot
  `/tmp/dart_capture_multibody_dynamics_terms_metrics_1781236627/rigid_multibody_dynamics_terms.png`,
  and final contacts `0`.
- Latest captured payload details: row `rigid_multibody_dynamics_terms`,
  solver `world_multibody_dynamics_terms`, scope
  `contact_free_joint_space_dynamics`, executor `Sequential`, lane count `3`,
  target acceleration `2.2`, joint impulse `3.0`, heavy distal mass scale `4.0`,
  gravity scale `1.0`, latest coupled/heavy coupling about `0.3566/1.4265`,
  latest coupled/heavy response norm about `15.4636/8.6328`, latest
  coupled/heavy tau norm about `8.8632/27.0012`, heavy-to-coupled tau ratio
  about `3.0464`, heavy-to-coupled response ratio about `0.5583`, near-zero
  residuals, latest step time about `0.0574 ms`, history samples `97`, and max
  observed step time about `0.1144 ms`.
- The broader workflow/doc drift guard with row ordering, viewer-title
  numbering, sidecar/README/capture-command drift checks, motor-limit coverage,
  passive-parameter coverage, screw-pitch coverage, replay snapshot coverage,
  and high-value panel coverage reported `13 passed`.
- `pixi run lint` passed.
- Bounded
  `DART_PARALLEL_JOBS=2 CTEST_PARALLEL_LEVEL=2 CMAKE_BUILD_PARALLEL_LEVEL=2 pixi run build`
  passed and reported `ninja: no work to do`.
- `git diff --check` passed.

Immediate next step for a fresh session:

1. Verify `git status -sb` and `git log --oneline --decorate -5`.
2. If the no-tunneling local checkpoint has not been committed yet, commit it
   after lint/focused gates are recorded.
3. Continue the non-numbered shelf audit only where a related route adds
   distinct user evidence beyond the numbered row; do not add another numbered
   row by default.
4. Push, PR creation, comments, review replies, CI retriggers, merges, or other
   GitHub mutations still require explicit maintainer/user approval.

## Last Session Summary

Created the first paired rigid solver comparison scene for Python `py-demos`.
`rigid_solver_compare` runs sequential impulse and rigid IPC in matched DART 7
Worlds, renders them side by side, exposes an executor selector, and plots
speed, wall clearance, position divergence, and step profile timing.

Focused Python demo tests pass, and `py-demo-capture` now applies the same
Linux software-Mesa defaults as GUI tests. The verified visual smoke command is:

```bash
pixi run py-demo-capture -- --scene rigid_solver_compare --frames 24 \
    --width 960 --height 540 --show-ui
```

The second P0 slice, `rigid_friction_threshold`, is implemented and visually
captured. It shows three rigid IPC inclined-ramp lanes below, near, and above
`mu = tan(angle)`.

The executor-equivalence slice, `rigid_executor_equivalence`, is implemented and
visually captured. It runs matched DART 7 Worlds side by side with one shared
physics solver while the left world uses the sequential executor and the right
world uses a parallel executor. Its panel plots pose divergence, velocity
divergence, contact-count delta, and per-executor step timing so users can
distinguish executor performance from physics changes.

The contact-solver policy slice, `rigid_contact_solver_compare`, is implemented
and visually captured. It keeps the rigid-body solver family fixed to
sequential impulse, then compares the default sequential-impulse contact policy
against the opt-in boxed-LCP contact method on the same tilted multi-contact
plank. Its panel plots contact count, penetration depth, analytic corner
clearance, speed, angular speed, kinetic energy, pose divergence, and per-step
timing. The shared replay timeline now also shows this pose-divergence signal
and marks frames with coupled contacts or positive depth while scrubbing saved
states.

The restitution material-response slice, `rigid_restitution_ladder`, is
implemented and visually captured. It drops three matched rigid spheres with
low, medium, and high restitution, exposes solver/executor, launch height, and
restitution scale, and plots height, vertical velocity, contact count, rebound
height, mechanical-energy trend, and step timing without claiming exact energy
conservation.

The pair-material ownership slice, `rigid_material_mixing`, is implemented
immediately after `rigid_restitution_ladder`. It pins the row to sequential
impulse and uses swapped body/surface lanes: two bounce lanes for
`max(restitution)` and two flat sliding lanes for
`sqrt(friction_a * friction_b)`. The row computes expected pair coefficients
from public `RigidBody.restitution` and `RigidBody.friction`, and it avoids IPC
restitution claims or duplicating the incline stick/slip threshold row.

The third P0 slice, `rigid_stack_stability`, is implemented and visually
captured. It is a compact two-block top-heavy stack with sequential impulse and
rigid IPC side by side. A local probe showed taller IPC stacks are too slow for
the default GUI, so this scene uses a deliberately small stack to expose
mass-ratio jitter/drift without turning the live demo into a benchmark-only
surface.

The contact-observability slice, `rigid_contact_inspector`, is implemented and
visually captured. It uses static targets plus kinematic probes to keep stable
contacts available for direct `World.collide()` inspection: selected/total
contact count, representative point, normal, depth, local points, shape
indices, and depth histories. The inspector now covers representative public
shape families in one row: sphere/box, box/ground, capsule/sphere,
cylinder/sphere, mesh/sphere, and compound/sphere, including compound child
shape-index diagnostics.

The collision-query-options slice, `rigid_collision_query_options`, is
implemented and visually captured. It keeps four stable sphere overlap lanes
visible for direct `World.collide(options)` inspection: rigid/rigid,
rigid/link, same-multibody link/link, and cross-multibody link/link. Its panel
toggles the public include flags, selects a persistent ignored pair, reports
baseline, option-filtered, pair-ignored, and active contacts, marks filtered
lanes, and records replay snapshots for the query controls.

The current collision ignored-pair follow-up extends that same row instead of
adding a new verifier row. It separates body-kind option filtering from
persistent pair ignores, records the ignored-pair selector in replay snapshots,
exposes the same query counts and lane statuses through
`SceneSetup.info["capture_metrics"]`, and the docked capture wrote a nonblank
960x540 screenshot, 23 PNG frames, one dropped UI warmup frame, and docked
workspace evidence. A later 8-frame capture wrote eight scene-metrics events
for the same row and mirrored the rigid/link/cross-link lane set into the
manifest.

The collision-casts slice, `rigid_collision_casts`, is implemented and visually
captured. It uses public `CollisionGroup.raycast_result()` and
`CollisionGroup.sphere_cast_result()` queries to show nearest/all ray hits,
swept-sphere time of impact, sweep margin, first point/normal, and replayed
ray-offset/all-hit/sweep-radius controls without claiming World contact-solver
or CCD time-step behavior.

The contact-manipulation slice, `rigid_contact_manipulation`, is implemented
and visually captured. It runs matched sequential impulse and rigid IPC
table-push tasks side by side, with controls for executor, pusher launch speed,
table friction, and pusher mass. Its panel plots target travel, pusher-target
gap, contact/proximity evidence, target speed, lateral drift, goal error,
solver divergence, and per-step timing. This is a bounded task-like pusher row,
not a full articulated arm or gripper controller.

The kinematic-driver slice, `rigid_kinematic_driver`, is implemented and
visually captured. It shows IPC prescribed tangential motion carrying a box on a
kinematic support through friction, a zero-friction IPC slip baseline, and the
current sequential-impulse static-like caveat.

The normal prescribed-motion follow-up, `rigid_kinematic_normal_push`, is
implemented as a caveat verifier after `rigid_kinematic_driver`. It drives an
explicit normal kinematic paddle into a target and records IPC normal/heavy
penetration-caveat lanes next to a sequential-impulse push lane, with push
speed, target mass, driver travel, target travel, analytic gap, penetration
depth, contact count, speed ratio, step timing, capture metrics, and replayed
controls. Do not claim robust IPC normal kinematic manipulation from this row.

The constraints slice is implemented and visually captured. `rigid_fixed_joint`
now has perturb/reset controls and plots relative offset, relative orientation,
payload speed, and angular-speed histories. `rigid_limited_joints` now verifies
the locked directions for revolute and prismatic rigid-body joints while showing
the free hinge yaw and slider z-axis travel. `rigid_joint_motor_limits` now
covers the stable World multibody joint-actuator path for rigid links: velocity
motor clamping, a gravity-driven position stop, and capped versus reference
force response. It intentionally does not claim rigid-body joint motor support,
because a local probe showed rigid-body joint actuation is not stable enough for
a bounded public visual verifier yet.

The passive joint-parameter slice is implemented and visually captured as
`rigid_joint_passive_parameters`, immediately after the joint motor/limit row in
the curated workflow. It keeps contacts and gravity out of the scene while
showing World multibody spring/rest, damping, Coulomb friction, and armature
behavior in prismatic lanes. Its panel exposes spring stiffness, rest position,
damping, Coulomb friction, slip force, armature, acceleration-versus-expected
diagnostics, energy histories, and step-profile timing. The row is a passive
parameter verifier, not a motor, limit, or contact-load scene.

The current curation slice records the rigid visual-verification workflow in
`python/examples/demos/README.md`: baseline World rigid dynamics, restitution
material response, contact inspection, solver comparison, executor equivalence,
contact-solver policy, friction threshold, stack stability, contact
manipulation, kinematic drivers, joint constraints, joint motor/limit behavior,
and the existing IPC no-tunneling capability scene. It also records the
no-tunneling scope decision: do not add a separate thin-wall catalog row this
slice unless a future high-speed preset asks a distinct user question.

The durable evidence packet for this slice now lives in
`docs/plans/103-examples-strategy/rigid-body-visual-verification.md`. That
sidecar is the source of truth for scene order, capture commands, test evidence,
visual-smoke evidence, known limitations, and the no-tunneling decision.

A review-readiness pass then tightened the current branch: the controller-heavy
rigid rows now include user-facing controls in replay snapshots, and
`rigid_solver_compare` has a focused wall-response invariant test.

A front-door alignment pass made `rigid_body` the default Python `py-demos`
launch scene and moved the curated rigid visual-verification block to the front
of the global catalog and `World Rigid Body` category. The replay timeline
remains explicitly selectable as `replay_scrubber`.

The Demos navigator follow-up now prefixes the first 36 interactive
**World Rigid Body** viewer titles with workflow position and role, such as
`01/36 Baseline: World Rigid Body` and
`15/36 Solver family: Rigid Solver Compare`, while preserving stable
`py-demos --list` scene ids and titles for scripts. This turns the existing
curated order into an in-app breadcrumb instead of relying only on the README
and PLAN-103 sidecar.

The external-loads slice is implemented and visually captured as
`rigid_external_loads`, immediately after `rigid_body` in the curated workflow.
It keeps gravity and contacts out of the scene so users can inspect persistent
`apply_force()`/`apply_torque()` accumulators, mass-scaled acceleration,
inertia-scaled angular response, one-step pulse clearing, and the static-body
caveat. The row intentionally does not claim point-force, impulse, or contact
load semantics.

The time-step-sensitivity slice is implemented and visually captured as
`rigid_timestep_sensitivity`, immediately after `rigid_link_point_loads` in the
curated workflow. It keeps the displayed simulation-time advance matched across
fine, medium, and coarse lanes while changing each World's integration time
step and substep count. Its panel exposes solver, executor, base time step,
gravity scale, free-fall error, contact timing, clearance, coarse/fine error
ratio, and step-profile timing. The row is a parameter/convergence diagnostic,
not a solver correctness proof or exact contact-threshold claim.

The step-diagnostics slice is implemented as `rigid_step_diagnostics`,
immediately after `rigid_timestep_sensitivity` in the curated workflow. It runs
single-body, contact-pair, and small-stack Worlds under one selected
solver/executor so users can inspect `World.last_step_profile`,
`World.memory_diagnostics`, ECS entity/component counts, contact counts, and
frame-scratch usage without turning the row into another solver-family
comparison. It now also surfaces each top profile stage's domain,
backend-neutral acceleration mask, accelerated-backend flag, and accelerated
stage count so users can distinguish executor parallelism from actual
accelerator use without implying rigid CUDA support. If profiling is compiled
out, the panel still reports memory and contact diagnostics and marks profile
timing unavailable.

The contact-scale budget slice is implemented as
`rigid_contact_scale_budget`, immediately after `rigid_step_diagnostics` in the
curated workflow. It runs one-, four-, and nine-box contact workloads under one
selected solver, executor, frame budget, and friction value. Its panel exposes
contact-point count, bodies, contacts per body, wall time, per-contact cost, top
profile stage, frame-scratch peak usage, ECS counters, worker count,
dense/single wall-time ratio, and budget status. The row is a bounded live-GUI
performance diagnostic, not a benchmark suite or heavy IPC stress packet.

The free-flight slice is implemented and visually captured as
`rigid_free_flight`, immediately after `rigid_body` in the curated workflow. It
keeps contacts out of the scene while showing zero-gravity linear drift, a
gravity arc against analytic trajectory and momentum references, and
low/high-inertia spin lanes. Its panel exposes executor, launch speed, launch
angle, gravity scale, spin speed, inertia ratio, position error, momentum
residual, energy drift, spin momentum/energy ratios, contact count, and
step-profile timing. The row is a no-contact initial-state diagnostic, not a
load, restitution, contact, or solver-family row.

The loop-closure slice is implemented and visually captured as
`rigid_loop_closure`, after the rigid joint motor/limit row in the curated
workflow. It compares the same four-link endpoint POINT closure in residual-only
and solved modes under the variational rigid multibody path. The residual-only
lane remains a live diagnostic and visibly drifts under gravity, while the
solved lane keeps the endpoint target locked and plots residual, tip error, tip
height, joint speed, residual ratio, and step-profile timing. The row is an
endpoint-closure verifier, not a distance-closure or rigid-closure family sweep.

## Current Branch

`feature/rigid-body-gui-visual-verification` - local work in progress. The
branch has been pushed to origin through `b41b9279b60`
(`Expose collision query metrics in captures`). The earlier merge commit
`cd7600f8cda` merged latest `origin/main` into the feature branch before push.
The only manual merge conflict was `python/tests/integration/test_demos_cycle.py`;
the resolved file keeps both the local rigid visual-verification tests and the
new AVBD demo tests from `origin/main`. Post-merge validation passed: the
conflict-resolution `pixi run test-py` command ran the full Python suite and
reported `942 passed, 9 skipped`, and `pixi run lint` passed before the merge
commit was created and pushed.

The current distance-spring follow-up adds `rigid_distance_spring` as row 27/36
after the AVBD breakage lifecycle and before the one-DOF joint row. It shows
unsprung, soft, stiff, and off-center anchor lanes for the public
`World.add_rigid_body_distance_spring()` path, keeps IPC/multibody rejection
scope visible in the panel/docs, exports replay-state controls and
`capture_metrics`, and updates the registry, runner guide, README, PLAN-103
sidecar, changelog, dev-task notes, and PR draft. The focused behavior/replay/
category/order/viewer-title/guidance/docs/README/capture-command/panel guard
reported `17 passed` before and after lint. The real docked capture
`pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_distance_spring_1781220718`
wrote a nonblank 960x540 screenshot with docked-workspace detection, 71 PNG
frames, and 72 scene-metrics events for lanes `free/soft/stiff/offset`.

The previous loop-closure follow-up added the scene, registry row, panel
coverage, behavior coverage, replay-control restore checks, quickstart docs,
workflow docs, changelog entry, and dev-task evidence. The focused
category/order/behavior/replay/panel command reported `5 passed`, and the
`rigid_loop_closure` visual capture wrote a nonblank 960x540 screenshot plus 71
nonblank PNG frames with final contacts at 0. `pixi run test-py` reported
`600 passed, 9 skipped`, `pixi run lint` passed, bounded `pixi run build`
passed with `DART safe jobs: 5`, and `git diff --check` was clean after the
loop-closure scene, registry, test, quickstart, workflow, changelog, and
dev-task updates.

The current passive joint-parameter follow-up has added the scene, registry
row, panel coverage, behavior coverage, replay-control restore checks,
quickstart docs, workflow docs, changelog entry, and dev-task evidence. The full
Python sweep through `pixi run test-py` reported `601 passed, 9 skipped`, and
the `rigid_joint_passive_parameters` visual capture wrote a nonblank 960x540
screenshot plus 119 nonblank PNG frames with final contacts at 0.

The current step-diagnostics follow-up has added the scene, registry row, panel
coverage, behavior coverage, replay-control restore checks, quickstart docs,
workflow docs, changelog entry, and dev-task evidence. The focused
category/order/profile-memory/replay/panel command reported `5 passed`; the
visual capture wrote a nonblank 960x540 screenshot plus 71 nonblank PNG frames;
`pixi run test-py` reported `603 passed, 9 skipped`; `pixi run lint` passed;
and bounded `pixi run build` passed with no work remaining.

The current material-mixing follow-up has added the scene, registry row, panel
coverage, pair-rule behavior coverage, replay-control restore checks,
quickstart docs, workflow docs, changelog entry, and dev-task evidence. The
focused category/order/pair-rule/replay/panel command reported `5 passed`, and
the visual capture wrote a nonblank 960x540 screenshot plus 71 nonblank PNG
frames with final contacts at 0. `pixi run test-py` reported `604 passed, 9
skipped`, `pixi run lint` passed, and bounded `pixi run build` passed with no
work remaining.

The current curation-audit follow-up fixed the PLAN-103 sidecar's duplicate
workflow order number and added
`test_rigid_visual_verification_sidecar_matches_registry_order`. The test keeps
the numbered sidecar workflow unique, consecutive, backed by registered scene
ids, and aligned with the front-of-catalog World rigid-body verifier block while
keeping `rigid_ipc_tunnel` in the separate Rigid IPC capability shelf. The
sidecar validation snapshot is now a compact current-evidence summary instead
of a chronological list of stale per-slice pass counts.

The current README-sidecar drift guard adds
`test_rigid_visual_verification_readme_matches_sidecar_order`, keeping the
Python demo README quick workflow table in exact scene-id order with the
PLAN-103 sidecar. The capture-command companion
`test_rigid_visual_verification_capture_commands_match_workflow` keeps the
sidecar table commands, sidecar quick-refresh commands, and Python demo README
capture commands in the same ordered rigid workflow with matching frame budgets,
960x540 size, and `--show-ui`. The capture helper default now matches the
`py-demos` `rigid_body` front door, pinned by
`test_visual_capture_default_scene_matches_py_demos_front_door`. The focused
default/order/sidecar/README/capture/front-door guard command reported
`5 passed`.

The current backend-status follow-up keeps the next work inside the existing
`rigid_step_diagnostics` row instead of adding another catalog row. The panel
now reports top-stage domain, acceleration mask, backend-active status, and
accelerated-stage count from public `WorldStepStageProfile` metadata. The
focused step-diagnostics/panel command reported `2 passed`.

The current differentiable contact-gradient follow-up keeps the rigid workflow
from duplicating `diff_drone_liftoff` as a second numbered row. Instead, the
README and PLAN-103 sidecar now route rigid users to the Differentiable shelf
when forward contact looks correct but gradient-based optimization stalls. The
shared `diff_drone_liftoff` panel now exposes aware thrust, aware loss, aware
thrust-gradient, analytic loss, and height histories plus the public
`ANALYTIC`/`COMPLEMENTARITY_AWARE` mode labels. A refreshed full
`pixi run test-py` after the curation, plane-shape, and differentiable routing
follow-ups reported `605 passed, 9 skipped`.

The current contact-inspector plane-shape follow-up adds a true public
`CollisionShape.plane` plus sphere lane to `rigid_contact_inspector` and extends
`test_rigid_contact_inspector_reports_contact_manifolds` to verify the plane
shape while continuing to check finite normals, depths, local points, and
compound shape indices. The focused contact/order/sidecar/panel command
reported `4 passed`, and the visual capture wrote a nonblank 960x540 screenshot
plus 23 nonblank PNG frames.

The current screw-joint pitch follow-up adds `rigid_screw_joint_pitch` after
the passive-joint row and before loop closure in the curated World rigid-body
workflow. It uses zero, fine, coarse, and reverse World multibody screw-joint
lanes under gravity to expose pitch as axial translation per radian, including
rotation sign, axial travel, travel-per-radian, effective mass, and
expected-versus-actual acceleration diagnostics. The focused
behavior/replay/category/order/sidecar/README/capture-command/panel command
reported `8 passed`, and the visual capture wrote a nonblank 960x540
screenshot plus 95 nonblank PNG frames with final contacts at 0. The broader
`pixi run test-py` sweep reported `609 passed, 9 skipped`.

The current loop-closure family follow-up keeps the existing
`rigid_loop_closure` workflow row but expands it from a POINT-only endpoint
closure into POINT, DISTANCE, and RIGID public closure families crossed with
residual-only and solved policies. The panel now reports the family-specific
signal users need to choose the narrowest closure: tip error for POINT,
tether-length error for DISTANCE, orientation residual for RIGID, and residual
ratios for all three families. The focused loop-closure/replay/panel command
with category/order/sidecar/README/capture-command drift guards reported
`8 passed`, and the refreshed visual capture wrote a nonblank 960x540
`rigid_loop_closure` screenshot plus 71 nonblank PNG frames with final contacts
at 0. The broader `pixi run test-py` sweep still reported
`609 passed, 9 skipped`.

The current link center-of-mass follow-up adds `rigid_link_center_of_mass`
between `rigid_multibody_dynamics_terms` and `rigid_link_jacobian`. It keeps
visual link geometry centered while moving public `Link.center_of_mass` markers
through centered, mirrored, and high-inertia lanes so users can see zero
gravity torque, opposite acceleration signs, reflected mass, and inertia-scaled
acceleration in one contact-free World. The focused COM/replay/order/docs/panel
guard reported `17 passed`, and the docked visual capture wrote a nonblank
960x540 screenshot plus 71 PNG frames with final contacts at 0.

The current body-mode follow-up adds `rigid_body_modes` directly after the
`rigid_body` front door. It compares dynamic, static, and kinematic rigid-body
mode semantics in one contact-free World with shared solver/executor controls,
gravity and force controls, prescribed kinematic speed, static-drift history,
and kinematic path-error history. The focused
behavior/replay/category/order/sidecar/README/capture-command/panel command
reported `8 passed`; visual capture wrote a nonblank 960x540 screenshot plus
71 PNG frames with final contacts at 0; a standard-library PNG pixel probe
reported RGB variance `[2490.449, 2653.248, 2853.391]` and 3208 unique colors;
`pixi run test-py` reported `610 passed, 9 skipped`; `pixi run lint` passed;
the focused post-lint guard again reported `8 passed`; and `git diff --check`
was clean.

The previous free-flight follow-up added the scene, registry row, panel
coverage, behavior coverage, replay-control restore checks, quickstart docs,
workflow docs, changelog entry, and dev-task evidence. A direct scene probe
verified contact-free Worlds, near-zero zero-gravity drift error, bounded
gravity-arc error and momentum residual, and inertia-scaled spin momentum and
energy ratios. The focused behavior/order/replay/panel command reported
`5 passed`, `py-demos --list` shows the row directly after `rigid_body`, and
the `rigid_free_flight` visual capture wrote nonblank 960x540 PNG frames.
`pixi run test-py` reported `599 passed, 9 skipped`, `pixi run lint` passed,
bounded `pixi run build` passed with `DART safe jobs: 5`, and
`git diff --check` was clean after the final free-flight evidence update.
The previous time-step-sensitivity follow-up added the scene, registry row,
panel coverage, behavior coverage, replay-control restore checks, quickstart
docs, workflow docs, changelog entry, and dev-task evidence. A direct scene
probe verified matched displayed simulation time, ordered pre-contact
free-fall error across fine/medium/coarse time steps, and recorded contact
timing/clearance. The focused behavior/order/replay/panel command reported
`5 passed`, `py-demos --list` shows the row directly after
`rigid_link_point_loads`, and the `rigid_timestep_sensitivity` visual capture
wrote nonblank 960x540 PNG frames. `pixi run test-py` reported
`598 passed, 9 skipped`, `pixi run lint` passed, bounded `pixi run build`
passed with `DART safe jobs: 5`, and `git diff --check` was clean after the
final time-step-sensitivity evidence update.
The previous external-loads follow-up added the scene, registry row, panel
coverage, behavior coverage, replay-control restore checks, quickstart docs,
workflow docs, and dev-task evidence. A direct scene probe verified
mass-scaled linear acceleration, inertia-scaled angular acceleration, one-step
pulse clearing, and zero static drift. The focused behavior/order/replay/panel
command reported `5 passed`, `py-demos --list` shows the row directly after
`rigid_body`, and the `rigid_external_loads` visual capture wrote nonblank
960x540 PNG frames. `pixi run test-py` reported `597 passed, 9 skipped`,
`pixi run lint` passed, bounded `pixi run build` passed with
`DART safe jobs: 5`, and `git diff --check` was clean after the final
external-loads evidence update.
The previous front-door alignment follow-up updated the default launch scene,
catalog order, quickstart docs, workflow docs, and focused ordering/default
tests. The focused default/order/category command reported `3 passed`, and the
`rigid_body` visual capture wrote nonblank 960x540 PNG frames. `pixi run lint`
passed, bounded `pixi run build` passed with `DART safe jobs: 5`, and
`git diff --check` was clean after the final front-door evidence update.
The previous kinematic-driver follow-up has passed a direct scene probe,
focused behavior/order/replay/panel tests, full Python sweep, visual capture,
`pixi run lint`, bounded `pixi run build`, and `git diff --check`.
The previous collision-query-options follow-up has passed the direct scene
probe, focused filter/order/replay/panel tests, full Python sweep, visual
capture, `pixi run lint`, bounded `pixi run build`, and `git diff --check`.

## Immediate Next Step

Use the current handoff snapshot at the top of this file for live branch state.
After this local contact-gradient metrics slice is committed, confirm the branch
is clean and ahead of origin by the local commit. Do not push or open a PR
without fresh explicit approval.

## Context That Would Be Lost

- The broad goal is not complete. This is only the first P0 visual-debug slice.
- Two read-only research passes agreed that the highest-value next pattern is a
  side-by-side rigid solver comparison, followed by friction threshold,
  no-tunneling, and small stacking/mass-ratio scenes.
- A third read-only pass recommended friction threshold before no-tunneling
  because `rigid_ipc_tunnel` and `rigid_solver_compare` already cover the thin
  wall shape. It also recommended avoiding exact-threshold completion claims.
- A fourth read-only pass recommended the small resting-stack / mass-ratio
  verifier next because no-tunneling and constraints already have partial GUI
  coverage.
- Backend controls should compare executor performance/equivalence, not imply a
  separate physics mode.
- Contact-solver policy should stay separate from rigid-body solver family.
  `rigid_contact_solver_compare` is about sequential-impulse contacts versus
  boxed-LCP contacts under the same rigid solver, not IPC-vs-sequential rigid
  method-family behavior.
- Restitution should remain a relative material-response diagnostic. The
  default sequential-impulse lane ordering verifies higher restitution gives a
  stronger rebound in this setup; the energy plots are debugging signals, not a
  conservation guarantee.
- Contact inspection should remain query-level observability. Use it to debug
  contact packets before claiming solver-family differences. Broaden the
  existing contact-inspector row for public shape-family coverage instead of
  adding a duplicate collision-sandbox catalog row.
- Collision query options should remain a query-filtering diagnostic for
  `World.collide(options)`: rigid/rigid, rigid/link, same-multibody link/link,
  cross-multibody link/link, and persistent ignored pairs. It should not
  duplicate the shape-family manifold inspector or become a solver benchmark.
- Kinematic driver visuals should stay scoped to tangential prescribed motion
  under IPC: moving supports/conveyors, slip, and the sequential-impulse
  static-like caveat. Normal prescribed contact now belongs in
  `rigid_kinematic_normal_push`, which intentionally exposes the IPC
  penetration caveat next to an SI push lane instead of claiming robust IPC
  normal manipulation.
- The app front door should stay aligned with the curated rigid workflow:
  default `py-demos` launch opens `rigid_body`, while `replay_scrubber` remains
  an explicit scene for replay-specific debugging.
- The next distinct rigid-body feature gap is external loads: persistent
  `RigidBody.force`/`torque` accumulators and `apply_force`/`apply_torque`
  response. That gap is now covered by `rigid_external_loads`; keep any future
  point-force or impulse work separate unless it answers a distinct public API
  question.
- The no-contact rigid free-flight/initial-state gap is now covered by
  `rigid_free_flight`. Add a separate rigid energy/momentum accounting row only
  if it answers a distinct user question beyond the free-flight diagnostics.
  Keep contacts, restitution, load accumulators, and solver comparisons out of
  that row unless they become the distinct question.
- The explicit loop-closure gap is now covered by `rigid_loop_closure` as a
  public-family comparison row. POINT covers endpoint locking, DISTANCE covers
  tether length, and RIGID covers full-pose welds; add another row only for a
  distinct behavior such as compliance or breakage.
- The passive joint-parameter gap is now covered by
  `rigid_joint_passive_parameters` as a contact-free World multibody row. Add
  future passive rows only if they answer a distinct public question beyond
  spring/rest, damping, Coulomb friction, and armature.
- Sleeping/deactivation/island activation rows should stay deferred until
  public dartpy APIs expose those states; current branch audits found only
  internal island machinery and no public sleep/wake/deactivation surface.
- The current branch intentionally keeps `rigid_ipc_tunnel` as a focused IPC
  capability scene in the Rigid IPC shelf rather than adding another
  thin-wall comparison until there is a distinct user question not covered by
  `rigid_solver_compare`.
- The interrupted post-`rigid_ipc_tunnel` audit identified
  `diff_drone_liftoff` as the next possible related-shelf metrics target for
  contact-gradient evidence. It now has capture metrics, should stay in the
  Differentiable shelf, and should not become a numbered rigid row unless a
  distinct user question appears.
- Rigid-body joint motor visuals should stay deferred until that API behavior is
  stable enough to support bounded visual assertions. The current motor/limit
  row uses World multibody joints instead.
- Baseline local validation for the P0 workflow passed: focused rigid workflow
  checks, all documented visual captures, `pixi run lint`, `pixi run build`,
  the Python suite, `pixi run -e cuda test-all`, and `git diff --check`.
- Latest contact-solver follow-up validation passed: the 13-test focused rigid
  workflow refresh, `rigid_contact_solver_compare` visual capture,
  `pixi run lint`, `pixi run build`, and `git diff --check`.
- Latest restitution follow-up validation passed: the 14-test focused rigid
  workflow refresh, `rigid_restitution_ladder` visual capture, `pixi run lint`,
  `pixi run build`, and `git diff --check`.
- Latest joint motor/limit capture-metrics follow-up validation passed:
  focused motor-limit guard reported `1 passed`, the broader workflow/doc drift
  guard reported `11 passed`, the docked capture under
  `/tmp/dart_capture_joint_motor_limits_metrics_1781234533` wrote nonblank
  960x540 PNG frames plus 72 scene-metrics events, `pixi run lint` passed,
  bounded `pixi run build` passed with `DART safe jobs: 5`, and
  `git diff --check` passed.
- Latest passive joint-parameter capture-metrics handoff:
  focused passive-parameter guard reported `1 passed`, and the docked capture
  under `/tmp/dart_capture_joint_passive_parameters_metrics_1781235045` wrote
  a nonblank 960x540 screenshot, 119 PNG frames, and 120 scene-metrics events
  with manifest ranges for spring/damped energy, stiction/slip, armature
  acceleration/position gaps, step timing, time step, and world time. The
  skipped broader workflow/doc drift guard later reported `11 passed`;
  `pixi run lint`, bounded `pixi run build`, and `git diff --check` passed.
- Latest screw-joint pitch capture-metrics follow-up validation passed so far:
  focused screw-pitch guard reported `1 passed`, and the docked capture under
  `/tmp/dart_capture_screw_joint_pitch_metrics_1781235714` wrote a nonblank
  960x540 screenshot, 95 PNG frames, and 96 scene-metrics events with manifest
  ranges for pitch ratios, fine/coarse/reverse angle and axial travel,
  near-zero acceleration residuals, step timing, time step, and world time.
  The broader workflow/doc drift guard reported `12 passed`; `pixi run lint`,
  bounded `pixi run build`, and `git diff --check` passed.
- Latest loop-closure follow-up validation passed: the focused
  category/order/behavior/replay/panel command reported `5 passed`;
  `rigid_loop_closure` visual capture wrote a nonblank 960x540 screenshot and
  71 nonblank PNG frames with final contacts at 0; `pixi run test-py` reported
  `600 passed, 9 skipped`; `pixi run lint` passed; bounded
  `pixi run build` passed with `DART safe jobs: 5`; and `git diff --check` was
  clean.
- Latest passive joint-parameter follow-up validation passed so far:
  `pixi run test-py` reported `601 passed, 9 skipped`, and
  `rigid_joint_passive_parameters` visual capture wrote a nonblank 960x540
  screenshot plus 119 nonblank PNG frames with final contacts at 0.
  `pixi run lint` passed; bounded `pixi run build` passed with
  `DART safe jobs: 5`; and `git diff --check` was clean.
- Latest contact-inspector shape-family follow-up validation passed: the
  3-test focused contact/panel/replay command reported `3 passed`, the
  `rigid_contact_inspector` visual capture wrote nonblank 960x540 PNG frames,
  `pixi run lint` passed, bounded `pixi run build` passed with
  `DART safe jobs: 5`, and `git diff --check` was clean. The focused contact
  integration test verifies every lane has finite contact geometry and compound
  shape-index reporting.
- Current collision-query-options follow-up validation passed so far: direct
  scene probe reported four baseline contacts and four active contacts; the
  focused filter/order/replay/panel command reported `4 passed`;
  `pixi run test-py` reported `595 passed, 9 skipped`; and
  `rigid_collision_query_options` visual capture wrote nonblank 960x540 PNG
  frames. `pixi run lint`, bounded `pixi run build`, and `git diff --check`
  also passed after the scene, registry, test, replay-snapshot, and
  documentation updates.
- Current kinematic-driver follow-up validation passed so far: direct scene
  probe showed IPC grip carrying the box, IPC zero-friction slip, and the
  sequential static-like caveat; the focused behavior/order/replay/panel command
  reported `5 passed`; and `rigid_kinematic_driver` visual capture wrote
  nonblank 960x540 PNG frames. `pixi run test-py` reported `596 passed, 9
skipped`; `pixi run lint` passed; bounded `pixi run build` passed with
  `DART safe jobs: 5`; and `git diff --check` was clean after the final
  kinematic-driver evidence update.
- Current front-door alignment validation passed so far: the focused
  default/order/category command reported `3 passed`, `py-demos --list` shows
  `rigid_body` and the curated rigid verifier block at the front of the catalog,
  `rigid_body` visual capture wrote nonblank 960x540 PNG frames,
  `pixi run lint` passed, bounded `pixi run build` passed with
  `DART safe jobs: 5`, and `git diff --check` was clean after the final
  front-door evidence update.
- Current external-loads validation passed so far: direct scene probe reported
  the intended force, torque, pulse, and static-body metrics; focused
  behavior/order/replay/panel tests reported `5 passed`; `py-demos --list`
  shows `rigid_external_loads` directly after `rigid_body`; and
  `rigid_external_loads` visual capture wrote nonblank 960x540 PNG frames.
  `pixi run test-py` reported `597 passed, 9 skipped`; `pixi run lint` passed;
  bounded `pixi run build` passed with `DART safe jobs: 5`; and
  `git diff --check` was clean after the final evidence update.
- Current point-load validation passed so far: direct scene probe reported
  centered translation, world-space off-center lever-arm yaw acceleration,
  one-step pulse clearing, double-apply accumulation, and world/local frame
  separation; focused behavior/order/replay/panel tests reported `5 passed`;
  and visual capture wrote a nonblank 960x540 screenshot plus 71 nonblank PNG
  frames with final contacts at 0. `pixi run test-py` reported `602 passed, 9
skipped`; `pixi run lint` passed; and bounded `pixi run build` passed with
  `DART safe jobs: 5`.
- Current step-diagnostics validation passed so far: focused
  category/order/profile-memory/replay/panel tests reported `5 passed`;
  visual capture wrote a nonblank 960x540 screenshot plus 71 nonblank PNG
  frames; `pixi run test-py` reported `603 passed, 9 skipped`;
  `pixi run lint` passed; and bounded `pixi run build` passed with no work
  remaining. `git diff --check` was clean after the final documentation update.
- Current material-mixing validation passed so far: focused
  category/order/pair-rule/replay/panel tests reported `5 passed`, and visual
  capture wrote a nonblank 960x540 screenshot plus 71 nonblank PNG frames with
  final contacts at 0. `pixi run test-py` reported `604 passed, 9 skipped`;
  `pixi run lint` passed; and bounded `pixi run build` passed with no work
  remaining.
- Current multibody solver-family validation passed so far:
  `rigid_multibody_solver_family` is implemented after the screw-joint pitch
  row and before the loop-closure family row. The focused
  behavior/order/replay/panel guard reported `8 passed`; `pixi run test-py`
  reported `611 passed, 9 skipped`; visual capture wrote a nonblank 960x540
  screenshot plus 71 PNG frames with final contacts at 0 and RGB variance
  `[2505.408, 2673.722, 2866.646]`.
- Current multibody dynamics-terms validation passed so far:
  `rigid_multibody_dynamics_terms` is implemented after the screw-joint pitch
  row and before the solver-family row. The focused
  behavior/order/replay/panel guard reported `8 passed`; `pixi run test-py`
  reported `612 passed, 9 skipped`; visual capture wrote a nonblank 960x540
  screenshot plus 95 PNG frames with final contacts at 0 and RGB variance
  `[2496.525, 2663.351, 2868.943]`.
- Current link-Jacobian validation passed so far: `rigid_link_jacobian` is
  implemented after generalized dynamics terms and before the solver-family
  row. The focused behavior/order/replay/panel guard reported `8 passed`;
  `pixi run test-py` reported `613 passed, 9 skipped`;
  visual capture wrote a nonblank 960x540 screenshot plus 95 PNG frames with
  final contacts at 0 and RGB variance `[2483.852, 2653.334, 2866.583]`.
- Current contact-scale budget validation passed so far:
  `rigid_contact_scale_budget` is implemented after the step-diagnostics row and
  before the restitution ladder. The focused behavior/order/replay/panel guard
  reported `8 passed`; `pixi run test-py` reported `614 passed, 9 skipped`;
  `pixi run lint` passed; the focused post-lint guard again reported `8 passed`;
  and `git diff --check` was clean. Visual capture wrote a nonblank 960x540
  screenshot plus 71 PNG frames. A 72-frame controller probe reported
  one/four/nine-box contact points at 4, 16, and 36 with dense/single wall ratio
  `6.016` and RGB variance `[2502.671, 2667.095, 2860.303]`. The selected slice
  came from a fresh audit of remaining public rigid workflow gaps:
  arbitrary-point/contact Jacobian and sleep/island rows are deferred until
  stronger public APIs exist.
- Current collision-casts validation passed so far: `rigid_collision_casts` is
  implemented after the collision-query-options row and before the solver-family
  row. The focused behavior/order/replay/panel guard reported `7 passed`;
  `pixi run test-py` reported `615 passed, 9 skipped`; `pixi run lint` passed;
  and bounded `pixi run build` passed with `DART safe jobs: 3` and
  `ninja: no work to do`; `git diff --check` was clean after the final
  documentation update. Visual capture wrote a nonblank 960x540 screenshot plus
  47 PNG frames with final contacts at 0, docked UI detected, 4286 unique colors,
  and RGB variance `[2507.382, 2770.902, 2914.942]`.
- Current Demos navigator title validation passed so far: the focused
  list/sidecar/viewer-title/README guard reported `4 passed`; `pixi run lint`
  passed; the short docked capture command for `rigid_body` wrote a nonblank
  960x540 screenshot plus 7 PNG frames through the docked viewer path;
  `pixi run test-py` reported
  `616 passed, 9 skipped`; bounded `pixi run build` reported
  `ninja: no work to do`; and `git diff --check` was clean after the evidence
  update.
- The bounded contact-rich manipulation row is now implemented as
  `rigid_contact_manipulation`.
- A fuller articulated arm/gripper row remains deferred after the latest public
  API/runtime probe. Rigid-body joints are not supported by the IPC rigid-body
  solver, multibody link contacts do not expose material/friction controls, and
  scripted IPC two-jaw pinch settings that actually carry a target measured in
  the hundreds of milliseconds per step. Do not add this as a default live GUI
  row until the public API can express a stable, interactive gripper without
  overclaiming IK, actuator dynamics, or link-material behavior.
- The local review packet draft now lives at `PR_DRAFT.md` and follows the
  repository PR template for a `main`/DART 7.0 review. That draft-only slice
  did not perform a commit, push, or PR mutation.
- Fresh pre-review validation on 2026-06-10: `pixi run test-py` reported
  `615 passed, 9 skipped`; bounded `pixi run build` passed with
  `DART safe jobs: 3` and `ninja: no work to do`; `pixi run lint` passed; the
  focused post-lint guard reported `7 passed`; `git diff --check` was clean; and
  the current branch still has no associated PR.
- Fresh navigator-title validation on 2026-06-11: `pixi run test-py` reported
  `616 passed, 9 skipped`; bounded `pixi run build` passed with
  `DART safe jobs: 3` and `ninja: no work to do`; `pixi run lint` passed; the
  focused list/sidecar/viewer-title/README guard reported `4 passed`;
  `py-demo-capture` produced a nonblank docked `rigid_body` screenshot; and
  `git diff --check` was clean. The current branch still has no associated PR.
- Fresh breakage-lifecycle validation on 2026-06-11: the focused
  category/order/sidecar/viewer-title/README/capture-command/panel guard reported
  `8 passed`, and `py-demo-capture` produced a nonblank docked
  `rigid_joint_breakage` screenshot plus 47 PNG frames with final contacts at 0.
  `pixi run test-py` reported `617 passed, 9 skipped`; bounded
  `pixi run build` passed with `DART safe jobs: 3` and `ninja: no work to do`;
  `pixi run lint` passed; and `git diff --check` was clean after the evidence
  update.
- Fresh navigator-count drift validation on 2026-06-11: the focused
  viewer-title/docs-count/sidecar/README guard reported `4 passed` after fixing
  stale navigator-count examples to `15/36`; `pixi run lint` passed after adding
  the guard.
- Fresh frame-hierarchy validation on 2026-06-11: the focused
  category/order/viewer-title/docs-count/sidecar/README/capture-command/frame
  invariant/replay/panel guard reported `12 passed` before and after lint.
  `py-demo-capture` produced a nonblank docked `rigid_frame_hierarchy`
  screenshot plus 71 PNG frames with final contacts at 0. `pixi run lint`
  passed; bounded `pixi run build` reported `ninja: no work to do`; and
  `git diff --check` was clean after the evidence update.
- Fresh baseline-hardening validation on 2026-06-11: the focused
  baseline/body-mode/frame/breakage/replay/category/order/viewer-title/docs-count/
  sidecar/README/capture-command/panel guard reported `15 passed`. The docked
  `rigid_body` visual capture produced a nonblank 960x540 screenshot plus 23 PNG
  frames with final contacts at 0. `pixi run lint` passed, and bounded
  `pixi run build` reported `ninja: no work to do`.
- Fresh spin/roll coupling validation on 2026-06-11: the focused
  category/order/viewer-title/docs-count/sidecar/README/capture-command/spin-roll/
  replay/panel guard reported `10 passed`. The docked
  `rigid_spin_roll_coupling` visual capture produced a nonblank 960x540
  screenshot plus 95 PNG frames with final contacts at 0. `pixi run lint`
  passed; bounded `pixi run build` reported `ninja: no work to do`; and
  `git diff --check` was clean after the evidence update.
- Fresh in-viewer workflow-guide follow-up on 2026-06-11: a specialized UX
  audit found that the 36-row learning path was still mostly documented outside
  the app. The runner now injects a compact `Rigid Workflow` panel for numbered
  World Rigid Body rows, sourced from the PLAN-103 user questions, with inspect
  signals and previous/next route guidance. The focused
  category/order/viewer-title/guidance/docs-count/sidecar/README/capture-command/
  spin-roll/replay/panel guard reported `13 passed` before and after lint.
  Docked visual captures for `rigid_body`, `rigid_solver_compare`, and
  `rigid_loop_closure` each produced nonblank 960x540 screenshots plus 7 PNG
  frames with final contacts at 0. `pixi run lint` passed, and bounded
  `pixi run build` reported `ninja: no work to do`.
- Fresh checklist/contact-body hardening on 2026-06-11: the shared
  `Rigid Workflow` panel now presents each numbered row as `Question`,
  `Try first`, `Look for`, and `Do not infer` so the main control, healthy
  signal, and scope caveat are visible in-app. `rigid_collision_query_options`
  now surfaces public `Contact.body_a/body_b` `CollisionBody` kind, validity,
  and cast diagnostics for rigid/rigid, rigid/link, same-multibody link/link,
  and cross-multibody link/link lanes. The focused
  category/order/viewer-title/guidance/docs-count/sidecar/README/capture-command/
  query-options/spin-roll/replay/panel guard reported `14 passed` after lint.
  Docked visual captures for `rigid_body`, `rigid_solver_compare`,
  `rigid_loop_closure`, and `rigid_collision_query_options` produced nonblank
  960x540 screenshots.
- Fresh workflow-route navigation on 2026-06-11: Python panel contexts now
  expose `request_scene_switch(scene_id)`, and the shared `Rigid Workflow`
  panel renders previous/next routes plus a direct row selector as scene-switch
  rows. The focused panel route/docs guard reported `9 passed`; after rebuilding
  dartpy, the focused GUI lifecycle binding guard reported `2 passed` for
  `request_scene_switch()` and `request_scene_replay()` state transitions.
- Fresh workflow-search/restart hardening on 2026-06-11: Python panel contexts
  now also expose `request_scene_replay(scene_id)`, and the shared
  `Rigid Workflow` panel renders a restart command plus a text filter over row
  ids, questions, checklist text, and inspect signals. The filter now includes
  explicit `NN/MM` row-id tokens such as `15/36` so documented row-id search
  switches to the intended workflow row. The focused panel/stub guard reported
  `7 passed`.
- Fresh collision-casts capsule hardening on 2026-06-11:
  `rigid_collision_casts` now visualizes public swept-capsule
  time-of-impact queries alongside raycast and swept-sphere probes, with
  capsule offset/radius/height controls, margins, point/normal metrics, and
  replayed controls. The focused casts/replay guard reported `2 passed`; the
  docked visual capture produced a nonblank 960x540 screenshot plus 47 PNG
  frames with final contacts at 0.
- Fresh stack-stability verification hardening on 2026-06-11: the existing
  `rigid_stack_stability` invariant test now checks both solver lanes,
  per-lane metric histories, and finite top-x divergence while keeping the
  strict stability thresholds on IPC. The focused stack guard reported
  `3 passed`, and the docked visual capture produced a nonblank 960x540
  screenshot plus 23 PNG frames.

The latest follow-up promotes the stable `contact` scene id into row 18/36 as
`Rigid Link Contact`, directly after contact-solver policy and before the
friction-threshold row. It now shows multibody links dropping, friction-sliding,
and pushing a rigid target through the public World contact path, with executor,
friction/restitution, drop/slide/push controls, link contact counts, rebound,
slide travel, target travel, replay restore coverage, and docked visual capture.
Arm/gripper manipulation, direct contact impulse, sleep/island/deactivation,
and loop compliance remain blocked until public APIs or runtime evidence justify
GUI rows.

Fresh link-contact validation on 2026-06-11: the focused
order/guidance/docs/capture/contact/replay/panel guard reported `18 passed`;
the docked `contact` capture produced a nonblank 960x540 screenshot plus 71 PNG
frames; `pixi run lint`, bounded default `pixi run build`, and
`git diff --check` passed; and CUDA-host `pixi run -e cuda test-all` reported
all 7 stages passed, including 206 C++ tests, 68 simulation tests, 643 Python
items, documentation, and CUDA benchmark smoke.

Fresh capture-evidence hardening on 2026-06-11: `py-demo-capture` manifests now
record requested width/height/frames, converted frame count, and
`visual_evidence` for the screenshot and first UI-ready frame: dimensions,
nonzero pixels, unique RGB count, RGB/luminance variance, and
docked-workspace detection. `pixi run test-py` reported
`636 passed, 9 skipped`; a direct `pixi run python -m pytest` probe still needs
the built dartpy `PYTHONPATH` and is not the right entry point for this suite.

Fresh guided-replay timeline hardening on 2026-06-11: the shared bottom
`Replay` panel now accepts optional `replay_timeline` metadata so a scene can
feed the saved-state scrubber one diagnostic signal and one marker track from
recorded replay snapshots. `rigid_solver_compare` uses this to show position
divergence and near-wall marker frames without adding another row or panel. The
focused replay timeline and solver-comparison guard reported `5 passed`, and
`pixi run test-py` reported `640 passed, 9 skipped` on the post-lint tree.

Fresh related-evidence routing on 2026-06-11: the runner-owned `Rigid Workflow`
panel now keeps non-numbered shelves visible without changing the 36-row order.
Row 3/36 `rigid_free_flight` links to `floating_base` as a broader
floating-joint drift/spin example, row 32/36
`rigid_multibody_dynamics_terms` links to `articulated` as a broader two-link
arm example, row 15/36 `rigid_solver_compare` links to `rigid_ipc_tunnel` as a
focused IPC no-tunneling view, and row 17/36
`rigid_contact_solver_compare` links to `diff_drone_liftoff` as a
differentiable contact-gradient route. The same route table also links row
18/36 `contact` to `avbd_rigid_fixed_joint_contact`, row 26/36
`rigid_joint_breakage` to the AVBD fixed/spherical break-reset scenes, and row
29/36 `rigid_joint_motor_limits` to the AVBD revolute/prismatic motor scenes.
The PLAN-103 sidecar owns the route table, tests verify those scene ids remain
registered outside the numbered workflow, and `Find row` now indexes related
shelf names, scene ids, labels, and scope notes so searches such as
`floating_base`, `two-link arm`, `rigid_ipc_tunnel`, `contact gradient`,
`avbd fixed contact`, `avbd spherical`, and `avbd prismatic` route to the
numbered source row. When the match comes only from related evidence, the
search result labels the target scene and the tooltip explains the related
shelf. The sidecar and Python demo README also document docked
`py-demo-capture --show-ui` commands for every related target, with drift
coverage tying those commands to the route table.

Fresh capture-first IPC stack packet on 2026-06-11: `rigid_ipc_stack_packet`
was added to the non-numbered Rigid IPC shelf, outside the 36-row World Rigid
Body workflow, to answer what happens when a four-box IPC stack leaves the live
demo budget. Its panel reports frame-budget status, wall time, min clearance,
contact count, top drift, height error, max speed, and the
`bm_rigid_ipc_solver` benchmark owner. The PLAN-103 sidecar now has a dedicated
`Capture-First Rigid IPC Packets` table so future heavy packets stay separate
from numbered workflow rows. The focused catalog/sidecar/panel/packet guard
reported `12 passed`; the docked capture produced a nonblank 960x540 screenshot
plus 23 PNG frames, and a 24-step probe ended capture-first with min clearance
`0.000267` m, top drift `0.000360` m, and last-step wall time `805.087` ms.

Fresh scene-metrics capture hardening on 2026-06-11: `py-demo-capture` now
passes a runner-local metrics log path to `py-demos`; scenes that expose
`SceneSetup.info["capture_metrics"]` write per-frame `scene_metrics.jsonl`
events during the actual viewer loop, and the capture manifest summarizes the
stream with first/latest events, per-key presence counts, and top-level numeric
ranges. `rigid_ipc_stack_packet` exports its clearance, contact-count, drift,
height-error, speed, wall-time, frame-budget, solver, capture-first, and
`bm_rigid_ipc_solver` benchmark fields through that hook. The capture/runner
unit guard reported `25 passed`, the stack-focused guard reported `3 passed`,
and the real docked stack capture wrote 24 scene-metrics events with frame 24
ending capture-first at min clearance `0.000267` m and step wall time
`626.778` ms.

Fresh runtime-row capture metrics on 2026-06-11: the numbered runtime evidence
rows now use the same capture-metrics hook. `rigid_step_diagnostics` exports
solver/executor, time step, world time, and each lane's profile, memory, speed,
and contact counters; `rigid_contact_scale_budget` adds budget, friction,
dense/single wall-time ratio, and per-lane contact-budget metrics. The focused
guard
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads -q`
reported `2 passed`. Short real `py-demo-capture --show-ui` runs for both rows
wrote eight scene-metrics events each and mirrored the expected lane sets into
the manifests.

Fresh numbered workflow capture hardening on 2026-06-11: the capture test now
uses the docking build to run a real `rigid_contact_scale_budget --show-ui`
capture through the numbered World Rigid Body workflow. The new guard checks
the nonblank docked screenshot and first frame, confirms a `scene_metrics`
manifest summary exists, and verifies the latest metrics row exports
`single/medium/dense`. A companion factory-composition guard verifies the
actual numbered row gets `Rigid Workflow`, `Rigid Contact Scale Budget`, and
`Replay` panels in order.

Fresh post-push replay-contract refresh on 2026-06-11: the branch was committed
as `245095f1164` and pushed to
`origin/feature/rigid-body-gui-visual-verification`. The first full
`pixi run test-py` caught `rigid_ipc_stack_packet` missing shared replay-state
hooks; the packet now exposes capture/restore callbacks for its controls and
history, the focused replay/stack guard reported `2 passed`, and the refreshed
full Python sweep reported `648 passed, 9 skipped`.

Fresh workflow-search UX follow-up on 2026-06-11: a specialized UX audit found
that `Find row` ranked scope caveats before intent matches. The runner now
scores row ids, scene ids, labels, questions, and positive signals before scope
caveats, so `contact` and `solver` searches surface the intended workflow rows.
Related-shelf links now include the target shelf and scene id in the visible
row label.

Fresh kinematic normal-push caveat follow-up on 2026-06-11:
`rigid_kinematic_normal_push` is row 24/36 after the tangential kinematic-driver
row. The focused order/guidance/docs/replay/panel/normal-push guard reported
`17 passed`. The standard docked capture wrote a nonblank 960x540 screenshot,
71 PNG frames, and 72 scene-metrics events; frame 72 ended with IPC normal and
heavy lanes at `status=ipc penetration caveat` with about 0.125 m depth and
near-zero target travel, while the sequential-impulse lane reached
`status=pushed` with about 0.123 m target travel.

Fresh solver/contact comparison capture-metrics follow-up on 2026-06-11:
`rigid_solver_compare` now records method-family capture metrics for
sequential impulse vs IPC cases, and `rigid_contact_solver_compare` records
contact-policy capture metrics for sequential-impulse vs boxed-LCP contact
methods. The focused guard reported `2 passed`. The real docked captures wrote
24 and 72 scene-metrics events respectively, with nonblank 960x540 docked
screenshots and manifest summaries for case enums, controls, executor, world
time, and divergence histories.

Fresh baseline capture-metrics follow-up on 2026-06-11: `rigid_body` now records
the default first-run scene's solver/material controls, dynamic-body count,
current speed/height/energy/contact/timing diagnostics, and history ranges in
`py-demo-capture` manifests. The focused baseline guard reported `1 passed`, and
the real docked capture wrote a nonblank 960x540 screenshot, 23 PNG frames, and
24 scene-metrics events.

The specialized-agent audit for the next slice ranked row 18 `contact` first,
row 12 `rigid_contact_inspector` second, and row 5 `rigid_external_loads` third
for capture-metrics follow-up value/risk. Prefer `contact` next unless PR/CI
state or maintainer instruction changes the priority.

## How to Resume

```bash
git checkout feature/rigid-body-gui-visual-verification
git fetch origin main feature/rigid-body-gui-visual-verification
git status -sb
git log --oneline --decorate --max-count=5
```

If the branch is clean and matches origin after this handoff push, this slice is
done locally. Use `PR_DRAFT.md` only after maintainer approval to update or open
a PR. For future implementation work, refresh the focused curation guard before
editing:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_default_timeline_without_metadata \
    python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata \
    python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_keeps_frame_marks_for_signal_only_metadata \
    python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_ignores_malformed_timeline_metadata \
    python/tests/integration/test_demos_cycle.py::test_rigid_solver_compare_records_wall_response -q
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/integration/test_demos_cycle.py::test_rigid_solver_compare_records_wall_response \
    python/tests/integration/test_demos_cycle.py::test_rigid_body_baseline_reports_restartable_first_run_diagnostics \
    python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics \
    python/tests/integration/test_demos_cycle.py::test_rigid_free_flight_preserves_initial_state_diagnostics \
    python/tests/integration/test_demos_cycle.py::test_rigid_frame_hierarchy_tracks_body_fixed_frame \
    python/tests/integration/test_demos_cycle.py::test_rigid_external_loads_scale_force_and_torque_response \
    python/tests/integration/test_demos_cycle.py::test_rigid_link_point_loads_show_lever_arm_and_frame_semantics \
    python/tests/integration/test_demos_cycle.py::test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size \
    python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters \
    python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads \
    python/tests/integration/test_demos_cycle.py::test_rigid_contact_inspector_reports_contact_manifolds \
    python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds \
    python/tests/integration/test_demos_cycle.py::test_rigid_collision_casts_report_nearest_all_and_swept_hits \
    python/tests/integration/test_demos_cycle.py::test_rigid_restitution_ladder_orders_rebound_height \
    python/tests/integration/test_demos_cycle.py::test_rigid_material_mixing_applies_pair_rules \
    python/tests/integration/test_demos_cycle.py::test_rigid_contact_solver_compare_records_coupled_contact_policy \
    python/tests/integration/test_demos_cycle.py::test_rigid_link_contact_exercises_multibody_contact_response \
    python/tests/integration/test_demos_cycle.py::test_rigid_friction_threshold_separates_stick_and_slip_lanes \
    python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll \
    python/tests/integration/test_demos_cycle.py::test_rigid_contact_manipulation_pushes_target_toward_goal \
    python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc \
    python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_normal_push_exposes_normal_pusher_caveat \
    python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage \
    python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort \
    python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response \
    python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation \
    python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures \
    python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families \
    python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls -q
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/unit/collision/test_collision.py::test_continuous_capsule_cast_result -q
pixi run test-py -- python/tests/unit/test_py_demo_panels.py python/tests/integration/test_demos_cycle.py
pixi run py-demo-capture -- --scene rigid_body --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_body_modes --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_free_flight --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_frame_hierarchy --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_external_loads --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_point_loads --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_timestep_sensitivity --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_step_diagnostics --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_scale_budget --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_restitution_ladder --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_material_mixing --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_inspector --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_collision_casts --frames 48 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_solver_compare --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_executor_equivalence --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_solver_compare --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene contact --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_friction_threshold --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_spin_roll_coupling --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_link_jacobian --frames 96 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_multibody_solver_family --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_tunnel --frames 24 --width 960 --height 540 --show-ui
pixi run py-demo-capture -- --scene rigid_ipc_stack_packet --frames 24 --width 960 --height 540 --show-ui
pixi run lint
DART_PARALLEL_JOBS=${DART_SAFE_JOBS:-5} \
    CTEST_PARALLEL_LEVEL=${DART_SAFE_JOBS:-5} \
    CMAKE_BUILD_PARALLEL_LEVEL=${DART_SAFE_JOBS:-5} \
    pixi run build
git diff --check
```

## 2026-06-11 Fixed/One-DOF Joint Capture Metrics Follow-Up

The current follow-up adds scene-owned `capture_metrics` hooks to the existing
`rigid_fixed_joint` and `rigid_limited_joints` rows. The captures now record
fixed relative-transform errors plus revolute/prismatic locked-axis errors in
`scene_metrics.jsonl`, matching the panel diagnostics without changing row
ordering or scope claims.

Validation so far:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/integration/test_demos_cycle.py::test_rigid_fixed_joint_verifier_restores_captured_transform \
    python/tests/integration/test_demos_cycle.py::test_rigid_one_dof_joint_verifier_preserves_locked_directions \
    python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls \
    python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar \
    python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order \
    python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order \
    python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow \
    python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q
pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_fixed_joint_metrics_1781222114
pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_limited_joints_metrics_1781222114
pixi run lint
pixi run build
```

Observed capture evidence:

- `rigid_fixed_joint`: nonblank 960x540 docked screenshot, 23 PNG frames, 24
  scene-metrics events, latest translation error
  `2.1673896011265015e-10`.
- `rigid_limited_joints`: nonblank 960x540 docked screenshot, 23 PNG frames, 24
  scene-metrics events, latest hinge radius/z and slider orthogonal errors
  `0.0`.

## 2026-06-11 Joint-Breakage Capture Metrics Follow-Up

The current follow-up adds scene-owned `capture_metrics` to the shared
`avbd_rigid_breakable_joint` builder and tags the numbered
`rigid_joint_breakage` row with its own row id. The metric stream records the
AVBD fixed break-force lifecycle: intact/broken status, captured-offset error,
payload release distance, payload speed, dynamic break-force value, and whether
the capture saw a broken state. This keeps the row AVBD-pinned and does not add
sequential-impulse or IPC breakage claims.

Validation so far:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage \
    python/tests/integration/test_demos_cycle.py::test_avbd_breakable_joint_demo_marks_and_resets_joint -q
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest \
    python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle -q
pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_breakage_metrics_1781223331
pixi run lint
DART_PARALLEL_JOBS=${DART_SAFE_JOBS:-5} CTEST_PARALLEL_LEVEL=${DART_SAFE_JOBS:-5} CMAKE_BUILD_PARALLEL_LEVEL=${DART_SAFE_JOBS:-5} pixi run build
```

Observed capture evidence:

- `rigid_joint_breakage`: nonblank 960x540 docked screenshot, 47 PNG frames,
  48 scene-metrics events, latest status `broken`, `saw_broken=1.0`, and latest
  `payload_release_distance=0.41319290960568955`.
