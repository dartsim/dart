# PR Draft: Rigid-Body Visual Verification Workflow

## Summary

- Curates the Python `py-demos` front door around a 36-row World rigid-body
  visual-verification workflow.
- Adds focused rigid-body GUI scenes, panels, replay-state coverage, capture
  commands, and drift checks for solver, executor, contact, material, joint,
  multibody, profiling, and loop-closure debugging.
- Promotes the durable evidence matrix to the PLAN-103 sidecar so future agents
  can keep the user-facing workflow synchronized.

## Motivation / Problem

- DART 7 introduces new rigid-body solvers, executors, contact policies,
  diagnostics, and multibody APIs in parallel, but users need a coherent visual
  path to understand and debug them.
- The existing catalog had capability scenes, but not a carefully ordered
  first-run workflow that answers user questions before dropping into broad
  examples or planned ports.

## Changes / Key Changes

- Makes `rigid_body` the default `py-demos` and `py-demo-capture` front door,
  with solver/material controls, reset, contact, energy, height, speed,
  step-timing, force-drag, and replay-control diagnostics.
- Moves the curated World rigid-body verifier block to the front of the Python
  demo registry and documents the row order in `python/examples/demos/README.md`.
- Prefixes the interactive `Demos` navigator titles for the 36-row World Rigid
  Body workflow with their position and role while keeping `py-demos --list`
  titles and ids stable for scripts.
- Adds a runner-owned `Rigid Workflow` panel to numbered rigid rows with the
  maintained user question, inspect signals, restart command, selectable
  previous/next route, direct workflow-row selector, and searchable
  row-id/question/checklist/signal filter.
- Adds or upgrades focused visual-verification scenes for:
  - body modes, free flight, frame hierarchy, external loads, link point loads,
    time-step sensitivity, step diagnostics, and contact-scale budget;
  - restitution, material mixing, contact inspection, collision query options
    with persistent ignored-pair diagnostics, collision casts with swept-capsule
    queries, solver comparison, executor equivalence, contact-solver policy,
    multibody-link contact, friction threshold, spin/roll coupling, stack
    stability, contact manipulation, kinematic drivers, and normal-push caveat
    behavior;
  - fixed-joint preservation, AVBD-pinned break-force lifecycle, public
    rigid-body distance springs, one-DOF joints, motor/limit behavior, passive
    joint parameters, screw-joint pitch, generalized multibody dynamics terms,
    link center-of-mass offsets, link-origin Jacobians, multibody solver-family
    routing, and loop-closure families.
- Adds replay-state snapshot coverage for controller-heavy rigid verifier rows.
- Adds opt-in replay timeline metadata for the shared bottom `Replay` panel,
  piloted by `rigid_solver_compare` and `rigid_contact_solver_compare` with
  divergence values plus marker frames while scrubbing saved states.
- Adds related-evidence links from numbered rigid workflow rows to
  non-numbered shelves for broader World `floating_base`/`articulated`
  examples, focused IPC no-tunneling, differentiable contact-gradient
  debugging, and AVBD-specific rigid constraint variants. The `Find row` filter
  indexes those related shelf labels and scene ids, so users can search for the
  target evidence and land on the numbered source row; the search result labels
  the matched related scene and the docs provide docked capture commands for
  every related target.
- Adds a non-numbered `rigid_ipc_stack_packet` Rigid IPC shelf scene for
  capture-first heavier stack evidence with frame-budget, wall-time,
  clearance, contact-count, drift, height-error, speed, and benchmark-owner
  diagnostics.
- Adds scene-owned capture metrics to `py-demo-capture` manifests via
  `SceneSetup.info["capture_metrics"]`, including step-diagnostics,
  contact-scale budget, fixed/one-DOF joint constraint errors, and stack-packet
  physics/runtime fields in `scene_metrics.jsonl` and `manifest.json`. The
  manifest summarizes the full event stream with first/latest events, per-key
  presence counts, and top-level numeric ranges so mid-capture metric dropouts
  are visible.
- Adds drift tests that keep the registry, PLAN-103 sidecar, README quick table,
  and capture commands synchronized.
- Keeps heavy IPC stacks, arm/gripper manipulation, arbitrary-point/contact
  Jacobians, sleep/island/deactivation, and duplicate no-tunneling rows out of
  the numbered workflow until the public API or runtime evidence supports an
  interactive GUI row.

## Testing

- Latest fixed/one-DOF joint capture-metrics follow-up:
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_fixed_joint_verifier_restores_captured_transform python/tests/integration/test_demos_cycle.py::test_rigid_one_dof_joint_verifier_preserves_locked_directions python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
    - `8 passed`
  - `pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_fixed_joint_metrics_1781222114`
    - nonblank 960x540 screenshot, docked-workspace detection, 23 PNG frames,
      24 scene-metrics events, latest translation error
      `2.1673896011265015e-10`
  - `pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_limited_joints_metrics_1781222114`
    - nonblank 960x540 screenshot, docked-workspace detection, 23 PNG frames,
      24 scene-metrics events, latest hinge/slider locked-axis errors `0.0`
  - `pixi run lint`
    - passed
  - `pixi run build`
    - passed
- Latest distance-spring follow-up:
  - `pixi run lint`
    - passed
  - `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_labels_related_evidence_search_matches -q`
    - `17 passed` before and after lint
  - `pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_distance_spring_1781220718`
    - nonblank 960x540 screenshot, docked-workspace detection, 71 PNG frames,
      72 scene-metrics events, and manifest lanes
      `free/soft/stiff/offset`
- Latest post-merge validation on pushed branch `cd7600f8cda`:
  - merged latest `origin/main` into
    `feature/rigid-body-gui-visual-verification`;
  - resolved the `python/tests/integration/test_demos_cycle.py` conflict by
    keeping both the rigid workflow test block and the new AVBD demo test block;
  - `pixi run test-py -- python/tests/integration/test_demos_cycle.py -k "world_rigid_visual_verification_scenes_are_ordered or rigid_visual_verification_sidecar_matches_registry_order or rigid_kinematic_normal_push_exposes_normal_pusher_caveat or avbd_empty_baseline_demo_steps_empty_world or avbd_demo2d_fracture_scene_breaks_and_resets_source_joints or avbd_fixed_joint_contact_demo_exercises_contact_path"`
    - `942 passed, 9 skipped`; the Pixi task forwarded this as a full Python
      suite run after rebuilding the updated native bindings.
  - `pixi run lint`
    - passed
- `pixi run test-py`
  - `942 passed, 9 skipped`; includes the capture manifest evidence guard for
    requested dimensions, converted frame count, screenshot stats, first
    UI-ready-frame stats, the shared replay timeline metadata guard, and the
    stack-packet replay-state contract.
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_registered_world_scenes_receive_shared_replay_controls python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics -q`
  - `2 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_default_timeline_without_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_keeps_frame_marks_for_signal_only_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_ignores_malformed_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_solver_compare_records_wall_response -q`
  - `5 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_solver_compare_records_coupled_contact_policy python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_keeps_frame_marks_for_signal_only_metadata -q`
  - `5 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  - `7 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_labels_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented -q`
  - `6 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_registry_has_scenes python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_ipc_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_stack_packet_panel_exposes_capture_first_signals -q`
  - `12 passed` after lint
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py python/tests/unit/test_py_demo_gpu_toggle.py -q`
  - `25 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_ipc_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_stack_packet_panel_exposes_capture_first_signals -q`
  - `3 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads -q`
  - `2 passed`
- `PYTHONPATH=build/default/cpp/Release-docking/python:build/default/cpp/Release-docking/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_py_demo_capture_records_numbered_rigid_workflow_metrics_artifacts python/tests/unit/test_py_demo_panels.py::test_numbered_rigid_workflow_factory_combines_panels -q`
  - `2 passed`
- `pixi run py-demo-capture -- --scene rigid_step_diagnostics --frames 8 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_step_metrics`
  - nonblank 960x540 screenshot, 7 PNG frames, 8 scene-metrics events,
    manifest lanes `single/contact/stack`
- `pixi run py-demo-capture -- --scene rigid_contact_scale_budget --frames 8 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_budget_metrics`
  - nonblank 960x540 screenshot, 7 PNG frames, 8 scene-metrics events,
    manifest lanes `single/medium/dense`
- `pixi run py-demo-capture -- --scene rigid_ipc_stack_packet --frames 24 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 23 PNG frames, 24 scene-metrics events,
    frame 24 `status=capture-first`, min clearance `0.000267` m, step wall time
    `626.778` ms, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_center_of_mass_offsets_gravity_torque python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  - `17 passed`
- `pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_normal_push_72`
  - nonblank 960x540 screenshot, 71 PNG frames, 72 scene-metrics events,
    frame 72 IPC normal/heavy `status=ipc penetration caveat` with about
    0.125 m depth and SI `status=pushed` with about 0.123 m target travel
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_contact_solver_compare_records_coupled_contact_policy python/tests/integration/test_demos_cycle.py::test_rigid_link_contact_exercises_multibody_contact_response python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  - `18 passed`
- `pixi run py-demo-capture -- --scene contact --frames 72 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 71 PNG frames, final contacts 0
- `pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 71 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_body_baseline_reports_restartable_first_run_diagnostics python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics python/tests/integration/test_demos_cycle.py::test_rigid_frame_hierarchy_tracks_body_fixed_frame python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_body_panel_resets_baseline_scene python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle -q`
  - `15 passed`
- `pixi run py-demo-capture -- --scene rigid_body --frames 24 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 23 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_frame_hierarchy_tracks_body_fixed_frame python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle -q`
  - `12 passed` before and after lint
- `pixi run py-demo-capture -- --scene rigid_frame_hierarchy --frames 72 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 71 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_runner_list_prints_catalog python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  - `4 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  - `4 passed`
- `pixi run py-demo-capture -- --scene rigid_body --frames 8 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 7 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `8 passed`
- `pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 47 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `10 passed`
- `pixi run py-demo-capture -- --scene rigid_spin_roll_coupling --frames 96 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 95 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  - `13 passed` before and after lint
- `pixi run py-demo-capture -- --scene rigid_body --frames 8 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 7 PNG frames, final contacts 0
- `pixi run py-demo-capture -- --scene rigid_solver_compare --frames 8 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 7 PNG frames, final contacts 0
- `pixi run py-demo-capture -- --scene rigid_loop_closure --frames 8 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 7 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_viewer_titles_are_numbered python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds python/tests/integration/test_demos_cycle.py::test_rigid_spin_roll_coupling_converts_slip_to_roll python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows -q`
  - `14 passed` after lint
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/simulation/test_world.py::test_simulation_collision_query_can_ignore_specific_pairs python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `5 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds -q`
  - `1 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows python/tests/unit/gui/test_gui_scene.py::test_gui_stub_surface_is_backend_hidden python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  - `9 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/gui/test_gui_scene.py::test_gui_stub_surface_is_backend_hidden python/tests/unit/gui/test_gui_scene.py::test_gui_camera_and_run_helpers -q`
  - `2 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_jump_selector_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_skips_non_numbered_world_rows python/tests/unit/gui/test_gui_scene.py::test_gui_stub_surface_is_backend_hidden -q`
  - `7 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_collision_casts_report_nearest_all_and_swept_hits python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls -q`
  - `2 passed`
- `pixi run py-demo-capture -- --scene rigid_collision_casts --frames 48 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 47 PNG frames, final contacts 0
- `pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 24 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 23 PNG frames, final contacts 0
- `pixi run py-demo-capture -- --scene rigid_collision_query_options --frames 8 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_collision_query_metrics`
  - nonblank 960x540 screenshot, 7 PNG frames, 8 scene-metrics events,
    manifest lanes `rigid_rigid/rigid_link/same_links/cross_links`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_stack_stability_keeps_ipc_stack_ordered python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `3 passed`
- `pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui`
  - nonblank 960x540 screenshot, 23 PNG frames, final contacts 0
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_rigid_collision_casts_report_nearest_all_and_swept_hits python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q`
  - `7 passed`
- `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run pytest python/tests/integration/test_demos_cycle.py::test_world_rigid_visual_verification_scenes_are_ordered python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/unit/test_capture_py_demo.py::test_visual_capture_default_scene_matches_py_demos_front_door -q`
  - `5 passed`
- `pixi run lint`
  - passed
- `DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 pixi run build`
  - `ninja: no work to do`
- `git diff --check`
  - clean
- `DART_PARALLEL_JOBS=8 CTEST_PARALLEL_LEVEL=8 CMAKE_BUILD_PARALLEL_LEVEL=8 pixi run -e cuda test-all`
  - all 7 stages passed: lint, build, unit tests, simulation tests, Python
    tests, documentation, and CUDA tests/benchmark smoke
- Visual smoke: every numbered workflow row has a recorded nonblank
  `pixi run py-demo-capture -- --show-ui` run in the PLAN-103 sidecar.

## Breaking Changes

- [x] None

## Related Issues / PRs (backports)

- PLAN-103 examples strategy.
- No backport PR: this is DART 7 `main` example-catalog work.

---

#### Checklist

- [ ] Milestone set: `DART 7.0`
- [x] CHANGELOG.md updated
- [x] Add unit tests for new functionality
- [x] Document new methods and classes: N/A; scene docs and sidecar updated.
- [x] Add Python bindings (dartpy) if applicable: N/A; uses existing dartpy APIs.
