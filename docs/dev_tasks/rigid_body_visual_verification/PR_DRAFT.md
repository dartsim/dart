<!--
PR body seed for the rigid-body visual verification branch. Keep the durable
source of truth in
docs/plans/103-examples-strategy/rigid-body-visual-verification.md,
python/examples/demos/README.md, and the generated capture manifests. Use
docs/dev_tasks/rigid_body_visual_verification/COMPLETION_AUDIT.md for the
current evidence-to-requirement map and remaining external gates.
-->

## Summary

- Reworks the `py-demos` rigid-body path into a curated DART 7 visual
  verification workflow for maintainers and users.
- Adds the maintained World Rigid Body sequence, comparison-focused panels,
  replay/debug metadata, capture packet generation, and review-index artifacts
  for visual inspection.
- Keeps unsupported direct rigid-body impulse, sleep/wake or island activation,
  and loop-closure compliance rows explicitly deferred until public `dartpy`
  APIs exist.

## Motivation / Problem

DART 7 introduces new rigid dynamics, solver families, compute/executor
controls, and contact/constraint behavior in parallel. Users need a
first-screen GUI path that answers practical debugging questions rather than a
flat scene catalog. This branch makes the Python GUI examples the maintained
rigid-body visual verification surface for the current cycle.

## Changes / Key Changes

- Curates the first 36 World Rigid Body rows around user questions, solver or
  parameter comparisons, held-fixed context, scope caveats, and per-row
  guidance.
- Adds optional extended rows for related evidence, direct Rigid IPC shelf
  routes, and capture-first IPC stress packets; selecting all optional groups
  now produces a 53-row review packet after the pre-contact surrogate related
  route is included.
- Adds or revises rigid scenes for baseline behavior, modes, free flight,
  frame hierarchy, external loads, point loads, timestep sensitivity,
  diagnostics, contact budget, material response, query/cast inspection, solver
  comparison, executors, multibody contact, friction/spin/stack behavior,
  kinematic drivers, rigid constraints, passive joint parameters, screw joints,
  generalized dynamics terms, COM offsets, link Jacobians, multibody solver
  family routing, and loop-closure family selection.
- Adds `diff_pre_contact_surrogate` as a Differentiable related-evidence route
  for `ContactGradientMode.PRE_CONTACT_SURROGATE`, proving the
  approaching-but-not-touching backward-only signal without expanding the
  numbered rigid workflow.
- Exposes the row-26 fixed-joint breakage threshold as a log-scale GUI
  parameter backed by public `Joint.break_force`, with capture metrics and
  replay state recording the active threshold.
- Exposes row-27 rigid-body distance-spring retuning through narrow public
  `World`/dartpy accessors and GUI sliders for rest length plus soft, stiff,
  and off-center stiffness controls.
- Tightens the row-2 body-mode verifier so the first early workflow comparison
  row now publishes the same `comparison_axis` and held-fixed context pattern
  used by later rigid workflow rows.
- Exposes the row-30 passive-joint hold-force and armature-drive-force controls
  in the GUI, matching the existing capture/replay state for the stiction and
  direct-versus-armature lanes.
- Extends the workflow panel and capture helper with open-live commands,
  row-range packet commands, video packet commands, guidance completeness,
  failure summaries, scene-metrics completeness enforcement, validated
  solver-identity completeness enforcement, replay metadata, latest-signal
  summaries, top-level command provenance, and static review-index links.
- Fixes Linux headless Filament engine creation to use Filament's anonymous
  soft CircularBuffer fallback, avoiding `/tmp` tmpfs-quota `SIGBUS` crashes
  during native capture startup.
- Documents the workflow in the durable PLAN-103 sidecar and the py-demos
  README, including a formal `WP-103.1` work-packet record; dev-task notes stay
  only for active handoff and cleanup.
- Updates the DART 7 changelog with the rigid workflow, capture packet,
  review-index, replay metadata, related-evidence, and scene additions.

## Testing

- `pixi run lint`
- Current-head focused docs/capture drift guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks -q`
  reported `6 passed`.
- Focused editable break-force threshold guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_edits_break_force_threshold python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage -q`
  reported `4 passed`.
- Focused body-mode comparison-axis guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks -q`
  reported `4 passed`.
- Focused passive-joint force-control guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_joint_passive_parameters_panel_edits_drive_forces python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `5 passed`.
- Focused distance-spring public-parameter guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/unit/simulation/test_world.py::test_simulation_api_exposes_python_names_only python/tests/unit/simulation/test_world.py::test_simulation_stub_tracks_public_runtime_symbols python/tests/unit/simulation/test_world.py::test_simulation_world_rigid_body_distance_spring_reduces_stretch python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_distance_spring_panel_edits_public_spring_parameters python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `8 passed`.
- Adjacent row-27 workflow-doc guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `3 passed`.
- API boundary inventory guard:
  `pixi run check-api-boundary-inventory` reported the inventory is up to
  date.
- Adjacent rigid workflow docs consistency guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks -q`
  reported `3 passed`.
- Fresh row-2 single-scene visual evidence:
  `build/captures/rigid_body_modes_comparison_axis_1781325323` captured
  `rigid_body_modes` at 72 requested UI frames, wrote 71 PNG frames plus
  `rigid_body_modes.png`, and recorded `scene_metrics.event_count=72` with
  `comparison_axis=rigid_body_mode_semantics` plus held-fixed context.
- Fresh row-26 single-scene visual evidence:
  `build/captures/rigid_joint_breakage_editable_threshold_1781324921`
  captured `rigid_joint_breakage` at 48 requested UI frames, wrote 47 PNG
  frames plus `rigid_joint_breakage.png`, and recorded
  `scene_metrics.event_count=48` with editable-threshold workflow guidance.
- Fresh row-30 single-scene visual evidence:
  `build/captures/rigid_joint_passive_controls_1781325827` captured
  `rigid_joint_passive_parameters` at 120 requested UI frames, wrote 119 PNG
  frames plus `rigid_joint_passive_parameters.png`, and recorded
  `scene_metrics.event_count=120` with default `hold_force=3.0` and
  `armature_force=6.0` controls in latest scene metrics.
- Fresh row-27 single-scene visual evidence:
  `build/captures/rigid_distance_spring_controls_1781326877` captured
  `rigid_distance_spring` at 72 requested UI frames, wrote 71 PNG frames plus
  `rigid_distance_spring.png`, and recorded `scene_metrics.event_count=72`
  with default `rest_length=0.45`, `soft_stiffness=45.0`,
  `stiff_stiffness=220.0`, and `offset_stiffness=120.0` controls in latest
  scene metrics.
- Current-head broad default validation:
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run test-all`
  passed all wrapper gates: linting, build, unit tests, simulation tests,
  Python tests, and documentation.
- Earlier branch-level CUDA validation on a visible
  `NVIDIA GeForce RTX 4080 Laptop GPU` host:
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run -e cuda test-all`
  exited successfully. This predates the latest continuation commits.
- Focused docs/API drift guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_deferred_api_gaps_are_documented -q`
- Focused scene-metrics/docs guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_routes_publish_self_describing_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count -q`
  reported `5 passed`.
- Focused capture/review-index guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_request_video_commands python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_links_resolve_workspace_relative_artifacts python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count -q`
- Focused workflow evidence enforcement guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_image_evidence python/tests/unit/test_capture_py_demo.py::test_resolved_solver_identity_requires_solver_family_and_context python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_warns_when_scene_metrics_are_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_warns_when_solver_identity_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_links_resolve_workspace_relative_artifacts python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_can_continue_after_scene_failure python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row -q`
  reported `11 passed`.
- DART 7 harness solver-identity guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_image_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/integration/test_demos_cycle.py::test_rigid_visual_routes_publish_self_describing_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_collision_query_options_filter_body_kinds -q`
  reported `4 passed`. A full GUI packet regeneration attempt at
  `build/captures/rigid_workflow_rows_01_36_1781311276` failed on row 1 with
  `return_code=-7` before scene metrics were written. The underlying native
  `SIGBUS` was traced to Filament's Linux `/tmp`-backed CircularBuffer path
  under tmpfs user-quota pressure. After the scoped headless engine-creation
  fix, direct `py-demos` with `/dev/shm` output returned `RC=0`, single-scene
  `py-demo-capture` with `/dev/shm` output returned `RC=0`, and the full plus
  optional workflow packets below regenerated with complete solver identity.
  The failed `1781311276` refresh path is not included in the evidence below.
- Full numbered workflow packet:
  `build/captures/rigid_workflow_rows_01_36_1781323428`
  (`status=complete`, `capture_count=36`, `failed_count=0`,
  `guidance_complete=true`, `scene_metrics_complete=true`,
  `scene_metrics_count=36`, `resolved_solver_identity_complete=true`,
  `resolved_solver_identity_count=36`, 2388 PNG frames).
- Current optional rows 37-53 packet after `diff_pre_contact_surrogate` was
  added:
  `build/captures/rigid_workflow_optional_rows_37_53_1781321474`
  (`status=complete`, `capture_count=17`, `failed_count=0`,
  `guidance_complete=true`, `scene_metrics_complete=true`,
  `resolved_solver_identity_complete=true`,
  `resolved_solver_identity_count=17`, 1027 PNG frames).
- Read-only per-scene metrics audit: both current packet directories have
  latest scene metrics for every captured row (36/36 and 17/17).
- Static review-index asset audit: 0 missing local assets for both current
  review indexes (181/181 links for rows 01-36, 86/86 links for rows 37-53).
- Historical stopped broad validation attempt, superseded by the green broad
  runs above: after a first wrapper attempt was interrupted during Release test
  build, direct
  `pixi run build-tests ON Release` passed. A later
  `timeout 7200s pixi run test-all --skip-lint --skip-build` attempt restarted
  the full wrapper because the pixi task did not forward the flags; it was
  terminated on user request during simulation-labeled ctest after visible
  progress showed 219/219 C++ unit tests passed and simulation output through
  52/65 visible tests. CUDA validation was not run after the stop instruction.
- Completion audit:
  `docs/dev_tasks/rigid_body_visual_verification/COMPLETION_AUDIT.md`
- AI principle audit: recorded in
  `docs/dev_tasks/rigid_body_visual_verification/COMPLETION_AUDIT.md`

## Breaking Changes

- [x] None

## Related Issues / PRs

- Follows the DART 7 architecture/work-packet harness from PR #2986.
- Target milestone: DART 7.0.

---

#### Checklist

- [ ] Milestone set to DART 7.0
- [x] CHANGELOG.md updated if required
- [x] Unit/integration tests added or updated for workflow behavior
- [x] User-facing py-demos documentation updated
- [x] Python GUI/capture workflow updated
- [ ] Maintainer accepts the current 36-row workflow plus optional 53-row
      packet as the completed scope for this dev task
