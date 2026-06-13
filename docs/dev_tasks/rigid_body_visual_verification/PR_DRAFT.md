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
- Covers public direct rigid-body impulse behavior in row 5 while keeping
  unsupported sleep/wake, island activation, and loop-closure compliance rows
  explicitly deferred until public `dartpy` APIs exist, with search tooltips
  and static review cards marking those nearest-row routes as deferred API
  caveats.

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
- Covers public direct rigid-body impulse behavior in `rigid_external_loads`
  via C++/dartpy `RigidBody.apply_linear_impulse()` and
  `RigidBody.apply_angular_impulse()` APIs, GUI sliders, capture metrics, and
  workflow search routing for `direct rigid body impulse`.
- Marks deferred public-API search routes such as `sleep wake`,
  `island activation`, and `loop closure compliance` with tooltip-level and
  review-card deferred API caveats before opening or reviewing the closest
  current verifier row; workflow manifests and review indexes also summarize
  those caveats at packet level.
- Promotes rows 1 and 3 first-screen baseline/free-flight metrics into
  workflow review-card latest signals, so the default front door and
  contact-free initial-state row show speed, height, energy, contact,
  step-time, path-error, momentum, energy-drift, and spin-ratio evidence
  without opening raw manifests.
- Promotes rows 2 and 4-6 core body/frame/load metrics into workflow
  review-card latest signals, so body modes, frame hierarchy, external loads,
  and link point loads are statically reviewable without opening raw manifests.
- Exposes the row-30 passive-joint hold-force and armature-drive-force controls
  in the GUI, matching the existing capture/replay state for the stiction and
  direct-versus-armature lanes.
- Tightens the rows 28-34 panel contract so the perturbation, motor/limit,
  screw-pitch, dynamics-term, COM, and Jacobian slider controls stay covered,
  including a row-29 guard that proves GUI edits reach runtime joint command,
  limit, force, effort-cap, and capture-control state.
- Tightens the rows 35-36 panel contract and row-36 executor behavior: the
  final multibody solver-family and loop-closure rows now have explicit
  executor/gravity panel-edit guards, and loop closure resets/clears replay on
  executor-only edits while showing the active executor in the panel.
- Normalizes executor-only panel edits across the broader rigid GUI workflow:
  executor changes now reset the visual run for 24 executor-select panels, and
  rows that previously omitted it now publish `controls.executor_index` in
  capture metrics so review packets identify the selected executor.
- Adds the matching solver-selector guard for all seven current solver-select
  rigid panels; solver-only edits reset the visual run, and the baseline
  `rigid_body` row now publishes `controls.solver_index` in capture metrics.
- Adds a row-13 query-options control guard: `rigid_collision_query_options`
  now records its body-kind toggles and ignored-pair selector under
  `controls` in capture metrics, matching replay state.
- Adds optional Rigid IPC shelf and capture-first packet control metadata:
  direct shelf rows record `controls.friction`; stack packet variants record
  `controls.friction` and `controls.frame_budget_ms`, and reset records a
  fresh initial metrics sample.
- Adds non-numbered World related-shelf control metadata: `floating_base`
  records `controls.spin_command`, and `articulated` records
  `controls.shoulder_damping` plus `controls.wrist_damping` from live joint
  state.
- Separates backend-status workflow search from executor-equivalence search:
  backend terms such as `compute backend`, `backend comparison`, and
  `parallel backend`, plus GPU/CUDA shorthand, route to
  `rigid_step_diagnostics`, while executor terms such as `compute executor`,
  `executor comparison`, and `Taskflow executor` route to the same-solver
  `rigid_executor_equivalence` row.
- Adds workflow-search aliases for practical multibody and parameter wording:
  `semi implicit` / `semi-implicit` / `variational solver` route to
  `rigid_multibody_solver_family`, while `joint damping`, `joint friction`,
  `joint stiffness`, and `joint armature` route to
  `rigid_joint_passive_parameters`.
- Adds broader workflow-search aliases for user terminology and public API
  spellings: performance terms such as `perf`, `fps`, `throughput`, and
  `contact count`; material/contact terms such as `static friction`,
  `friction coefficient`, and `coefficient of restitution`; query terms such
  as `raycast`, `time-of-impact`, and `CollisionQueryOptions`; dynamics terms
  such as `mass_matrix`, `compute_inverse_dynamics`, and
  `compute_impulse_response`; direct `RigidBody.apply_*` load/impulse API
  terms; stack-jitter/resting-stack wording; and closed-chain/closed-loop
  wording.
- Normalizes workflow search across punctuation, underscores, hyphens, dotted
  API names, CamelCase, compact API tokens, and simple plurals, so
  `RigidBody.applyLinearImpulse`, `Multibody.computeImpulseResponse`,
  `ray-cast`, `shape-cast`, `body kind filters`, and `resting contacts` route
  like their documented snake_case or singular forms.
- Adds front-door `Workflow phase` and `Focus axis` fields to every numbered
  workflow row, grouping the route into foundations, diagnostics,
  contact/material/query basics, solver decisions, contact behavior, rigid
  constraints, and multibody kinematics while naming each row's main comparison
  or debugging dimension; both fields are searchable.
- Exports numbered-row workflow phase and focus-axis metadata into
  single-scene `workflow_guidance`, workflow manifests, and static review
  cards, and treats missing phase/focus metadata as a guidance-completeness
  failure for numbered rows.
- Adds a manifest-level `workflow_phase_summary` and a review-index
  `Workflow Phase Map`, so full and row-range packets show selected numbered
  row ranges, phase counts, per-phase status, focus axes, and scene ids before
  reviewers open individual cards.
- Adds search-result tooltip explanations for maintained aliases, row numbers,
  user questions, related evidence, and scope caveats, plus no-result guidance
  that suggests row numbers, scene ids, solver, contact, backend, or API names.
- Adds a visible search-count summary for broad workflow queries, so capped
  lists show the top six rows plus the full ranked match count before users
  choose a row.
- Promotes row-8 nested step-profile lane metrics into workflow review cards as
  `backend diagnostics`, so backend activity, accelerated-stage counts, worker
  counts, top stages, and stage timings are visible from the static review
  packet.
- Promotes rows 12-14 contact-query/cast metrics into workflow review-card
  latest signals, so selected contact counts/depths, query-filter counts,
  ignored-pair state, ray hit fractions, swept-probe times of impact, and cast
  margins are visible from the static review packet.
- Promotes optional rows 37-53 into workflow review-card latest signals, so
  World related, focused Rigid IPC, direct IPC shelf, capture-first stack,
  differentiable contact-gradient, and AVBD related rows show their actual
  health metrics instead of only generic solver/executor/contact metadata.
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
- Focused executor-switch panel guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_executor_panel_edits_reset_visual_runs -q`
  reported `24 passed`.
- Focused solver-switch panel guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_solver_panel_edits_reset_visual_runs -q`
  reported `7 passed`.
- Focused row-13 query-option control guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_collision_query_options_panel_edits_capture_controls -q`
  reported `1 passed`.
- Focused Rigid IPC shelf/packet control guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_shelf_panel_edits_capture_controls python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_stack_packet_panel_exposes_capture_first_signals python/tests/integration/test_demos_cycle.py::test_rigid_ipc_shelf_scenes_report_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics -q`
  reported `7 passed`.
- Focused World related-shelf control guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_world_related_shelf_panel_edits_capture_controls python/tests/integration/test_demos_cycle.py::test_world_related_evidence_routes_report_capture_metrics -q`
  reported `2 passed`.
- Focused backend/executor workflow-search guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats -q`
  reported `2 passed`.
- Focused search-alias/docs guard for GPU/CUDA backend, multibody solver, and
  passive-joint parameter wording:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `3 passed`.
- Focused search-alias/docs guard for broader user terminology and public API
  spellings:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_empty_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_user_terminology_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_routes_deferred_api_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `11 passed`.
- Focused backend-diagnostics review-card guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_summarizes_backend_diagnostics python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests -q`
  reported `2 passed`.
- Focused contact-query/cast latest-signal guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_query_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_cast_values -q`
  reported `2 passed`.
- Focused core body/frame/load latest-signal guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_body_frame_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_load_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_card_summarizes_link_point_load_signals python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_parameter_budget_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_restitution_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_material_response_values -q`
  reported `6 passed`.
- Focused baseline/free-flight latest-signal guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_baseline_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_free_flight_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_body_frame_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_query_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_cast_values python/tests/integration/test_demos_cycle.py::test_rigid_body_baseline_reports_restartable_first_run_diagnostics -q`
  reported `6 passed`.
- Focused optional rows latest-signal guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_optional_related_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_optional_ipc_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_capture_first_stack_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_optional_joint_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_optional_diff_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_baseline_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_free_flight_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_query_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_cast_values -q`
  reported `9 passed`.
- Adjacent executor-switch panel/replay/docs guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_executor_panel_edits_reset_visual_runs python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `23 passed`.
- Focused row-29 and adjacent panel-control guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_joint_motor_limits_panel_edits_public_limits -q`
  reported `2 passed`.
- Adjacent row-29 runtime/replay guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort -q`
  reported `2 passed`.
- Focused rows 35-36 panel-control guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_multibody_solver_family_panel_edits_execution_controls python/tests/unit/test_py_demo_panels.py::test_rigid_loop_closure_panel_edits_execution_controls -q`
  reported `3 passed`.
- Adjacent rows 35-36 behavior guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families -q`
  reported `2 passed`.
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
- Fresh row-8 single-row workflow evidence:
  `build/captures/rigid_workflow_backend_diagnostics_1781335057` captured
  `rigid_step_diagnostics` at 72 requested UI frames, wrote 71 PNG frames plus
  `rigid_step_diagnostics.png`, reported `scene_metrics_complete=true` and
  `resolved_solver_identity_complete=true`, and generated a review card with
  the new `backend diagnostics` field.
- Fresh rows 12-14 workflow evidence:
  `build/captures/rigid_workflow_query_signal_highlights_1781335507`
  captured `rigid_contact_inspector`, `rigid_collision_query_options`, and
  `rigid_collision_casts`, reported `status=complete`,
  `scene_metrics_complete=true`, `resolved_solver_identity_complete=true`, and
  generated review cards with the promoted contact-query/cast latest signals.
- Fresh rows 2-6 workflow evidence:
  `build/captures/rigid_workflow_core_signal_highlights_1781337019` captured
  the core body/frame/load rows after baseline, reported `status=complete`,
  `capture_count=5`, `failed_count=0`, `scene_metrics_complete=true`,
  `resolved_solver_identity_complete=true`, and generated review cards with
  the promoted body mode, frame hierarchy, external-load, and point-load latest
  signals.
- Fresh rows 1-3 workflow evidence:
  `build/captures/rigid_workflow_baseline_freeflight_signals_1781337861`
  captured the baseline/body-mode/free-flight front-door rows, reported
  `status=complete`, `capture_count=3`, `failed_count=0`,
  `scene_metrics_complete=true`, `resolved_solver_identity_complete=true`, and
  generated review cards with the promoted baseline and free-flight latest
  signals.
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
- Latest broad default validation for pushed commit `fbbd5de0005`:
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run test-all`
  passed all wrapper gates: linting, build, unit tests, simulation tests,
  Python tests, and documentation.
- Latest branch-level CUDA validation for pushed commit `fbbd5de0005`, on a
  visible `NVIDIA GeForce RTX 4080 Laptop GPU` host:
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run -e cuda test-all`
  passed all seven wrapper gates, including CUDA runtime smoke tests and
  benchmark smoke.
- Focused docs/API drift guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_deferred_api_gaps_are_documented -q`
- Focused panel/search-count/docs guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_summarizes_limited_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_empty_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_user_terminology_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_routes_deferred_api_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `12 passed`.
- Focused workflow focus-axis guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_focus_axis_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_focus_axis_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_summarizes_limited_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases -q`
  reported `6 passed`; after docs updates, the broader panel/search/docs-order
  guard reported `15 passed`.
- Focused workflow phase guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_workflow_phase_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_workflow_phase_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats -q`
  reported `4 passed`.
- Broader workflow phase/focus/search/docs-order guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_workflow_phase_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_workflow_phase_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_focus_axis_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_focus_axis_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_summarizes_limited_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_empty_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_user_terminology_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_routes_deferred_api_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
  reported `17 passed`.
- Focused workflow phase/focus manifest and review-card guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_image_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_manifest_reports_missing_guidance -q`
  reported `4 passed`.
- Focused workflow phase-map manifest/review-index guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests -q`
  reported `3 passed`; the follow-up guard covering per-phase status and
  focus-axis display reported `4 passed`.
- Row-15 workflow phase/focus dry-run:
  `DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_phase_review_dry_run_current`
  reported `capture_count=1`, `workflow_total_count=36`, selected row 15,
  `workflow_label=Solver family`, `workflow_phase=4. Solver decision path`,
  `focus_axis=rigid-body solver family`, and review-card `<dt>phase</dt>` /
  `<dt>focus axis</dt>` entries.
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
  `build/captures/rigid_workflow_rows_01_36_1781335894`
  (`status=complete`, `capture_count=36`, `failed_count=0`,
  `guidance_complete=true`, `scene_metrics_complete=true`,
  `scene_metrics_count=36`, `resolved_solver_identity_complete=true`,
  `resolved_solver_identity_count=36`, 2388 PNG frames; `review_index.html`
  includes the latest backend-diagnostics and contact-query/collision-cast
  review-card summaries).
- Current optional rows 37-53 packet after the optional latest-signal refresh:
  `build/captures/rigid_workflow_optional_signal_highlights_1781338541`
  (`status=complete`, `capture_count=17`, `failed_count=0`,
  `guidance_complete=true`, `scene_metrics_complete=true`,
  `resolved_solver_identity_complete=true`,
  `resolved_solver_identity_count=17`; `review_index.html` shows promoted
  optional health signals and passed an 86/86 local-link audit).
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

## Related Issues / PRs (backports)

- Follows the DART 7 architecture/work-packet harness from PR #2986.
- Target milestone: DART 7.0 (`gh` milestone number 84 in the current
  repository milestone list).
- Backports: not applicable; this is DART 7 `main` work.

---

#### Checklist

- [ ] Milestone set to DART 7.0
- [x] CHANGELOG.md updated if required
- [x] Unit/integration tests added or updated for workflow behavior
- [x] Document new methods and classes where applicable
- [x] Add Python bindings if applicable
- [x] User-facing py-demos documentation updated
- [ ] Maintainer accepts the current 36-row workflow plus optional 53-row
      packet as the completed scope for this dev task
