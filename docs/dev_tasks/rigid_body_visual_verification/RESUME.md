# Resume: Rigid Body Visual Verification

## Current Handoff (2026-06-13)

Current branch snapshot:
The active branch is `feature/rigid-body-gui-visual-verification`. This
handoff starts from pushed checkpoint
`765376c93960 Summarize deferred API caveats in review index`, and
`origin/feature/rigid-body-gui-visual-verification` matched local `HEAD` after
the latest user-approved push. Fresh sessions should verify the exact local
and remote state with `git status -sb` and `git log -5 --oneline`; the latest
resume check also confirmed `origin/main` is an ancestor of `HEAD`. There is
still no PR for this branch. Do not push new commits, create a PR, set a
milestone, rerun CI, comment on reviews, resolve threads, or mutate any other
GitHub state without explicit maintainer/user approval.

Latest pushed continuation: deferred public-API searches now explain themselves
in both the live selector and static review packets. `sleep wake` and
`island activation` still route to the body-mode semantics row, while
`loop closure compliance` still routes to the loop-closure family row, but
their result tooltips, guide data, workflow manifests, and `review_index.html`
cards now show a `Deferred API caveat` note before opening or reviewing the
closest current verifier row. Workflow manifests also publish
`deferred_api_caveat_summary`, and the review index shows a top-level
`Deferred API Caveats` table and badge so maintainers can see unsupported API
routes without opening each card. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_deferred_api_caveats python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_deferred_api_search_matches python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_deferred_api_gaps_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `7 passed`. This slice is pushed at
`765376c93960 Summarize deferred API caveats in review index`.

Previous local continuation: workflow manifests now record
`workflow_phase_summary` for selected numbered rows, and `review_index.html`
shows a top-level `Workflow Phase Map` with row ranges, phase counts,
per-phase status, focus axes, and scene ids before the per-row cards. This
makes full and row-range static review packets easier to scan after the
phase/focus metadata export. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests -q`
reported `3 passed`; the follow-up guard covering per-phase status and
focus-axis display reported `4 passed`. This slice is included in the current
branch history; verify remote state before resuming.

Previous local continuation: numbered workflow captures now export
`workflow_phase` and `focus_axis` into single-scene `workflow_guidance`,
workflow manifests, and static review cards, matching the live `Rigid Workflow`
panel. Missing numbered-row phase or focus metadata now fails the workflow
guidance completeness audit. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_image_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_manifest_reports_missing_guidance -q`
reported `4 passed`. Dry-run evidence:
`DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_phase_review_dry_run_current`
reported `capture_count=1`, `workflow_total_count=36`, selected row 15,
`workflow_label=Solver family`, `workflow_phase=4. Solver decision path`,
`focus_axis=rigid-body solver family`, and review-card `<dt>phase</dt>` /
`<dt>focus axis</dt>` entries. This slice is included in the current branch
history; verify remote state before resuming.

Previous local continuation: every numbered `Rigid Workflow` row now exposes a
front-door `Workflow phase` before the focus axis and try/inspect checklist.
The phase text groups rows into foundations, diagnostics,
contact/material/query basics, solver decision path, contact behavior cases,
rigid constraints/joint mechanics, and multibody dynamics/kinematics.
Multi-token phase searches route to the first row of the matching phase and
tooltip reasons report `Search match: workflow phase` without changing broad
single-token query ordering. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_workflow_phase_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_workflow_phase_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats -q`
reported `4 passed`; the broader panel/search/docs-order guard across workflow
phase, focus-axis, capped search, existing aliases, deferred API routes,
related evidence, and README sidecar order reported `17 passed`. This local
slice is now represented in branch history.

Latest pushed continuation: every numbered `Rigid Workflow` row now exposes a
front-door `Focus axis` before the try/inspect checklist. The axis names the
main comparison or debugging dimension, such as solver family, executor-only
behavior, contact solver method, workload shape/backend profile, passive joint
parameter family, or loop-closure family, and focus-axis text participates in
search ranking and tooltip reasons. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_focus_axis_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_focus_axis_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_summarizes_limited_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases -q`
reported `6 passed`; after docs updates, the broader panel/search/docs-order
guard reported `15 passed`. This slice has been pushed at `8f589fc1c99`.

Previous pushed continuation: the in-viewer `Rigid Workflow` filter now counts
the full ranked result set while still showing only the top six rows, so broad
queries such as `contact` display an explicit `Showing 6 of N matching workflow
rows` line before the selectable results. Search-result tooltips still name the
match source, and empty-result feedback still suggests row numbers, scene ids,
solver, contact, backend, or API names. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_summarizes_limited_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_empty_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_user_terminology_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_routes_deferred_api_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `12 passed`. This slice has been pushed at `8801dc28ee1`.

Latest pushed search-discovery continuation: rigid workflow search now covers more terms users
type while diagnosing performance, contact/material parameters, collision
queries, dynamics terms, impulses, stack behavior, and closed chains.
`throughput` routes to `rigid_contact_scale_budget`; `latency` and
`wall time` route to `rigid_step_diagnostics`; `dt`, `substeps`, and
`gravity tuning` route to `rigid_timestep_sensitivity`; friction and
restitution coefficient terms route to the material/restitution rows;
`raycast`, `swept sphere`, and related sweep terms route to
`rigid_collision_casts`; `CollisionQueryOptions` and collision-filter terms
route to `rigid_collision_query_options`; static-friction terms route to
`rigid_friction_threshold`; kinetic/dynamic-friction terms route to
`rigid_spin_roll_coupling`; `generalized force`, `coriolis`, `mass_matrix`,
`compute_inverse_dynamics`, `compute_impulse_response`, and `impulse response`
route to `rigid_multibody_dynamics_terms`; direct `RigidBody` impulse/load API
terms route to the public load/impulse row; `stack jitter` and resting-stack
terms route to `rigid_stack_stability`; and `closed chain` / `closed loop`
terms route to `rigid_loop_closure`. The search now folds punctuation,
underscores, hyphens, dotted API names, CamelCase, compact API tokens, and
simple plurals, so `RigidBody.applyLinearImpulse`,
`Multibody.computeImpulseResponse`, `ray-cast`, `shape-cast`,
`body kind filters`, and `resting contacts` route like their documented
snake_case or singular forms. Search-result tooltips now name whether the row
matched by maintained alias, row number, user question, related evidence, or
scope caveat, and no-result feedback suggests row numbers, scene ids, solver,
contact, backend, or API names. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_empty_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_user_terminology_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_routes_deferred_api_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `11 passed`.

Latest pushed continuation: rigid workflow search now routes GPU/CUDA shorthand
(`GPU`, `CUDA`, `GPU backend`, `CUDA backend`, `GPU acceleration`,
`CUDA acceleration`) to `rigid_step_diagnostics`, where users can inspect
backend status and fallback without implying a separate rigid CUDA solver row.
It also routes multibody solver-family terms (`semi implicit`,
`semi-implicit`, `variational solver`) to `rigid_multibody_solver_family`, and
passive-joint parameter terms (`joint damping`, `joint friction`,
`joint stiffness`, `joint armature`) to `rigid_joint_passive_parameters`.
Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `3 passed`. This slice was pushed at `cb5deee7bfd`.

Latest local continuation: row 5, `rigid_external_loads`, now covers public
direct rigid-body impulse behavior instead of documenting it as a deferred API
gap. `RigidBody` has C++ and dartpy `apply_linear_impulse()` /
`apply_angular_impulse()` surfaces, the scene adds direct linear/angular
impulse lanes and sliders, capture metrics promote impulse momentum/speed, and
PLAN-103 plus the py-demos README now route `direct rigid body impulse` to the
public load/impulse row while keeping sleep/wake, island activation, and
loop-closure compliance deferred. Focused evidence: `pixi run
build-py-dev-docking` rebuilt `dartpy`; the focused pytest set covering the
new impulse API, external-loads scene metrics, API/docs drift guard, panel
search routing, and capture-card load highlights reported `6 passed`; and
`pixi run py-demo-capture -- --scene rigid_external_loads --frames 12 --width
640 --height 360 --show-ui --output-dir
/tmp/dart_capture_rigid_external_loads_impulses_1781345833` wrote a nonempty
640x360 screenshot, 11 PNG frames, 12 scene-metrics events, complete resolved
solver identity (`solver=sequential_impulse`, `executor=Sequential`), and
latest metrics with `scope=external_load_and_direct_impulse_response`,
`linear_impulse_momentum_x=0.8`, `linear_impulse_speed=0.8`,
`angular_impulse_momentum_z=0.09`, `angular_impulse_speed=2.0`, and
`static_drift=0.0`. This slice has been pushed at `fbbd5de0005` and covered by
the default and CUDA wrapper validations recorded below.

Latest local continuation: a fresh DART 7 work-packet dry-run checked the
current 53-row rigid workflow harness shape without rendering:
`DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dryrun_current`
completed successfully. The dry-run manifest reports `status=planned`,
`dry_run=true`, `capture_count=53`, `guidance_complete=true`,
`guidance_missing_count=0`, all requested optional groups enabled, all selected
optional groups enabled, and `review_index.html` shows rows `1-53 / 53` with
requested/selected groups `numbered, related, ipc shelf, packets`.

Latest local continuation: workflow review-card `latest signals` now promote
optional-row health signals instead of collapsing rows 37-53 to generic
solver/executor/contact metadata. The highlighter covers World related rows
(`floating_base`, `articulated`), focused Rigid IPC capability rows, direct IPC
shelf rows, capture-first IPC stack packets, differentiable contact-gradient
routes, and AVBD related joint/motor rows. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_optional_related_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_optional_ipc_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_capture_first_stack_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_optional_joint_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_optional_diff_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_baseline_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_free_flight_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_query_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_cast_values -q`
reported `9 passed`. Fresh optional rows 37-53 workflow evidence:
`build/captures/rigid_workflow_optional_signal_highlights_1781338541`
completed with `status=complete`, `capture_count=17`, `completed_count=17`,
`failed_count=0`, `guidance_complete=true`, `scene_metrics_complete=true`,
`scene_metrics_count=17`, `resolved_solver_identity_complete=true`,
`resolved_solver_identity_count=17`, and `review_index.html` showing promoted
signals such as linear speed, min tunnel margin, capture-first/frame-budget
stack metrics, differentiable fallback metrics, and AVBD target/measured speed.
A static review-index audit found 86/86 local links present.

Latest local continuation: workflow review-card `latest signals` now promote
the first-screen baseline and free-flight row diagnostics. Row 1 surfaces
baseline max speed, minimum height, kinetic energy, contact count, step time,
and dynamic body count directly from capture metrics; row 3 surfaces zero-g
drift error, momentum drift, drift speed, gravity-arc position error, momentum
residual, energy drift, height, and spin momentum ratio. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_baseline_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_free_flight_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_body_frame_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_query_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_cast_values python/tests/integration/test_demos_cycle.py::test_rigid_body_baseline_reports_restartable_first_run_diagnostics -q`
reported `6 passed`. Fresh rows 1-3 workflow evidence:
`build/captures/rigid_workflow_baseline_freeflight_signals_1781337861`
completed with `status=complete`, `capture_count=3`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`resolved_solver_identity_complete=true`, and `review_index.html` showing the
new baseline/free-flight latest signals.

Latest local continuation: workflow review-card `latest signals` now promote
the first core body/frame/load rows that a reviewer scans after the baseline.
Row 2 surfaces dynamic displacement/height/speed, kinematic path error/x, and
static drift; row 4 surfaces world/relative/orientation transform residuals,
sensor position, and parent; row 5 surfaces force- and inertia-scaled
acceleration plus static drift; row 6 surfaces centered/doubled/local-frame
acceleration, off-center yaw acceleration, and pulse count. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_body_frame_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_load_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_card_summarizes_link_point_load_signals python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_parameter_budget_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_restitution_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_material_response_values -q`
reported `6 passed`. Fresh rows 2-6 workflow evidence:
`build/captures/rigid_workflow_core_signal_highlights_1781337019` completed
with `status=complete`, `capture_count=5`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`resolved_solver_identity_complete=true`, and `review_index.html` showing the
new row-specific latest signals.

Latest local continuation: the full numbered rows 01-36 workflow packet was
regenerated after the backend-diagnostics and contact-query review-card
improvements, so the broad maintainer review artifact now matches current
capture-helper behavior. Fresh evidence:
`build/captures/rigid_workflow_rows_01_36_1781356342` completed with
`status=complete`, `capture_count=36`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`resolved_solver_identity_complete=true`, and `review_index.html` now showing
row-8 `backend diagnostics` plus row 12-14 contact-query/collision-cast
`latest signals`. A read-only static review audit checked 181 local
`href`/`src` targets in `review_index.html` with 0 missing links. The
in-viewer resilient packet command now also matches the durable docs:
`/tmp/dart_capture_rigid_workflow_resilient`.

Latest local continuation: workflow review-card `latest signals` now promote
the contact-query and collision-cast values that users need while scanning the
static packet. Rows 12-14 now surface selected/total contact counts, selected
depth, selected pair and shape indices, query-filter active/baseline/filtered
counts, ignored-pair state, ray hit count/fraction, swept sphere/capsule hit
counts, time of impact, and cast margins instead of only solver/executor
metadata. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_query_values python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_latest_signals_prioritize_collision_cast_values -q`
reported `2 passed`. Fresh rows 12-14 workflow evidence:
`build/captures/rigid_workflow_query_signal_highlights_1781335507` completed
with `status=complete`, `capture_count=3`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`resolved_solver_identity_complete=true`, and `review_index.html` showing the
new row-specific latest signals.

Latest local continuation: workflow review cards now promote nested
`rigid_step_diagnostics` lane profile data into a compact
`backend diagnostics` line. The row-8 card names each workload lane's profile
status, accelerated-backend status, accelerated-stage count, max worker count,
top stage, top-stage time, and total stage time, so static review packets show
the backend/fallback/timing story without requiring reviewers to open the raw
manifest JSON. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_summarizes_backend_diagnostics python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests -q`
reported `2 passed`. Fresh single-row workflow evidence:
`build/captures/rigid_workflow_backend_diagnostics_1781335057` captured row 8
with `status=complete`, `capture_count=1`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`resolved_solver_identity_complete=true`, and `review_index.html` containing
the new `backend diagnostics` field.

Latest local continuation: rigid workflow search now separates backend-status
queries from executor-equivalence queries. Backend terms such as
`compute backend`, `backend comparison`, `parallel backend`, and
`backend/executor` route to `rigid_step_diagnostics`, where the panel and
capture metrics expose backend status, acceleration masks, worker count, ECS,
scratch, contact, and profile-stage diagnostics. Executor terms such as
`compute executor`, `executor comparison`, `parallel executor`, and
`Taskflow executor` still route to the same-solver
`rigid_executor_equivalence` row, so users do not mistake an executor timing
comparison for an accelerated-backend comparison. The py-demos README and
PLAN-103 sidecar describe the split. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats -q`
reported `2 passed`. No new visual packet was generated in this slice.

Latest broad validation recorded for this task state: after a local
`origin/main` merge check reported `Already up to date`, pushed commit
`fbbd5de0005` passed both wrapper suites with the DART safe parallelism cap:
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run test-all`
passed all six default wrapper gates, and
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run -e cuda test-all`
passed all seven CUDA wrapper gates on an `NVIDIA GeForce RTX 4080 Laptop GPU`
host, including the CUDA smoke tests and benchmark smoke. Documentation built
with the known generated-stub warnings for `dartpy._world_render_bridge`, and
both wrappers ended green.

Latest artifact refresh: after the backend/executor search-routing,
backend-diagnostics, contact-query review-card, optional-signal,
phase/focus-map, deferred-API caveat, and direct-impulse slices, the full rows
01-36 packet remains current at
`build/captures/rigid_workflow_rows_01_36_1781356342` and the optional rows
37-53 packet is current at
`build/captures/rigid_workflow_optional_signal_highlights_1781338541`. Their
manifests report complete scene metrics, complete resolved solver identity,
zero failed rows, and the static review-index asset audit found 181/181 and
86/86 local links present respectively.

Previous pushed continuation: non-numbered World related shelf rows now publish
their user-editable controls in capture metrics. `floating_base` records
`controls.spin_command` from the live floating-joint velocity, and
`articulated` records `controls.shoulder_damping` plus
`controls.wrist_damping` from the live two-link arm joints. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_world_related_shelf_panel_edits_capture_controls python/tests/integration/test_demos_cycle.py::test_world_related_evidence_routes_report_capture_metrics -q`
reported `2 passed`. No new visual packet was generated in this slice.

Previous local continuation: optional Rigid IPC shelf and capture-first packet
rows now publish their user-editable controls in capture metrics. The direct
Rigid IPC shelf rows (`rigid_ipc`, `rigid_ipc_slide`, `rigid_ipc_incline`, and
`rigid_ipc_pile`) record `controls.friction`, and the capture-first stack
packet variants record `controls.friction` plus `controls.frame_budget_ms`.
`rigid_ipc_stack_packet` also records a fresh initial metrics sample after a
friction-triggered reset, so the panel/controller state stays self-describing.
Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_shelf_panel_edits_capture_controls python/tests/unit/test_py_demo_panels.py::test_rigid_ipc_stack_packet_panel_exposes_capture_first_signals python/tests/integration/test_demos_cycle.py::test_rigid_ipc_shelf_scenes_report_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_ipc_stack_packet_reports_capture_first_metrics -q`
reported `7 passed`. No new visual packet was generated in this slice.

Previous local continuation: row 13, `rigid_collision_query_options`, now
publishes a proper capture-metrics `controls` block for its query toggles and
ignored-pair selector. This matches the row's replay state and makes review
packets identify the active `World.collide(options)` body-kind filter instead
of only reporting derived contact counts. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_collision_query_options_panel_edits_capture_controls -q`
reported `1 passed`. No new visual packet was generated in this slice.

Previous local continuation: solver-select rigid rows now have the same
run-defining control guard as executor-select rows. The focused panel test
covers all seven current solver-selector panels (`rigid_body`,
`rigid_body_modes`, `rigid_timestep_sensitivity`, `rigid_step_diagnostics`,
`rigid_contact_scale_budget`, `rigid_executor_equivalence`, and
`rigid_restitution_ladder`) and proves solver-only edits reset the visual run
and publish `controls.solver_index` in capture metrics. The baseline
`rigid_body` row now records that solver control in capture metrics, matching
its replay state. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_solver_panel_edits_reset_visual_runs -q`
reported `7 passed`. No new visual packet was generated in this slice.

Latest pushed continuation: the executor-switch guard now covers all audited
executor-select rigid rows that are safe to construct without loop-closure
feature gating. Twenty-four panels, including the originally missed friction
threshold, spin/roll coupling, and stack-stability rows, reset the visual run on
executor-only edits and publish `controls.executor_index` in capture metrics.
Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_executor_panel_edits_reset_visual_runs -q`
reported `24 passed`. No new visual packet was generated in this slice.

Previous local continuation: executor-driven rigid GUI panels now treat
`Executor` as a run-defining visual-control edit. Nineteen panels, including
body modes, free flight, frame hierarchy, timestep/diagnostics/contact-budget,
solver/contact comparisons, link contact, manipulation, kinematic drivers,
external/point loads, distance springs, passive joints, and screw pitch, reset
the visual run on executor-only edits instead of mixing old timelines with the
new executor selection. Capture metrics also record `controls.executor_index`
for the rows that previously omitted it. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_executor_panel_edits_reset_visual_runs -q`
reported `19 passed`; the adjacent panel/replay/docs guard
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_executor_panel_edits_reset_visual_runs python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `23 passed`. No new visual packet was generated in this slice.

Latest pushed continuation: rows 35-36 now close the editable-control guard for
the numbered World Rigid Body workflow. Row 35,
`rigid_multibody_solver_family`, and row 36, `rigid_loop_closure`, have panel
guards for their gravity controls and focused panel-edit tests proving executor
and gravity changes update runtime state, reset simulation time, and publish
capture controls. Row 36 also now resets/clears replay on executor-only edits,
matching the other executor-driven verifier rows, and its panel shows the active
executor. Focused guards:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_multibody_solver_family_panel_edits_execution_controls python/tests/unit/test_py_demo_panels.py::test_rigid_loop_closure_panel_edits_execution_controls -q`
reported `3 passed`; the adjacent behavior guard
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families -q`
reported `2 passed`. No new visual packet was generated in this slice.

Latest local continuation: rows 28-34 now have a stronger panel-contract guard
for the editable controls that users rely on in the later joint and multibody
workflow rows. The comparison-panel test now asserts the Perturbation,
motor/limit, screw-pitch, dynamics-term, COM, and Jacobian slider labels and
ranges. Row 29, `rigid_joint_motor_limits`, also has a focused panel-edit
guard proving the GUI sliders update the controller, joint command velocity,
velocity limits, position upper limit, requested force, effort cap, and capture
controls. Focused guards:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_joint_motor_limits_panel_edits_public_limits -q`
reported `2 passed`; the adjacent runtime/replay guard
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort -q`
reported `2 passed`. No new capture packet was generated in this slice.

Latest local continuation: row 27, `rigid_distance_spring`, now exposes the
public distance-spring parameters promised by the workflow. `World` has narrow
`has/get/set` accessors for named rigid-body distance springs, dartpy exposes
the snake_case API, and the GUI retunes existing springs during simulation
mode so users can edit global rest length plus the soft, stiff, and off-center
lane stiffnesses. Focused guards:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/unit/simulation/test_world.py::test_simulation_api_exposes_python_names_only python/tests/unit/simulation/test_world.py::test_simulation_stub_tracks_public_runtime_symbols python/tests/unit/simulation/test_world.py::test_simulation_world_rigid_body_distance_spring_reduces_stretch python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_distance_spring_panel_edits_public_spring_parameters python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `8 passed`; the adjacent docs guard
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `3 passed`; `pixi run check-api-boundary-inventory` reported the
API boundary inventory is up to date. Fresh single-scene visual evidence:
`build/captures/rigid_distance_spring_controls_1781326877` captured
`rigid_distance_spring` at 72 requested UI frames, wrote 71 PNG frames plus
`rigid_distance_spring.png`, recorded `scene_metrics.event_count=72`, and its
latest scene metrics include default controls `rest_length=0.45`,
`soft_stiffness=45.0`, `stiff_stiffness=220.0`, and
`offset_stiffness=120.0`.

Latest local continuation: row 30, `rigid_joint_passive_parameters`, now
exposes the passive-joint force controls that were already modeled in capture
and replay state. The GUI adds `Hold force` for the stiction lane and
`Armature drive force` for the direct-versus-armature lanes, applies them
through the existing reset path, and keeps capture metrics recording
`hold_force` and `armature_force`. Focused guards:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_joint_passive_parameters_panel_edits_drive_forces python/tests/integration/test_demos_cycle.py::test_rigid_verifier_replay_snapshots_restore_controls python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `5 passed`. Fresh single-scene visual evidence:
`build/captures/rigid_joint_passive_controls_1781325827` captured
`rigid_joint_passive_parameters` at 120 requested UI frames, wrote 119 PNG
frames plus `rigid_joint_passive_parameters.png`, recorded
`scene_metrics.event_count=120`, and its latest scene metrics include default
`hold_force=3.0` and `armature_force=6.0` controls under the contact-free
passive-parameter row scope.

Latest local continuation: row 2, `rigid_body_modes`, now follows the
comparison-axis contract used by the later rigid workflow rows. Its panel and
capture metrics expose `comparison_axis=rigid_body_mode_semantics` plus
held-fixed solver/executor/gravity/force/body-mass/time-step context before the
dynamic/static/kinematic lane diagnostics. Focused guards:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/integration/test_demos_cycle.py::test_rigid_body_modes_compare_dynamic_static_kinematic_semantics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks -q`
reported `4 passed`. Fresh single-scene visual evidence:
`build/captures/rigid_body_modes_comparison_axis_1781325323` captured
`rigid_body_modes` at 72 requested UI frames, wrote 71 PNG frames plus
`rigid_body_modes.png`, recorded `scene_metrics.event_count=72`, and its
latest metrics include `comparison_axis=rigid_body_mode_semantics` plus the
held-fixed context.

Latest local continuation: row 26, `rigid_joint_breakage`, now exposes the
public `Joint.break_force` parameter directly through a log-scale
`Break force log10(N)` GUI slider. The row records the active break threshold
in capture metrics and replay state, supports resetting with the current
threshold, and keeps the existing locked-reset and weak-rearm paths. The
workflow scope and PLAN-103 row now describe this as an AVBD-pinned
editable-threshold breakage row rather than a fixed-threshold caveat; it still
makes no sequential-impulse or IPC break-force parity claim. Focused guards:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_edits_break_force_threshold python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage -q`
reported `4 passed`, and the adjacent docs/workflow consistency subset
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_capture_metric_docs_match_hooks -q`
reported `3 passed`. Fresh single-scene visual evidence:
`build/captures/rigid_joint_breakage_editable_threshold_1781324921` captured
`rigid_joint_breakage` at 48 requested frames with UI, wrote 47 PNG frames plus
`rigid_joint_breakage.png`, recorded `scene_metrics.event_count=48`, and its
manifest reports workflow guidance scope
`AVBD-pinned editable-threshold breakage row; no sequential-impulse or IPC
break-force parity claim.`

Remote publication checkpoint: use `git status -sb` as the source of truth for
local/remote parity. The last recorded remote checkpoint before this
validation-language correction was
`6bafd605907 Add pre-contact surrogate visual demo`; later handoff/docs commits
may be present when the branch matches
`origin/feature/rigid-body-gui-visual-verification`. Fresh
`gh pr list --head "$(git branch --show-current)"` and `gh pr status` checks
still reported no PR for the branch. The latest pre-push `git fetch origin main
&& git merge --no-edit origin/main` reported `Already up to date`, so the
branch remained aligned with the PR #2986 DART 7 architecture/work-packet
harness at publication time. The approved push did not approve PR creation,
milestone mutation, CI reruns, review comments, thread resolution, or other
GitHub review-state changes.

Latest local evidence refresh: the optional extended workflow packet was
regenerated after `diff_pre_contact_surrogate` landed in the related-evidence
set. `build/captures/rigid_workflow_optional_signal_highlights_1781338541` selected
rows 37-53 from the fully extended 53-row packet with related evidence, direct
Rigid IPC shelf rows, and capture-first packet groups requested and selected.
Its manifest reports `status=complete`, `capture_count=17`,
`completed_count=17`, `failed_count=0`, `workflow_total_count=53`,
`workflow_row_start=37`, `workflow_row_end=53`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`scene_metrics_count=17`, `resolved_solver_identity_complete=true`,
`resolved_solver_identity_count=17`, `failed_rows=[]`, and 17 captured
manifests. A read-only static review audit checked 86 local `href`/`src`
targets in `review_index.html` with 0 missing links. Use this rows 37-53
packet as current optional maintainer-review evidence; the older rows 37-52
packet is historical.

Latest pushed continuation: the rigid workflow evidence contract is
being tightened to match the DART 7 work-packet harness. Workflow manifests and
review indexes now treat latest scene metrics as required evidence for captured
rows, resolved solver identity must include both solver-family and context
fields, and formerly weak rigid/related rows now publish explicit executor or
not-applicable context markers in their capture metrics. PLAN-103 now records a
formal `WP-103.1` work-packet section with objective, scope, non-goals,
acceptance evidence, gates, dependencies, and evidence pointers. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_image_evidence python/tests/unit/test_capture_py_demo.py::test_resolved_solver_identity_requires_solver_family_and_context python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_warns_when_scene_metrics_are_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_warns_when_solver_identity_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_links_resolve_workspace_relative_artifacts python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_can_continue_after_scene_failure python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row -q`
reported `11 passed`.

Latest local continuation: a real current-head artifact refresh plus read-only
artifact audit rechecked the current review packet manifests and static review
indexes. The full rows 01-36 packet reports `status=complete`,
`capture_count=36`, `failed_count=0`, `guidance_complete=true`,
`scene_metrics_complete=true`, `resolved_solver_identity_complete=true`,
`resolved_solver_identity_count=36`, `deferred_api_caveat_count=2`, 2544 PNG
frames, row-5 direct impulse signals, and 181/181 local review-index assets
present. The current optional rows 37-53 packet reports `status=complete`,
`capture_count=17`, `failed_count=0`, `guidance_complete=true`,
`scene_metrics_complete=true`, `resolved_solver_identity_complete=true`,
`resolved_solver_identity_count=17`, and 86/86 local review-index assets
present.

Latest local harness hardening: workflow review indexes now show a solver
identity badge and a `Rows Missing Solver Identity` warning when any captured
row lacks a resolved solver/contact/executor identity, and non-dry-run
`py-demo-capture -- --rigid-workflow` packets now return failure status instead
of completing green in that state. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_warns_when_solver_identity_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_links_resolve_workspace_relative_artifacts python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_can_continue_after_scene_failure python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row -q`
reported `7 passed`.

Latest publication-readiness audit: `git fetch origin main && git merge
--no-edit origin/main` still reports `Already up to date`; `gh pr status` still
reports no PR for `feature/rigid-body-gui-visual-verification`; the open
milestone list includes `DART 7.0`; and `PR_DRAFT.md` is safe to pass as a body
file because the visible body starts at `## Summary` after the HTML comment and
contains Testing, Breaking Changes, Related Issues / PRs, milestone, and
completion-audit references.

Latest deferred-API audit: runtime `dartpy.simulation` symbol introspection and
targeted binding/stub reads now find public
`RigidBody.apply_linear_impulse()` and `RigidBody.apply_angular_impulse()`
surfaces, so direct rigid-body impulse is covered by row 5
`rigid_external_loads`. The same audit still finds no public sleep/wake,
island activation, or loop-closure compliance/stiffness/damping control, so
those queries remain routed to caveated nearest rows instead of adding
speculative numbered rows.

Fresh focused guard for that audit:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_routes_deferred_api_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_deferred_api_gaps_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `6 passed`.

Latest validation follow-up: after merging latest `origin/main` (`Already up
to date`), broad default validation was rerun from branch
`feature/rigid-body-gui-visual-verification` at pushed commit `fbbd5de0005`
with `DART_SAFE_JOBS=5`. The
default command
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run test-all`
completed successfully: linting, build, unit tests, simulation tests, Python
tests, and documentation all passed. A later CUDA follow-up on a visible RTX
4080 Laptop GPU ran `pixi run -e cuda test-all` and passed all seven wrapper
gates, including CUDA runtime smoke tests and benchmark smoke. These broad
wrapper runs cover the row-5 direct-impulse implementation state at
`fbbd5de0005`; later review-prep/search-discovery commits have focused guards
but have not rerun the full default/CUDA wrappers.
The previous stopped validation attempt remains useful archaeology only; do
not report it as the latest state.

Historical publication state before the later approved pushes: after committing
`84897c2fde5 Record rigid workflow validation evidence`, the local branch was
ahead of `origin/feature/rigid-body-gui-visual-verification`, and
`gh pr list --head "$(git branch --show-current)"` returned no PR. That is now
superseded by the newer remote checkpoints recorded above. Future sessions must
rerun `git status -sb` and must get explicit approval before pushing newer
local commits, creating a PR, setting the `DART 7.0` milestone, rerunning CI, or
mutating GitHub review state.

Stop/push handoff: the newest instruction was to stop code changes, stop
further verification, update only the handoff docs, merge latest `origin/main`,
push to `origin`, and stop. The active broad validation process was terminated
by request. It was running
`timeout 7200s pixi run test-all --skip-lint --skip-build`; because the pixi
task did not forward those flags, it restarted the full `test-all` wrapper.
Visible evidence before termination: lint had passed, Release configure/build
had completed, `pixi run build-tests ON Release` had passed when re-run
directly after an earlier SIGTERM/interruption, C++ unit tests reported
219/219 passed, and simulation-labeled ctest output had reached 52/65 visible
tests. This is not a completed `test-all` result. CUDA validation was not run.
Do not report this stopped run as green; a future session should rerun whatever
validation the maintainer asks for.

Latest merge/push state for the previous stop-only handoff: `git fetch origin
main && git merge --no-edit origin/main` reported `Already up to date`. That
session had explicit approval to push that handoff state to `origin`, but not
to create a PR, set a milestone, rerun CI, mutate review threads, or treat the
dev-task as complete. This docs-only handoff refresh intentionally skipped
additional verification because the user explicitly requested no further
verification in that session. This is superseded by the successful validation
follow-up above.

Latest local follow-up: fetched `origin/main` and merged it again; Git reported
`Already up to date`, so the branch still carries the PR #2986 DART 7
architecture/work-packet harness. The previous native Filament `SIGBUS`
blocker is now understood and fixed for Linux headless engine creation:
Filament's file-backed CircularBuffer path used a `/tmp` ashmem-style mapping
under tmpfs user-quota pressure. During the failing repros, `quota -s` reported
`tmpfs 25573M/25573M`; the current occupancy can drift, so recheck it before
debugging host state. `dart::gui` now scopes a temporary file-size limit around
headless `Engine::Builder().build()` on Linux to force Filament's anonymous
"soft" CircularBuffer fallback, then restores the original limit before
screenshot/frame output. Treat
`build/captures/rigid_workflow_rows_01_36_1781311276` and the earlier `/tmp`
reproducers as failure evidence only.

Latest local follow-up: after the headless engine-creation fix and the DART 7
harness scene-metrics contract update, the full numbered packet regenerated at
`build/captures/rigid_workflow_rows_01_36_1781356342` and completed with
`status=complete`, `capture_count=36`, `completed_count=36`,
`failed_count=0`, `workflow_total_count=36`, `workflow_row_start=1`,
`workflow_row_end=36`, `guidance_complete=true`, `guidance_missing_count=0`,
`scene_metrics_complete=true`, `scene_metrics_count=36`,
`resolved_solver_identity_complete=true`, `resolved_solver_identity_count=36`,
`resolved_solver_identity_missing_count=0`, `failed_rows=[]`, and 2544 frame
PNGs. The current optional rows 37-53 packet
`build/captures/rigid_workflow_optional_signal_highlights_1781338541` completed with
`status=complete`, `capture_count=17`, `completed_count=17`,
`failed_count=0`, `workflow_total_count=53`, `workflow_row_start=37`,
`workflow_row_end=53`, `include_related=true`, `include_ipc_shelf=true`,
`include_packets=true`, `selected_include_related=true`,
`selected_include_ipc_shelf=true`, `selected_include_packets=true`,
`guidance_complete=true`, `guidance_missing_count=0`,
`scene_metrics_complete=true`, `scene_metrics_count=17`,
`resolved_solver_identity_complete=true`, `resolved_solver_identity_count=17`,
`resolved_solver_identity_missing_count=0`, `failed_rows=[]`, and 1027 frame
PNGs. A read-only HTML asset audit found 0 missing local assets in both copied
review indexes: 181/181 links for rows 01-36 and 86/86 links for rows 37-53.
The copied `review_index.html` files use relative asset links; the JSON
manifests preserve the top-level workflow command and per-scene capture
provenance.

Latest local follow-up: regenerated the current-HEAD review packets after the
workflow-command provenance and review-index link-normalization fix. The full
numbered packet
`build/captures/rigid_workflow_rows_01_36_1781309127` completed with
`status=complete`, `capture_count=36`, `completed_count=36`,
`failed_count=0`, `workflow_total_count=36`, `guidance_complete=true`,
`guidance_missing_count=0`, and `failed_rows=[]`. It wrote
`manifest.json`, `review_index.html`, 36 docked row screenshots, and 2544
frame PNGs; the first row was `rigid_body` and the last was
`rigid_loop_closure`. The optional extended packet
`build/captures/rigid_workflow_optional_rows_37_52_1781309448` completed with
`status=complete`, `capture_count=16`, `completed_count=16`,
`failed_count=0`, `workflow_total_count=52`, `workflow_row_start=37`,
`workflow_row_end=52`, `include_related=true`, `include_ipc_shelf=true`,
`include_packets=true`, `continue_on_failure=true`,
`guidance_complete=true`, `guidance_missing_count=0`, and `failed_rows=[]`.
It wrote `manifest.json`, `review_index.html`, 16 docked row screenshots, and
1004 frame PNGs; the first selected scene was `floating_base` and the last was
`rigid_ipc_heavy_stack_packet`. A read-only HTML asset audit found 0 missing
local assets in both review indexes: 181/181 links for rows 01-36 and 81/81
links for rows 37-52. At that checkpoint, those packets superseded the older
`1781305407`/`1781305860` packet paths as review-index scan artifacts; keep the
older packet paths only as historical completion evidence.

Latest local follow-up: the workflow packet manifest and review index now
record the exact top-level `pixi run py-demo-capture -- --rigid-workflow ...`
command that produced the packet, and review-index screenshot/frame/video links
now normalize workspace-relative artifact paths from per-scene manifests so
they resolve from `review_index.html`. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_request_video_commands python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_links_resolve_workspace_relative_artifacts python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos -q`
reported `4 passed`. The regenerated packets above exercise this fix from the
current HEAD.

Latest local follow-up: fetched `origin/main` and merged it into
`feature/rigid-body-gui-visual-verification`; Git reported the branch was
already up to date with the PR #2986 DART 7 architecture/work-packet harness.
The workflow review-index path now has a focused unit guard that each captured
row card renders exactly one screenshot thumbnail instead of duplicating the
same image in the static contact sheet. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests -q`
reported `1 passed`.

Latest local follow-up: the API-deferred gap audit still finds no public
direct `RigidBody` impulse surface, no public sleep/wake or island activation
surface, and no public loop-closure compliance/stiffness/damping surface. The
in-viewer `Rigid Workflow` search now routes deferred-user terms such as
`direct rigid body impulse`, `sleep wake`, `island activation`, and
`loop closure compliance` to the closest current rows while preserving explicit
non-claim caveats instead of adding speculative numbered rows. Focused guard:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_routes_deferred_api_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_deferred_api_gaps_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `6 passed`.

Latest local state: after the row-36 loop-closure implementation commit, a
fresh full numbered workflow packet completed all 36 World Rigid Body rows in
one `py-demo-capture -- --rigid-workflow` run. That packet exercises the same
docked Filament/ImGui capture path, per-row manifests, screenshots, frame
directories, workflow guidance, and `review_index.html` contact sheet that a
maintainer would use to review the row-15-through-row-36 DART 7 harness pass.
The current optional extended packet has also been rechecked for rows 37-53,
covering related evidence, direct Rigid IPC shelf routes, and capture-first IPC
stress packets.

Latest implementation slice: row 36 now gives the variational rigid multibody
loop-closure family row the same reviewable comparison shape as the surrounding
rigid workflow rows while continuing to follow the DART 7
architecture/work-packet harness from PR #2986. `rigid_loop_closure` names
`loop_closure_family_policy_selection` as its comparison axis in the panel and
capture metrics, records held-fixed contact-free variational rigid
multibody/four-link-chain/gravity/time-step context, exports top-level
POINT/DISTANCE/RIGID residual ratios, distance-family distance/tip error,
RIGID orientation error, and maximum step-time metrics, and feeds decisive
latest signals into the workflow review index.

Resume from this state:

- Start with `git status -sb` and `git log -5 --oneline`.
- Expect branch `feature/rigid-body-gui-visual-verification`; verify
  local/remote parity with `git status -sb`.
- The latest recorded PR checks found no PR for the branch. The last remote
  checkpoint before this validation-language correction was
  `6bafd605907 Add pre-contact surrogate visual demo`.
- Do not push newer local commits without explicit approval in the session that
  performs the push.
- Use `build/captures/rigid_workflow_rows_01_36_1781356342/review_index.html`
  and
  `build/captures/rigid_workflow_optional_signal_highlights_1781338541/review_index.html`
  as the current identity-complete review-index scan artifacts. Treat older
  packet directories as historical completion or failure evidence only.
- Use `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md` as the PR
  body seed after push/PR creation is explicitly approved. Its changelog
  checkbox is checked because `CHANGELOG.md` already records the DART 7 rigid
  workflow command, review packet, related-evidence groups, replay metadata,
  and scene additions.
- Use `docs/dev_tasks/rigid_body_visual_verification/COMPLETION_AUDIT.md` to
  distinguish proven local review readiness from missing external gates:
  push/PR approval, DART 7.0 milestone, maintainer acceptance, and same-PR
  dev-task cleanup. It also records the AI principle audit for this substantial
  AI-assisted workflow change.
- If more local progress is requested before pushing/review, keep it limited to
  evidence repair, review-prep, or a newly unblocked public API gap. The latest
  local API-deferred audit still found no current public direct rigid-body
  impulse, sleep/wake or island activation, or loop-closure compliance surface,
  so do not add speculative numbered rows for those topics. The
  non-speculative pre-contact surrogate gap is now covered by
  `diff_pre_contact_surrogate` in the Differentiable shelf and routed from
  `rigid_contact_solver_compare` as related evidence.

Post-approval publication path:

```bash
git fetch origin main
git merge --no-edit origin/main
git push origin HEAD
gh pr create --draft --base main \
  --title "Add rigid-body visual verification workflow" \
  --body-file docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md
gh pr edit --milestone "DART 7.0"
```

Only run those commands after explicit maintainer/user approval in the session
that performs the GitHub mutation. If a PR is created by another route, still
set the `DART 7.0` milestone and keep the dev-task folder until maintainer
acceptance and same-PR cleanup.

Files touched by this row-36 slice:

- `python/examples/demos/scenes/rigid_loop_closure.py`
- `scripts/capture_py_demo.py`
- `python/tests/integration/test_demos_cycle.py`
- `python/tests/unit/test_capture_py_demo.py`
- `python/tests/unit/test_py_demo_panels.py`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`
- `docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md`
- `docs/dev_tasks/rigid_body_visual_verification/COMPLETION_AUDIT.md`

Row-36 loop-closure slice:

- `rigid_loop_closure` now exports
  `comparison_axis=loop_closure_family_policy_selection`, held-fixed
  contact-free variational rigid multibody/four-link-chain/gravity/time-step
  context, controls, closure family/policy lane names, and top-level capture
  metrics for POINT/DISTANCE/RIGID residual ratios, distance-family
  distance/tip error, RIGID orientation error, and max step time.
- The loop-closure panel labels the comparison axis and held-fixed context
  before the existing family diagnostics.
- `scripts/capture_py_demo.py` prioritizes row-36 latest-signal keys so the
  review card orders closure family ratios, distance-family distance/tip error,
  RIGID orientation error, and solver signals before generic metrics.
- `python/examples/demos/README.md`, PLAN-103, and this dev-task hand-off
  document the loop-closure family/policy axis and held-fixed context.

Evidence for this slice:

- Fresh full numbered workflow packet:
  `build/captures/rigid_workflow_rows_01_36_1781309127` completed with
  `status=complete`, `capture_count=36`, `completed_count=36`,
  `failed_count=0`, `workflow_total_count=36`, `guidance_complete=true`,
  `guidance_missing_count=0`, and `failed_rows=[]`. It wrote
  `manifest.json`, `review_index.html`, 36 docked row screenshots, and 2544
  frame PNGs across rows 01-36; the first row was `rigid_body` and the last row
  was `rigid_loop_closure`.
- Fresh optional extended workflow packet:
  `build/captures/rigid_workflow_optional_rows_37_52_1781309448` completed with
  `status=complete`, `capture_count=16`, `completed_count=16`,
  `failed_count=0`, `workflow_total_count=52`, `workflow_row_start=37`,
  `workflow_row_end=52`, `include_related=true`, `include_ipc_shelf=true`,
  `include_packets=true`, `continue_on_failure=true`,
  `guidance_complete=true`, `guidance_missing_count=0`, and
  `failed_rows=[]`. It wrote `manifest.json`, `review_index.html`, 16 docked
  row screenshots, and 1004 frame PNGs; the first selected scene was
  `floating_base` and the last was `rigid_ipc_heavy_stack_packet`.
- Focused row/panel/docs-order/review-index pytest subset reported `6 passed`.
  It included
  `python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families`,
  the comparison-axis panel coverage, docs/sidecar order checks, capture-command
  sync check, and the unit guard that row-36 latest signals prioritize closure
  family ratios, distance-family distance/tip error, RIGID orientation error,
  and solver.
- Real row-36 workflow capture completed under
  `build/captures/rigid_loop_closure_row_36_1781304923` with
  `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`.
- Row 36 reported `comparison_axis=loop_closure_family_policy_selection`,
  held-fixed `contacts=off`, `integration_family=variational integrator`,
  `joint_family=four_revolute_links`, `chain_links=4`, `link_length=0.56`,
  `link_mass=0.55`, `initial_bend=0.18`, `gravity_scale=1.0`,
  `solver=variational_rigid_multibody_loop_closure`, `time_step_ms=5.0`,
  controls `executor_index=0.0`, `gravity_scale=1.0`,
  `closure_family_lanes=[POINT, DISTANCE, RIGID]`,
  `closure_policy_lanes=[residual, solved]`, POINT/DISTANCE/RIGID residual
  ratios about `7.595e11`/`7.458e11`/`7.740e11`, rigid residual orientation
  error about `0.149`, solved orientation error near `2.8e-17`, distance
  solved distance error near `9.4e-15`, distance solved tip error about
  `0.439`, and max step time about `0.483 ms`.
- `review_index.html` showed the row-36 card with `axis`, `held fixed`,
  `controls`, Replay signal/markers, metric-key summary, and latest-signal
  ordering for POINT/DISTANCE/RIGID residual ratios, distance-family
  distance/tip error, RIGID orientation error, and solver. The per-scene
  capture wrote a nonblank docked screenshot with 2446 unique colors and
  71 PNG frames from the 72-frame workflow row capture under
  `build/captures/`.

Previous completed and verified slice: row 35 gives the World multibody
solver-family routing row the same reviewable comparison shape as the
surrounding rigid workflow rows. `rigid_multibody_solver_family` names
`multibody_integration_solve_policy_family` as its comparison axis in the panel
and capture metrics, records held-fixed contact-free World
point-closure/three-link-chain/gravity/time-step context, exports top-level
residual-only residual, solved residual, residual solve ratio, lane residuals,
solved tip error, and maximum step-time metrics, and feeds decisive latest
signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-35 workflow capture completed under
`build/captures/rigid_multibody_solver_family_row_35_1781304535` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 71 PNG frames.

Previous completed and verified slice: row 34 gives the World multibody link
Jacobian row the same reviewable comparison shape as the surrounding rigid
workflow rows. `rigid_link_jacobian` names
`link_origin_jacobian_mapping_family` as its comparison axis in the panel and
capture metrics, records held-fixed contact-free World
two-revolute-link/time-step/finite-difference context, exports top-level link
linear/angular speed, world/body Jacobian gap, finite-difference residual,
transpose-mapped torque, and power-residual metrics, and feeds decisive latest
signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-34 workflow capture completed under
`build/captures/rigid_link_jacobian_row_34_1781304169` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 95 PNG frames.

Previous completed and verified slice: row 33 gives the World multibody link
center-of-mass offset row the same reviewable comparison shape as the
surrounding rigid workflow rows. `rigid_link_center_of_mass` names
`link_center_of_mass_offset_family` as its comparison axis in the panel and
capture metrics, records held-fixed contact-free World
revolute-link/fixed-visual-geometry/mass/gravity/time-step context, exports
top-level centered/positive/negative gravity torque, mirrored-angle,
high-inertia ratio, and acceleration-residual metrics, and feeds decisive
latest signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-33 workflow capture completed under
`build/captures/rigid_link_center_of_mass_row_33_1781303757` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 71 PNG frames.

Previous completed and verified slice: row 32 gives the World multibody
generalized dynamics terms row the same reviewable comparison shape as the
surrounding rigid workflow rows. `rigid_multibody_dynamics_terms` names
`joint_space_dynamics_term_family` as its comparison axis in the panel and
capture metrics, records held-fixed contact-free World dynamics/fixed-base/
revolute-link/target-acceleration/impulse/gravity/time-step context, exports
top-level scalar mass, coupling, torque-gap, response-gap, response-ratio, and
residual metrics, and feeds decisive latest signals into the workflow review
index. Evidence: focused row/panel/docs-order/review-index pytest subset
reported `6 passed`; the real row-32 workflow capture completed under
`build/captures/rigid_multibody_dynamics_terms_row_32_1781303198` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 95 PNG frames.

Previous completed and verified slice: row 31 gives the World multibody
screw-joint pitch row the same reviewable comparison shape as the surrounding
rigid workflow rows. `rigid_screw_joint_pitch` names
`screw_pitch_coupling_family` as its comparison axis in the panel and capture
metrics, records held-fixed contact-free World screw-joint/z-axis/mass/inertia/
time-step context, exports top-level pitch, travel-gap, reverse-sign, and
acceleration-residual metrics, and feeds decisive latest signals into the
workflow review index. Evidence: focused row/panel/docs-order/review-index
pytest subset reported `6 passed`; the real row-31 workflow capture completed
under `build/captures/rigid_screw_joint_pitch_row_31_1781302747` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 95 PNG frames.

Previous completed and verified slice: row 30 gives the World multibody
passive joint parameter row the same reviewable comparison shape as the
surrounding rigid workflow rows. `rigid_joint_passive_parameters` names
`passive_joint_parameter_family` as its comparison axis in the panel and
capture metrics, records held-fixed contact-free World-prismatic/mass/time-step
context, exports top-level energy, slip, and armature-gap metrics, and feeds
decisive latest signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-30 workflow capture completed under
`build/captures/rigid_joint_passive_parameters_row_30_1781302360` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 119 PNG frames.

Previous completed and verified slice: row 29 gives the World multibody joint
motor/limit row the same reviewable comparison shape as the surrounding rigid
workflow rows. `rigid_joint_motor_limits` names
`world_multibody_actuator_limit_family` as its comparison axis in the panel and
capture metrics, records held-fixed World-joint/axis/mass/time-step context,
exports top-level motor clamp, position-limit, and force-gap metrics, and feeds
decisive latest signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-29 workflow capture completed under
`build/captures/rigid_joint_motor_limits_row_29_1781301981` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 95 PNG frames.

Previous completed and verified slice: row 28 gives the rigid-body one-DOF
joint row the same reviewable comparison shape as the surrounding rigid
workflow rows. `rigid_limited_joints` names
`one_dof_joint_constraint_family` as its comparison axis in the panel and
capture metrics, records held-fixed sequential-joint/static-base/z-axis
context, exports top-level locked-axis and free-axis motion metrics, and feeds
decisive latest signals into the workflow review index. Evidence: focused
row/panel/docs-order/review-index pytest subset reported `6 passed`; the real
row-28 workflow capture completed under
`/tmp/dart_capture_rigid_limited_joints_row_28_1781301510` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 23 PNG frames.

Previous completed and verified slice: row 27 gives the rigid-body
distance-spring row the same reviewable comparison shape as the surrounding
rigid workflow rows. `rigid_distance_spring` names
`distance_spring_response_family` as its comparison axis in the panel and
capture metrics, records held-fixed executor/rest-length/payload/time-step
context, exports top-level distance-spring stretch and offset-spin metrics, and
feeds decisive latest signals into the workflow review index. Evidence:
focused row/panel/docs-order/review-index pytest subset reported `6 passed`;
the real row-27 workflow capture completed under
`/tmp/dart_capture_rigid_distance_spring_row_27_1781300614` with
`status=complete`, `capture_count=1`, `completed_count=1`, `failed_count=0`,
`workflow_total_count=36`, `guidance_complete=true`, a nonblank docked
screenshot, and 71 PNG frames from the 72-frame workflow row
capture.

Previous completed and verified slice: row 26 gives the AVBD-pinned
fixed-joint breakage row the same reviewable comparison shape as the
surrounding rigid workflow rows.
`rigid_joint_breakage` names `fixed_break_force_lifecycle` as its comparison
axis in the panel and capture metrics, records the held-fixed AVBD/static-base
context, exports top-level breakage metrics, and feeds decisive latest signals
into the workflow review index.

Files touched by this row-26 slice:

- `python/examples/demos/scenes/avbd_rigid_breakable_joint.py`
- `scripts/capture_py_demo.py`
- `python/tests/integration/test_demos_cycle.py`
- `python/tests/unit/test_capture_py_demo.py`
- `python/tests/unit/test_py_demo_panels.py`
- `python/examples/demos/README.md`
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`
- `docs/dev_tasks/rigid_body_visual_verification/README.md`
- `docs/dev_tasks/rigid_body_visual_verification/RESUME.md`

Row-26 fixed-joint breakage slice:

- `avbd_rigid_breakable_joint.build_breakable_joint_scene()` now exports
  `comparison_axis=fixed_break_force_lifecycle`, held-fixed AVBD/static-base
  context, controls, and top-level capture metrics for payload release
  distance, broken state, captured-offset error, payload speed, and status.
- The shared breakable-joint panel labels the comparison axis and held-fixed
  context before the existing break-force diagnostics.
- `scripts/capture_py_demo.py` prioritizes row-26 latest-signal keys so the
  visible review card shows payload release distance, broken state,
  captured-offset error, payload speed, status, and solver.
- `python/examples/demos/README.md`, PLAN-103, and this dev-task hand-off
  document the break-force lifecycle axis and held-fixed context.

Evidence for this slice:

- Focused row/panel/docs-order/review-index pytest subset reported `7 passed`.
  It included
  `python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage`,
  `python/tests/unit/test_py_demo_panels.py::test_rigid_joint_breakage_panel_resets_lifecycle`,
  and the unit guard that row-26 latest signals prioritize payload release
  distance, broken state, captured-offset error, payload speed, status, and
  solver.
- Real row-26 workflow capture completed under
  `/tmp/dart_capture_rigid_joint_breakage_row_26_1781299982` with
  `status=complete`, `capture_count=1`, `completed_count=1`,
  `failed_count=0`, `workflow_total_count=36`, and
  `guidance_complete=true`.
- Row 26 reported `comparison_axis=fixed_break_force_lifecycle`, held-fixed
  `base=static`, `captured_offset_m=0.62`, `ground_friction=0.6`,
  `payload_mass=1.0`, `solver=AVBD rigid joints`, `time_step_ms=4.0`,
  controls `break_force=1e-12`, `reset_break_force=1e12`,
  `breakage_payload_release_distance=0.41319290960568955`,
  `breakage_broken=1.0`,
  `breakage_captured_offset_error=0.27164249515625993`,
  `breakage_payload_speed=1.331957542419467`,
  `breakage_status=broken`, and history maxima
  `max_payload_release_distance=0.59045308585389`,
  `max_payload_speed=49.220347596711996`.
- `review_index.html` showed the row-26 card with `axis`, `held fixed`,
  `controls`, Replay signal/markers, metric-key summary, and visible latest
  signals for payload release distance, broken state, captured-offset error,
  payload speed, status, and solver. The per-scene capture wrote a nonblank
  docked screenshot and 47 PNG frames from the 48-frame workflow row capture.

Immediate next step if resumed after commit:

1. Verify the branch/worktree with `git status -sb` and
   `git log -5 --oneline`.
2. Continue from a fresh audit of the next rigid workflow gap; do not assume the
   broader DART 7 rigid showcase goal is complete.
3. Preserve the no-push rule until the user explicitly approves a push.

Previous checkpoint: row 25 exposes the fixed-joint pose verifier as a
reviewable comparison packet. `rigid_fixed_joint` labels the fixed relative
transform recovery axis, records held-fixed sequential-joint context, exports
top-level pose-recovery metrics, and gives the workflow `review_index.html`
visible latest signals for fixed-joint translation error, orientation error,
payload speed, angular speed, and solver.

Previous checkpoint: row 23 exposes the tangential kinematic-driver row as a
reviewable comparison packet. `rigid_kinematic_driver` labels the prescribed
tangential-contact response axis, records held-fixed tangential-support
context, exports solver/case/lane metrics, and gives the workflow
`review_index.html` visible latest signals for IPC grip travel, grip speed
ratio, low-friction slip, sequential-impulse caveat driver travel, IPC
proximity gap, and solver pair.

Previous checkpoint: row 24 exposes the normal kinematic-push caveat as a
reviewable comparison packet. `rigid_kinematic_normal_push` labels the
prescribed normal-contact response axis, records held-fixed normal-paddle
context, exports lane/solver/case metrics, and gives the workflow
`review_index.html` visible latest signals for target-travel divergence, SI
target travel, IPC penetration depth, solver pair, case pair, and solver label.

Previous checkpoint: rows 19-21 are a verified contact-failure comparison
mini-packet. `rigid_friction_threshold`, `rigid_spin_roll_coupling`, and
`rigid_stack_stability` expose comparison axes and held-fixed values in both
their panels and scene-owned capture metrics, and `scripts/capture_py_demo.py`
surfaces the decisive row 19-21 latest-signal values in `review_index.html`.

Rows 19-21 evidence:

- Focused row/panel/review-index pytest subset reported `5 passed`.
- Real workflow capture completed under
  `/tmp/dart_capture_rigid_contact_failure_rows_19_21_1781296166` with
  `status=complete`, `capture_count=3`, `completed_count=3`, `failed_count=0`,
  and `guidance_complete=true`.
- The review index showed `axis`, `held fixed`, `controls`, and
  `latest signals` for rows 19-21: `friction_threshold_lane` with IPC fixed,
  `spin_roll_initial_condition` with sequential impulse fixed, and
  `rigid_body_solver_family` with SI-vs-IPC solver pair plus top-x divergence
  and clearance signals.

Previous checkpoint: workflow `review_index.html` row cards now surface actual
decision values from scene-owned metrics, not only metric keys. Captured rows
show held-fixed values when present, control settings, and compact latest
signals such as solver pair, executor pair, contact-policy pair, divergence,
and step/contact counters.

Previous checkpoint: refreshed the core solver/contact-policy visual evidence
packet for rows 15-17. The real workflow capture completed
`rigid_solver_compare`, `rigid_executor_equivalence`, and
`rigid_contact_solver_compare` together, keeping solver-family,
executor-only, and contact-policy decisions adjacent in `review_index.html`.

Previous checkpoint: direct single-scene `py-demo-capture -- --scene ...`
manifests for maintained rigid workflow rows now include a `workflow_guidance`
block with row number, role, user question, try-first action, inspect signals,
healthy signal, and scope note. This keeps a one-row capture self-describing
when it is handed to a reviewer outside the full workflow packet.

Previous checkpoint: the rigid workflow `review_index.html` now shows each
selected row's packet-preserving `workflow_rerun_command` directly on the row
card, not only inside the Failed Rows summary. This lets reviewers rerun any
planned, captured, or failed row with the same workflow packet shape, absolute
row number, and row-specific rerun output directory.

Previous checkpoint: the C++ `dart-demos --list` path is now a non-graphics
catalog command. `runDemos()` prints the registered C++ demo catalog before GUI
backend initialization, `examples/demos/CMakeLists.txt` registers
`EXAMPLE_dart_demos_list` as a normal non-graphics example/catalog test, and
`examples/README.md` documents `pixi run demos -- --list` for listing scenes
without opening the GUI.

Previous checkpoint: commit
`41186522838 Retire C++ collision sandbox placeholder` brings the C++
companion back into sync with the completed py-demos collision-sandbox
retirement. The stale
`planned_collision_sandbox` row is removed from `examples/demos/registry.cpp`,
and legacy C++ runner requests for `collision_sandbox`, `point_cloud`, and
`polyhedron_visual` now route users to the concrete Python collision-debugging
GUI rows: `rigid_contact_inspector`, `rigid_collision_query_options`, and
`rigid_collision_casts`.

Previous checkpoint: commit `18c3750a7af Retire planned collision sandbox row`
retires the `planned_collision_sandbox` placeholder from the py-demos catalog
because the concrete collision-debugging GUI rows already cover that route.
PLAN-103 and the Python demo README name the replacement path.

Previous checkpoint: PLAN-103's remaining planned World-port GUI
rows after the rigid verifier reached maintainer-acceptance readiness. The
Planned World Ports placeholders now tell users which current py-demo route to
try first, which World/API or asset gap blocks the full port, and what
replacement condition will retire the placeholder; focused panel tests guard the
metadata and GUI text.

Previous checkpoint: failed rows from resilient workflow packets became
directly actionable. `py-demo-capture -- --rigid-workflow
--continue-on-failure` records `failed_rows` with workflow row-range rerun
commands that preserve optional packet flags and absolute row numbering, while
the final workflow manifest and process exit code still fail when any selected
row failed. The in-viewer `Rigid Workflow` panel exposes the resilient
extended-packet command, and README/PLAN-103 describe the same user path.

Previous checkpoint, updated after the pre-contact surrogate related route
expanded the optional packet: the public packet row-range rerun examples now
tell users to rerun rows 48-49 when they request
`--include-related --include-packets`, so both `rigid_ipc_stack_packet` and
`rigid_ipc_heavy_stack_packet` are included. The drift guard scopes the public
rerun example and prevents it from regressing to a single packet-row command.

Previous checkpoint: the capture-first heavier Rigid IPC stack packet slice
completed. The existing four-box `rigid_ipc_stack_packet` now shares a
spec-driven implementation with a six-box `rigid_ipc_heavy_stack_packet`; both
stay in the non-numbered Rigid IPC shelf and only enter workflow capture
packets through the opt-in `--include-packets` group. The default 36-row World
Rigid Body workflow remains unchanged.

Historical stop-only note: after the heavy packet commit, the user requested
no further implementation and no further verification while hand-off docs were
ensured. That note was superseded by later active-goal continuations.

Observed repository state at this hand-off:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Prior stop-state instruction: the user explicitly requested no further
  implementation and no further verification; only hand-off docs were ensured
  in that turn. This was later superseded by a continuation request for the
  active goal.
- Current continuation started with the branch clean at
  `867c5f5d8a5 Add rigid workflow guidance to capture manifests`, eleven
  commits ahead of `origin/feature/rigid-body-gui-visual-verification`,
  committed `bedc418df0e Record solver contact workflow packet evidence`, then
  improved workflow review-index row cards to expose actual latest metric
  values for solver/contact decision review.
- At the stop-state hand-off, those docs updates were uncommitted local
  modifications and no push was performed. The resumed continuation verified
  this evidence-docs slice locally before choosing the next bounded
  GUI-verification improvement; push still requires explicit approval.
- Resume check: inspect `git status -sb` and `git log -8 --oneline` before any
  resumed work. Expect the latest completed implementation commit to be
  `Surface workflow review latest signals` if this slice has been committed;
  otherwise inspect the uncommitted diff for the latest-signals review-index
  follow-up.
- Do not push without explicit approval in the session that performs the push.
- Latest local slice updates `examples/demos/registry.cpp`,
  `scripts/run_cpp_example.py`, `python/tests/unit/test_run_cpp_example.py`,
  `examples/README.md`, PLAN-102 dashboard text, and this handoff so the C++
  companion no longer advertises the retired collision-sandbox placeholder.
- Verification for the latest C++ companion cleanup: focused
  `python/tests/unit/test_run_cpp_example.py` reported `66 passed` before and
  after `pixi run lint`; `pixi run ex collision_sandbox` exits nonzero with
  the expected `pixi run py-demos -- --scene rigid_contact_inspector`,
  `rigid_collision_query_options`, and `rigid_collision_casts` guidance and no
  `planned_collision_sandbox` text.
- Latest local follow-up makes `dart-demos --list` a non-graphics catalog path:
  `runDemos()` now prints the registered C++ demo catalog before GUI backend
  initialization, and CMake registers `EXAMPLE_dart_demos_list` as a normal
  non-graphics example/catalog test. This removes the previous Mesa/GLX host
  caveat for the list command while leaving launch/cycle behavior unchanged.
- Verification for the latest `dart-demos --list` follow-up: the
  `pixi run demos -- --list` command returned the catalog with `rigid_body` and
  `planned_mobile_manipulation` present and `planned_collision_sandbox` absent.
  The focused `EXAMPLE_dart_demos_list` CTest passed, the focused
  `python/tests/unit/test_run_cpp_example.py` suite reported `66 passed`, and
  `pixi run lint`, `pixi run check-lint-md`, `pixi run check-docs-policy`,
  `pixi run check-lint-spell`, and `git diff --check` passed.
- Latest local review-index follow-up updates `scripts/capture_py_demo.py`,
  `python/tests/unit/test_capture_py_demo.py`, `python/examples/demos/README.md`,
  PLAN-103's rigid sidecar, and this handoff so every selected workflow row card
  shows its packet-preserving rerun command.
- Verification for the latest review-index follow-up: the focused
  `python/tests/unit/test_capture_py_demo.py -k rigid_workflow` suite reported
  `25 passed, 14 deselected` with the built `dartpy` PYTHONPATH. The final
  lint, Markdown-format, docs-policy, spelling, and `git diff --check` gates
  passed.
- Latest local direct-manifest follow-up updates `scripts/capture_py_demo.py`,
  `python/tests/unit/test_capture_py_demo.py`, `python/examples/demos/README.md`,
  PLAN-103's rigid sidecar, and this handoff so direct captures carry rigid
  workflow guidance without requiring the full packet context.
- Verification for the latest direct-manifest follow-up: the focused
  `python/tests/unit/test_capture_py_demo.py -k 'visual_capture_manifest_records_image_evidence or rigid_workflow'`
  suite reported `26 passed, 13 deselected` with the built `dartpy` PYTHONPATH.
  The final lint, Markdown-format, docs-policy, spelling, and `git diff --check`
  gates passed.
- Latest local solver/contact-policy mini-packet evidence updates
  `python/examples/demos/README.md`, PLAN-103's rigid sidecar, and this handoff
  with the current row 15-17 capture command and artifact summary.
- Verification for the latest solver/contact-policy mini-packet evidence: the
  row-range workflow capture completed rows 15-17 under
  `/tmp/dart_capture_rigid_workflow_solver_contact_rows_15_17_1781294600` with
  `capture_count=3`, `completed_count=3`, `failed_count=0`,
  `guidance_complete=true`, docked nonblank screenshots, PNG frame sequences,
  review-index row cards, and scene metrics for solver-family, executor, and
  contact-policy comparison axes. Post-resume `pixi run lint`,
  `pixi run check-lint-md`, `pixi run check-docs-policy`,
  `pixi run check-lint-spell`, and `git diff --check` passed for this
  evidence-docs slice.
- Latest local review-index latest-signals follow-up updates
  `scripts/capture_py_demo.py`, `python/tests/unit/test_capture_py_demo.py`,
  `python/examples/demos/README.md`, PLAN-103's rigid sidecar, and this handoff
  so row cards render held-fixed/control values and latest comparison signals
  directly from `scene_metrics.latest.metrics`.
- Verification for the latest review-index latest-signals follow-up: the
  focused rigid-workflow aggregate/dry-run/video unit subset in
  `python/tests/unit/test_capture_py_demo.py` reported `3 passed, 36
deselected`. A real rows 15-17 workflow capture completed under
  `/tmp/dart_capture_rigid_workflow_latest_signals_rows_15_17_1781295362` with
  `capture_count=3`, `completed_count=3`, and `failed_count=0`; the review
  index showed control values plus latest signals for the solver pair, executor
  pair, contact-policy pair, and divergence summaries.
- Latest local slices update `python/examples/demos/registry.py`,
  `python/examples/demos/scenes/planned.py`,
  `python/tests/unit/test_py_demo_panels.py`,
  `python/tests/integration/test_demos_cycle.py`,
  `python/examples/demos/README.md`, PLAN-103, and this handoff to remove the
  obsolete collision sandbox placeholder and route users to the real collision
  rows.
- Previous local slices update `python/examples/demos/scenes/planned.py`,
  `python/examples/demos/scenes/robot_puppets.py`,
  `python/tests/unit/test_py_demo_panels.py`,
  `python/examples/demos/README.md`, PLAN-103, and this handoff to make
  planned World-port rows actionable instead of inert placeholders.
- Earlier local slices update `scripts/capture_py_demo.py`,
  `python/examples/demos/runner.py`, `python/tests/unit/test_capture_py_demo.py`,
  `python/tests/unit/test_py_demo_panels.py`,
  `python/tests/integration/test_demos_cycle.py`,
  `python/examples/demos/README.md`, and the PLAN-103 rigid sidecar to support
  and document resilient workflow packets, failed-row summaries, and
  workflow-context rerun commands.
- Previous row-range slice updated `python/examples/demos/README.md`,
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`, and
  `python/tests/integration/test_demos_cycle.py` to keep the packet row-range
  example aligned with the two capture-first packet rows.
- Latest local failed-row rerun-command slice is titled
  `Add rigid workflow failed-row reruns`; it builds on
  `93a1ce752e5 Summarize rigid workflow failed rows`,
  `c9f00c351d6 Let rigid workflow packets continue after failures`, and
  `12df4e1f083 Refresh rigid packet row range guidance`.
- Origin tip observed before these local slices was
  `bdf757db2c9 Refresh rigid handoff stop state`.
- There is no PR associated with this branch.
- Do not push without explicit approval; always verify with `git status -sb`
  and `git log -5 --oneline` before resuming.
- The latest acceptance-readiness audit found no newly unblocked rigid visual
  row to implement before maintainer review. Public `dartpy` still lacks direct
  rigid-body impulse, sleep/wake, island activation, and loop-closure
  compliance surfaces; the point-force, joint-space impulse, energy/momentum,
  breakage, and spring/compliance-like questions are already represented by
  existing rows or durable follow-up language.
- The local implementation adds/updates
  `python/examples/demos/scenes/rigid_ipc_stack_packet.py`,
  `python/examples/demos/registry.py`, `scripts/capture_py_demo.py`,
  `python/tests/unit/test_py_demo_panels.py`,
  `python/tests/unit/test_capture_py_demo.py`,
  `python/tests/integration/test_demos_cycle.py`,
  `python/examples/demos/README.md`, and the PLAN-103 rigid sidecar.
- Verification for this slice: syntax checks passed; the focused Python suite
  below reported `11 passed`; the extended
  workflow dry-run planned rows `51-52 / 52` with both packet variants and
  complete guidance; the real row-52 docked workflow capture completed with one
  captured row, no failed rows, a nonblank docked screenshot, 11 PNG frames, 12
  metric events, `box_count=6.0`, `top_mass=4.25`, and
  `status=capture-first`; `pixi run lint` passed; the current continuation
  reran the focused pytest after lint with `11 passed`; and `git diff --check`
  was clean before committing the failed-row rerun-command slice.
- Verification for the latest row-range sync: the focused
  `test_rigid_visual_capture_first_packets_are_documented` pytest passed after
  the new guard was made whitespace/history scoped; the exact updated dry-run
  command planned rows `47-48 / 48` with two capture-first packet rows and
  complete guidance under
  `/tmp/dart_capture_rigid_workflow_packet_rows_47_48_1781287569`;
  `pixi run lint` passed; the post-lint focused pytest passed again; and
  `git diff --check` was clean.
- Verification for the latest failure-resilience slice: the focused pytest
  covering fail-fast behavior, continue mode, workflow-only flag validation,
  panel command rendering, failed-row summaries, workflow row rerun commands,
  and docs guard reported `11 passed`. The public
  dry-run with `--continue-on-failure` over rows 51-52 reported
  `status=planned`, `continue_on_failure=true`, `capture_count=2`,
  `workflow_total_count=52`, `guidance_complete=true`, and both capture-first
  stack packet rows. The current continuation reran the focused pytest after
  adding the review-index failure-mode badge, then extended the manifest and
  review index with failed-row triage summaries plus workflow row-range rerun
  commands for resilient packets, verified the public dry-run reports
  `failed_rows` length `0` and review-index `failure mode=continue` for a
  planned packet, verified planned rows 51 and 52 carry
  `workflow_rerun_command` values preserving the optional packet flags and
  absolute row ids, and then ran the required pre-commit lint gate.

Previous Replay capture-metadata checkpoint context: this checkpoint added
reviewer-facing Replay timeline metadata to the capture packet after the
numbered 36-row Replay timeline pass completed. Single-scene
`py-demo-capture` manifests include a JSON-safe
`scene_metadata.replay_timeline` summary with the Replay panel name,
value-track label, and signal/marker availability; workflow
`review_index.html` cards show that Replay track next to the existing row
guidance and metric summaries.

Previous loop-closure checkpoint context: it completed the
`rigid_loop_closure` Replay timeline slice after
the multibody solver-family checkpoint. The variational rigid multibody
loop-closure row now uses max closure residual ratio as its Replay value track
and marks solve-advantage, residual-versus-solved separation, distance-family
tip drift, and rigid-orientation frames.

Expected repository state for that earlier checkpoint:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Local `HEAD` before the loop-closure implementation work:
  `1add2036097 Add multibody solver family replay timeline`.
- `git status -sb` before this continuation resumed showed the branch ahead of
  `origin/feature/rigid-body-gui-visual-verification` by fourteen commits with
  uncommitted row 36 scene/test edits and stop-only hand-off docs; this
  checkpoint supersedes that hand-off state.
- Latest implementation checkpoints covered by this hand-off:
  `4c9f367bcd0 Preserve requested rigid workflow packet groups`,
  `f48187d6ce2 Summarize rigid workflow packet groups in review index`, and
  `f01f471bae7 Expose rigid workflow packet commands in the panel`, followed
  by `3c5b9e517d3 Enable rigid workflow video packets`,
  `d5c6de2bee1 Describe optional rigid workflow rows`,
  `5a4529f0083 Audit rigid workflow guidance coverage`, and
  `ad013e62069 Refresh rigid guidance audit handoff`, followed by the current
  live open-command, stack Replay timeline, pushed hand-off, contact
  manipulation Replay timeline, kinematic-driver Replay timeline,
  normal-push Replay timeline, fixed-joint Replay timeline, and
  joint-breakage Replay timeline, distance-spring Replay timeline,
  limited-joints Replay timeline, motor-limits Replay timeline,
  passive-parameters Replay timeline, screw-joint pitch Replay timeline,
  multibody dynamics-terms Replay timeline, link center-of-mass Replay
  timeline, link-Jacobian Replay timeline, multibody solver-family Replay
  timeline, and loop-closure Replay timeline checkpoint.
- `d98abdde973 Refresh rigid visual verification handoff` is a docs-only pushed
  checkpoint after the stack Replay timeline slice.
- Local `HEAD` before the link center-of-mass implementation commit was
  `7438e13cca9 Add multibody dynamics replay timeline`; the branch was
  observed clean and ahead of
  `origin/feature/rigid-body-gui-visual-verification` by eleven commits before
  this slice.
- Local `HEAD` before the link-Jacobian implementation work was
  `c7042091c2b Add link center of mass replay timeline`; the branch was ahead
  of `origin/feature/rigid-body-gui-visual-verification` by twelve commits and
  carried docs-only stop-handoff edits that this checkpoint supersedes.
- Local `HEAD` before the multibody dynamics-terms implementation commit was
  `29ab458fc01 Add screw joint replay timeline`; the branch was observed clean
  and ahead of `origin/feature/rigid-body-gui-visual-verification` by ten
  commits before this slice.
- Local `HEAD` before the screw-joint pitch implementation commit was
  `6f307966524 Add passive joint replay timeline`; the branch was ahead of
  `origin/feature/rigid-body-gui-visual-verification` by nine commits before
  that slice.
- The kinematic-driver Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `IPC grip box travel`, with markers for IPC grip contact/carry progress,
  slip-lane slip, and static-like caveat frames.
- The normal-push Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Target travel divergence`, with markers for contact, IPC penetration, SI
  push-progress, and divergence frames.
- The current fixed-joint Replay timeline slice adds `replay_timeline_signal(...)`,
  `replay_timeline_marker(...)`, and `info["replay_timeline"]` metadata. The
  intended value track label is `Fixed-joint offset error`, with markers for
  pose-error and residual-motion recovery frames.
- The current joint-breakage Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`,
  `replay_capture_state`, `replay_restore_state`, and
  `info["replay_timeline"]` metadata through the shared AVBD breakable-joint
  builder. The intended value track label is `Payload release distance`, with
  markers for broken or released frames.
- The current distance-spring Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Max spring stretch`, with markers for high-stretch or off-center-spin
  frames.
- The current limited-joints Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Locked-axis error`, with markers for locked-error or free-axis-motion
  frames.
- The current motor-limits Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Force travel gap`, with markers for velocity-clamp, position-stop, or
  effort-cap frames.
- The current passive-parameters Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Armature position gap`, with markers for damping energy separation, Coulomb
  slip, or armature-lag frames.
- The current screw-joint pitch Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Coarse/fine travel gap`, with markers for pitch-spread, zero-pitch
  contrast, or reverse-sign frames.
- The current multibody dynamics-terms Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Response norm gap`, with markers for response separation, off-diagonal
  coupling, or heavy-load torque frames.
- The current link center-of-mass Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Mirrored COM angle spread`, with markers for mirrored-angle,
  centered-still, or high-inertia-lag frames.
- The current link-Jacobian Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Link-origin speed`, with markers for high-twist, wrench-load, world/body
  Jacobian gap, or residual-alert frames.
- The current multibody solver-family Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track label is
  `Residual solve ratio`, with markers for solve-advantage, residual-only
  drift, or solved-tight frames while residual rows remain loose.
- The current loop-closure Replay timeline slice adds
  `_last_float(...)`, `replay_timeline_signal(...)`,
  `replay_timeline_marker(...)`, and `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_loop_closure.py`. The intended value
  track label is `Max closure residual ratio`, with markers for large closure
  solve advantage, residual-versus-solved separation, distance-family tip
  drift, and rigid-orientation solve frames.
- There is no PR associated with this branch at checkpoint time.
- The numbered rigid workflow Replay timeline pass is now complete through row 36. A future session should re-evaluate the durable sidecar and dashboard
  before selecting the next bounded rigid visual-verification slice.
- The current continuation resumed implementation from the active persistent
  goal and finished the pending guidance-audit checks after the previous
  hand-off-only stop checkpoint.
- The live open-command continuation finished the previously documented WIP,
  added capture-helper tests and public docs, and reran focused tests, a public
  dry-run artifact inspection, and `pixi run lint`.
- The stack Replay timeline continuation added `replay_timeline` metadata to
  `rigid_stack_stability`, updated tests and docs, and ran focused tests plus a
  real docked capture.
- The contact-manipulation Replay timeline continuation added
  `replay_timeline` metadata to `rigid_contact_manipulation`, updated tests and
  docs, and ran focused tests plus a real docked capture.
- The kinematic-driver Replay timeline continuation added
  `replay_timeline` metadata to `rigid_kinematic_driver`, updated tests and
  docs, and ran focused tests plus a real docked capture.
- The normal-push Replay timeline continuation added `replay_timeline` metadata
  to `rigid_kinematic_normal_push`, updated tests and docs, and ran focused
  tests plus a real docked capture.
- The fixed-joint Replay timeline continuation added `replay_timeline` metadata
  to `rigid_fixed_joint`, updated tests and docs, and ran focused tests plus a
  real docked capture.
- The joint-breakage Replay timeline continuation added `replay_timeline`
  metadata to the shared AVBD breakable-joint builder used by
  `rigid_joint_breakage`, updated tests and docs, and ran focused tests plus a
  real docked capture.
- The distance-spring Replay timeline continuation added `replay_timeline`
  metadata to `rigid_distance_spring`, updated tests and docs, and ran focused
  tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The limited-joints Replay timeline continuation added `replay_timeline`
  metadata to `rigid_limited_joints`, updated tests and docs, and ran focused
  tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The motor-limits Replay timeline continuation added `replay_timeline`
  metadata to `rigid_joint_motor_limits`, updated tests and docs, raised its
  maintained capture budget to 96 frames so the position stop is visible, and
  ran focused tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The passive-parameters Replay timeline continuation added `replay_timeline`
  metadata to `rigid_joint_passive_parameters`, updated tests and docs, and
  ran focused tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The screw-joint pitch Replay timeline continuation added `replay_timeline`
  metadata to `rigid_screw_joint_pitch`, updated tests and docs, and ran
  focused tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The multibody dynamics-terms Replay timeline continuation added
  `replay_timeline` metadata to `rigid_multibody_dynamics_terms`, updated tests
  and docs, and ran focused tests, drift guards, a real docked capture,
  `pixi run lint`, and `git diff --check`.
- The link center-of-mass Replay timeline continuation added `replay_timeline`
  metadata to `rigid_link_center_of_mass`, updated tests and docs, and ran
  focused tests, drift guards, a real docked capture, `pixi run lint`, and
  `git diff --check`.
- The multibody solver-family Replay timeline continuation added
  `replay_timeline` metadata to `rigid_multibody_solver_family`, updated tests
  and docs, ran focused tests, drift guards, a real docked capture,
  `pixi run lint`, and `git diff --check`, then committed locally as
  `1add2036097 Add multibody solver family replay timeline`.
- The loop-closure Replay timeline continuation added `replay_timeline`
  metadata to `rigid_loop_closure`, updated tests and docs, and ran focused
  tests plus a real docked capture. Drift guards, lint, and diff checks are
  recorded in the validation section below.
- Historical note: these checkpoints were local and unpushed at that moment.
  Current branch reality is recorded in the hand-off and readiness audit above.
- Before any future commit, rerun the repository-mandated `pixi run lint`.

## Last Session Summary

The latest continuation completed the heavy capture-first Rigid IPC stack
packet. `rigid_ipc_heavy_stack_packet` is a six-box, top-heavy variant that
shares the stack-packet implementation, stays outside the numbered 36-row
workflow, and appears only in the optional packet group after
`rigid_ipc_stack_packet`. Syntax checks, focused pytest, a rows 51-52 extended
dry-run, a real row-52 docked workflow capture, lint, and diff checks passed.
The current continuation then refreshed the public row-range rerun examples so
the related-plus-packets command covers rows 48-49 instead of only the original
row 47 packet, and added focused drift coverage for that public command.
The newest continuation adds `--continue-on-failure` to workflow packets,
surfaces a resilient extended-packet command in the live `Rigid Workflow`
panel, documents the command in README/PLAN-103, and adds focused regression
coverage for failure continuation and workflow-only flag validation.

The previous implementation session fixed `py-demos --cycle-scenes` so a
bounded `--frames N` value applies to every scene instead of ending the whole
catalog after the first frame. It updated tests and docs, validated the full
151-scene headless cycle command, and committed the change locally as
`0e38e3e807d Fix py-demos cycle scene frame budget`.

The latest session added uncommitted work for two user-facing rigid workflow
gaps: broader in-viewer search aliases for DART 7 solver/backend/contact terms,
and `py-demo-capture -- --rigid-workflow` for planning or running all numbered
rigid captures into a workflow-level manifest. Focused tests and the public
dry-run command passed, and the full 36-scene workflow capture later completed
successfully under `/tmp/dart_capture_rigid_workflow_full_1781257174`.
After that full capture, the capture helper was adjusted to flush workflow
progress and isolate each per-scene capture in a child process. The user then
requested handoff-only work and no further verification.

The current continuation inspected the uncommitted workflow diff, fixed a
missing-evidence bug where a child process could exit 0 without writing its
per-scene manifest, and added a regression so that case fails the top-level
workflow manifest. Focused tests, public dry-run, lint, and `git diff --check`
passed after the fix.

The latest continuation added two more user-facing GUI improvements: the
`Rigid Workflow` panel now shows each row's exact `py-demo-capture` command,
and adjacent parameter/profiling rows now label their comparison axis plus
held-fixed controls in both panels and capture metrics. Focused pytest,
the public workflow dry-run, lint, and `git diff --check` passed.

The current continuation adds the next workflow review surface:
`py-demo-capture -- --rigid-workflow` now rewrites a top-level
`review_index.html` contact sheet on dry-run, progress updates, failure, and
completion. The contact sheet links every numbered row's manifest, screenshot,
frame directory, command, and comparison/metrics summary so reviewers can scan
all 36 outputs without opening per-scene folders manually. Focused pytest,
public dry-run, lint, and `git diff --check` passed; the dry-run manifest
recorded `artifacts.review_index`, `capture_count=36`, and `status=planned`.
The latest continuation then ran the slow full workflow capture with the review
index enabled. It completed all 36 numbered rows under
`/tmp/dart_capture_rigid_workflow_full_review_index_1781259714`; the top-level
manifest reported `status=complete`, `completed_count=36`, `failed_count=0`,
and `elapsed_s=314.278`, and the generated `review_index.html` contained
36 screenshot thumbnails plus comparison/metric summaries.
The verified rigid workflow search/capture/review-index slice was then
committed and pushed as
`e8278b6fb53 Improve rigid workflow capture evidence`.

The current continuation makes one focused viewer-routing improvement:
related-evidence `Find row` matches such as `avbd prismatic` now open the
matched non-numbered shelf scene directly instead of landing on the numbered
source row first.

The newest continuation adds an opt-in related-evidence capture bundle:
`py-demo-capture -- --rigid-workflow --include-related` appends the ten
non-numbered related shelf routes after the 36 numbered rows and records them
in the same manifest/review index with `workflow_group=related_evidence`.

The latest continuation adds a second opt-in bundle:
`py-demo-capture -- --rigid-workflow --include-packets` appends the
capture-first `rigid_ipc_stack_packet` after the numbered rows and optional
related-evidence routes with `workflow_group=capture_first_packet`.

The current continuation adds workflow row-range selection:
`py-demo-capture -- --rigid-workflow --workflow-start-row N --workflow-end-row M`
captures or dry-runs only the selected absolute workflow rows while keeping
their original row labels and `scenes/NN_<scene>` output directories.

The latest continuation enriches the generated workflow evidence packet:
numbered rigid workflow entries now carry the same row guidance as the
in-viewer `Rigid Workflow` panel, including role label, user question,
try-first action, inspect signals, healthy signal, and scope note in both the
manifest and `review_index.html`.

The current continuation promotes `rigid_ipc_edge_drop` into a self-describing
related-evidence route from `rigid_solver_compare`, with a panel and capture
metrics for degenerate edge-barrier gap, tilt, angular speed, contact count,
step timing, and status. The scene intentionally remains outside the numbered
36-row World Rigid Body workflow because it is an IPC-specific capability view.

The newest continuation adds scene-owned capture metrics to the remaining
direct Rigid IPC shelf scenes: `rigid_ipc`, `rigid_ipc_slide`,
`rigid_ipc_incline`, and `rigid_ipc_pile`. These scenes still stay outside the
numbered 36-row workflow and outside the related-evidence route table, but
their direct docked captures now carry barrier-settle, tangential-slide,
inclined-slide, and pile speed/gap/contact/timing evidence while preserving
shared replay controls.

The current continuation adds `--include-ipc-shelf` to
`py-demo-capture -- --rigid-workflow`. The new opt-in group appends
`rigid_ipc`, `rigid_ipc_slide`, `rigid_ipc_incline`, and `rigid_ipc_pile` to
the workflow manifest and `review_index.html` with
`workflow_group=rigid_ipc_shelf`, without changing the default 36-row workflow
or the existing related/packet-only row counts.

The earlier stop checkpoint was hand-off documentation only. The current
continuation supersedes it by completing and verifying the link-Jacobian Replay
timeline slice.

The current continuation fixes a review-packet metadata edge case:
row-range workflow manifests now keep `include_related`, `include_ipc_shelf`,
and `include_packets` tied to the requested packet shape, while new
`selected_include_related`, `selected_include_ipc_shelf`, and
`selected_include_packets` fields describe the optional groups actually present
in the selected rows. The workflow `review_index.html` header shows the same
row-span, requested-groups, and selected-groups context for reviewers.

The newest continuation closes the loop back into the live GUI: the
`Rigid Workflow` panel now lists full numbered packet, current-row range rerun,
and extended related/IPC-shelf/packet commands under a `Review packet` section.

The current continuation adds workflow-packet motion evidence: `--video --fps`
is now accepted with `--rigid-workflow`, passed through to each selected row
capture, recorded in per-scene manifests, and linked from the workflow
`review_index.html`; the live `Rigid Workflow` panel exposes a current-row
motion packet command.

The final action in this checkpoint is hand-off documentation only. The user
explicitly requested stopping all further implementation and verification, so
no new tests, lint, dry-runs, or diff checks were run for this docs-only
handoff update.

The newest continuation adds optional-row self-description to workflow
evidence packets. Related-evidence, direct Rigid IPC shelf, and capture-first
packet rows now carry role, user question, try-first action, inspect signals,
healthy signal, scope, and related-source metadata into `manifest.json` and
`review_index.html`, rather than relying only on the `workflow_group` label.

The current continuation adds a guidance-coverage audit to the same outputs.
`manifest.json` records `guidance_complete`, `guidance_missing_count`, and
`guidance_missing_rows`, and `review_index.html` shows a guidance badge plus a
warning block when a selected row is missing required self-description fields.
Focused pytest and the public rows 37-51 extended-packet dry-run both passed
after the stop checkpoint was superseded by the active-goal continuation.

The latest continuation finishes the live open-command slice. The
`Rigid Workflow` panel and generated workflow review cards now put a live
`pixi run py-demos -- --scene ...` command next to the existing reproducible
capture command, and the manifest records the same command as
`viewer_command`. Focused pytest, public row-15 dry-run artifact inspection,
and `pixi run lint` passed.

The current continuation adds replay-guided timeline metadata to
`rigid_stack_stability`. The shared Replay panel now uses top-x divergence as
its value track and marks overlap, low-clearance, top-drift, or
solver-divergence frames for targeted visual debugging. Focused pytest and a
real docked capture passed.

That session then stopped at hand-off documentation only. That historical
stop-only handoff was later superseded by the active-goal continuation; the
contact-manipulation, kinematic-driver, normal-push, and fixed-joint timeline
slices below are the current local continuation state.

The current continuation adds replay-guided timeline metadata to
`rigid_contact_manipulation`. The shared Replay panel now uses travel
divergence as its value track and marks pusher contact/proximity,
target-motion, or solver-divergence frames for targeted visual debugging.
Focused pytest, sidecar/README drift guards, and a real docked capture passed.

The latest continuation completes the next Replay timeline slice for
`rigid_kinematic_driver`. The shared Replay panel now uses IPC grip box travel
as its value track and marks contact/carry progress, slip-lane slip, and
static-like caveat frames for targeted visual debugging. Focused pytest,
sidecar/README drift guards, `pixi run lint`, `git diff --check`, and a real
docked capture passed.

The current continuation completes the next Replay timeline slice for
`rigid_kinematic_normal_push`. The shared Replay panel now uses target-travel
divergence as its value track and marks contact, IPC penetration, SI
push-progress, and divergence frames for targeted visual debugging. Focused
pytest, sidecar/README drift guards, `pixi run lint`, `git diff --check`, and a
real docked capture passed before the local commit
`3ea4d1fcd4e Add normal push replay timeline`.

The latest continuation completes the next Replay timeline slice for
`rigid_fixed_joint`. The shared Replay panel now uses fixed-joint offset error
as its value track and marks pose-error or residual-motion frames during
recovery. Focused pytest, sidecar/README drift guards, `pixi run lint`,
`git diff --check`, and a real docked capture passed before the local fixed-joint
commit.

The current continuation completes the next Replay timeline slice for
`rigid_joint_breakage`. The shared Replay panel now uses payload release
distance as its value track and marks broken or released frames during the AVBD
break-force lifecycle. Focused pytest, sidecar/README drift guards,
`pixi run lint`, `git diff --check`, and a real docked capture passed before
the local joint-breakage commit.

The latest continuation completes the next Replay timeline slice for
`rigid_distance_spring`. The shared Replay panel now uses maximum spring
stretch as its value track and marks high-stretch or off-center-spin frames.
Focused pytest, sidecar/README drift guards, `pixi run lint`,
`git diff --check`, and a real docked capture passed before the local
distance-spring commit.

The current continuation completes the next Replay timeline slice for
`rigid_limited_joints`. The shared Replay panel now uses locked-axis error as
its value track and marks locked-error or free-axis-motion frames. Focused
pytest, sidecar/README drift guards, `pixi run lint`, `git diff --check`, and a
real docked capture passed before the local limited-joints commit.

The latest continuation completes the next Replay timeline slice for
`rigid_joint_motor_limits`. The shared Replay panel now uses force travel gap as
its value track and marks velocity-clamp, position-stop, or effort-cap frames.
The maintained workflow capture budget is now 96 frames, so the generated
visual packet reaches the position stop as well as the velocity clamp and
effort-cap lanes. Focused pytest, sidecar/README drift guards, `pixi run lint`,
`git diff --check`, and a real docked capture passed before the local
motor-limits commit.

The current continuation completes the next Replay timeline slice for
`rigid_joint_passive_parameters`. The shared Replay panel now uses armature
position gap as its value track and marks damping energy separation, Coulomb
slip, or armature-lag frames. Focused pytest, sidecar/README drift guards,
`pixi run lint`, `git diff --check`, and a real docked capture passed before
the local passive-parameters commit.

The latest continuation completes the next Replay timeline slice for
`rigid_screw_joint_pitch`. The shared Replay panel now uses coarse/fine travel
gap as its value track and marks pitch-spread, zero-pitch contrast, or
reverse-sign frames. Focused pytest, sidecar/README drift guards,
`pixi run lint`, `git diff --check`, and a real docked capture passed before
the local screw-joint pitch commit.

The current continuation completes the next Replay timeline slice for
`rigid_link_jacobian` after the multibody dynamics-terms and link
center-of-mass slices. The shared Replay panel now uses link-origin speed as
its value track and marks high-twist, wrench-load, world/body Jacobian-gap, or
residual-alert frames. Focused pytest, sidecar/README drift guards, and a real
docked capture passed. `pixi run lint` passed, the focused pytest plus
sidecar/README drift guards passed again after lint, and `git diff --check` was
clean before committing this checkpoint.

The current continuation completes the next Replay timeline slice for
`rigid_multibody_solver_family`. The shared Replay panel now uses residual
solve ratio as its value track and marks solve-advantage, residual-only drift,
or solved-tight frames while residual rows remain loose. Focused pytest and a
real docked capture passed before this hand-off update.

The latest continuation completes the next Replay timeline slice for
`rigid_loop_closure`. The shared Replay panel now uses max closure residual
ratio as its value track and marks solve-advantage,
residual-versus-solved separation, distance-family tip drift, and
rigid-orientation frames. Focused pytest and a real docked capture passed
before the continuation resumed; drift guards, lint, and diff checks are
recorded below.

## Current Branch

`feature/rigid-body-gui-visual-verification`

Current snapshot:

- The latest pushed checkpoint is
  `fbbd5de0005 Expose rigid body impulses in visual workflow`, which records
  the row-5 public direct-impulse slice and broad validation evidence.
- The latest implementation commit for the row-15-through-row-36 pass remains
  `608a8792afa Surface loop closure workflow signals`.
- At the latest recon, the branch was clean, synced with
  `origin/feature/rigid-body-gui-visual-verification`, and had no associated
  pull request.
- `gh pr list --head "$(git branch --show-current)"` reported no pull request.
- The latest user-approved push reported `Everything up-to-date`. Do not push
  again without explicit approval in the session that performs the push.

## Immediate Next Step

A future session should inspect `git status -sb` and `git log -5 --oneline`
first. Expect the latest pushed implementation commit to be
`fbbd5de0005 Expose rigid body impulses in visual workflow`, unless a newer
handoff or cleanup commit has been added.

If the tree is clean with that slice present, the next concrete unblocked step
is maintainer review/acceptance of the current scope or final cleanup in the
same completing PR. Direct rigid-body impulse is covered by row 5; sleep/wake,
island activation, and loop-closure compliance rows remain deferred until
public `dartpy` APIs exist. Use
`docs/dev_tasks/rigid_body_visual_verification/PR_DRAFT.md` as the PR body seed
after push/PR creation is explicitly approved, and check
`docs/dev_tasks/rigid_body_visual_verification/COMPLETION_AUDIT.md` before
claiming completion. Do not push without explicit approval in the session that
performs the push.

Replay capture-metadata checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_capture_metadata_projects_replay_timeline_for_manifests python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_image_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/integration/test_demos_cycle.py::test_rigid_visual_replay_timeline_rows_publish_scene_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_loop_closure_replay_metadata_1781289000
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 36 --workflow-end-row 36 --output-dir /tmp/dart_capture_rigid_workflow_replay_metadata_row36_1781289001
pixi run lint
git diff --check
```

The post-lint focused suite reported `10 passed`. The real single-scene
capture recorded `scene_metadata.replay_timeline.signal_label` as
`Max closure residual ratio` with both signal and markers enabled; the one-row
workflow capture completed row 36 and the generated `review_index.html` showed
`Max closure residual ratio (signal, markers)`.

Full replay-metadata workflow refresh:

```bash
pixi run py-demo-capture -- --rigid-workflow --output-dir /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053
jq -r '.status, .capture_count, .completed_count, .failed_count, .guidance_complete, .guidance_missing_count, .elapsed_s, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/manifest.json
rg -c 'Replay timeline coverage' docs/plans/103-examples-strategy/rigid-body-visual-verification.md
find /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/scenes -name manifest.json -print0 | xargs -0 jq -r 'select(.scene_metadata.replay_timeline != null) | .scene' | wc -l
rg -o '<dt>replay</dt>' /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/review_index.html | wc -l
find /tmp/dart_capture_rigid_workflow_replay_metadata_full_1781284053/scenes -name manifest.json -print0 | xargs -0 jq -r 'select((.capture.converted_frames // 0) <= 0 or .visual_evidence.screenshot.docked_workspace != true or (.visual_evidence.screenshot.unique_rgb_count // 0) <= 1) | [.scene, (.capture.converted_frames|tostring), (.visual_evidence.screenshot.docked_workspace|tostring), (.visual_evidence.screenshot.unique_rgb_count|tostring)] | @tsv'
```

The full refresh reported `status=complete`, `capture_count=36`,
`completed_count=36`, `failed_count=0`, `guidance_complete=true`,
`guidance_missing_count=0`, and elapsed `310.791`. The sidecar,
per-scene manifests, and review index each reported nineteen Replay rows; the
per-scene manifest anomaly query printed no rows.

Optional extended-packet capture refresh:

```bash
DART_PARALLEL_JOBS=4 CTEST_PARALLEL_LEVEL=4 CMAKE_BUILD_PARALLEL_LEVEL=4 pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 37 --workflow-end-row 51 --output-dir /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053
jq -r '.status, .capture_count, .completed_count, .failed_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .include_related, .include_ipc_shelf, .include_packets, .selected_include_related, .selected_include_ipc_shelf, .selected_include_packets, .guidance_complete, .guidance_missing_count, .elapsed_s, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/manifest.json
find /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/scenes -name manifest.json -print0 | xargs -0 jq -r 'select((.capture.converted_frames // 0) <= 0 or .visual_evidence.screenshot.docked_workspace != true or (.visual_evidence.screenshot.unique_rgb_count // 0) <= 1) | [.scene, (.capture.converted_frames|tostring), (.visual_evidence.screenshot.docked_workspace|tostring), (.visual_evidence.screenshot.unique_rgb_count|tostring)] | @tsv'
jq -r '.captures | group_by(.workflow_group)[] | [.[0].workflow_group, length] | @tsv' /tmp/dart_capture_rigid_workflow_optional_rows_37_51_1781285053/manifest.json
```

The optional packet reported `status=complete`, `capture_count=15`,
`completed_count=15`, `failed_count=0`, `workflow_total_count=51`, selected
rows `37-51`, all requested/selected related, IPC-shelf, and packet groups as
`true`, `guidance_complete=true`, `guidance_missing_count=0`, and elapsed
`149.609`. The per-scene manifest anomaly query printed no rows. The group
counts were ten `related_evidence`, four `rigid_ipc_shelf`, and one
`capture_first_packet`.

Passive-parameters checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q
pixi run py-demo-capture -- --scene rigid_joint_passive_parameters --frames 120 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_passive_parameters_timeline_1781277900
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

The focused passive-parameters test reported `1 passed`; the focused
Replay/passive-parameters pytest plus drift guards reported `6 passed`; and
the real docked capture wrote a nonblank 960x540 screenshot with docked UI,
119 PNG frames, and 120 scene-metric events. The manifest recorded row
`rigid_joint_passive_parameters`, spring energy about `2.1583`, damped energy
about `1.3403`, damped-energy ratio about `0.6210`, stiction position `0.0`,
slip position about `0.1742` m, slip speed about `0.7200` m/s, armature
acceleration gap about `2.25` m/s^2, and armature position gap about `0.2614`
m. `pixi run lint` passed and `git diff --check` was clean.

Screw-joint pitch checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_screw_joint_pitch_couples_rotation_and_translation python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_screw_joint_pitch --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_screw_joint_pitch_timeline_1781278553
pixi run lint
git diff --check
```

The focused Replay/screw-joint pytest plus drift guards reported `6 passed`;
the real docked capture wrote a nonblank 960x540 screenshot with docked UI,
95 PNG frames, and 96 scene-metric events. The manifest recorded row
`rigid_screw_joint_pitch`, fine angle about `-0.8317` rad, coarse angle about
`-0.6162` rad, reverse angle about `0.8317` rad, fine axial travel about
`-0.2329` m, coarse axial travel about `-0.3451` m, reverse axial travel about
`-0.2329` m, coarse/fine travel gap about `0.1122` m, and travel-per-radian
values `0.28`, `0.56`, and `-0.28`. `pixi run lint` passed and
`git diff --check` was clean.

Multibody dynamics-terms checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_dynamics_terms_expose_generalized_terms python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_multibody_dynamics_terms --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_dynamics_terms_timeline_1781279136
```

The focused Replay/multibody-dynamics pytest plus drift guards reported
`6 passed`; the real docked capture wrote a nonblank 960x540 screenshot with
docked UI, 95 PNG frames, and 96 scene-metric events. The manifest recorded row
`rigid_multibody_dynamics_terms`, coupled response norm about `15.46`, heavy
response norm about `8.63`, heavy/coupled response ratio about `0.558`,
heavy-minus-coupled torque norm about `18.14`, coupled/heavy off-diagonal
coupling about `0.357`/`1.427`, max impulse residual about `2.7e-14`, and
historical max coupling about `0.373`.

Link center-of-mass checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_center_of_mass_offsets_gravity_torque python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_link_center_of_mass --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_link_center_of_mass_timeline_1781279455
```

The focused Replay/link-center-of-mass pytest plus drift guards reported
`6 passed`; the real docked capture wrote a nonblank 960x540 screenshot with
docked UI, 71 PNG frames, and 72 scene-metric events. The manifest recorded row
`rigid_link_center_of_mass`, mirrored positive/negative angles about
`+/-0.449`, high-inertia angle about `0.153`, centered angle `0.0`, mirrored
torques about `+/-3.182`, high/positive acceleration ratio about `0.370`,
high/positive mass-matrix ratio about `2.948`, and historical max
positive/high-inertia angles about `0.449`/`0.153`.

Link-Jacobian checks for this slice:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_link_jacobian_maps_link_origin_twist_and_wrench python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_link_jacobian --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_link_jacobian_timeline_1781280127
```

The focused Replay/link-Jacobian pytest plus drift guards reported `6 passed`;
the real docked capture wrote a nonblank 960x540 screenshot with docked UI,
95 PNG frames, and 96 scene-metric events. The manifest recorded row
`rigid_link_jacobian`, latest linear/angular speed about `0.5652`/`0.6190`,
finite-difference error about `1.52e-7`, power error `0`, world/body Jacobian
gap about `0.1272`, torques about `-0.8650`/`-0.2949`, matching joint/wrench
power about `-0.4212`, historical max link speed about `0.6677`, historical
max world/body gap about `0.2055`, and historical max absolute torques about
`0.8650`/`0.2949`.

## Context That Would Be Lost

- The Replay capture-metadata slice deliberately serializes only a JSON-safe
  projection of `info["replay_timeline"]`: `signal_label`, `has_signal`,
  `has_markers`, and panel name. Do not try to serialize the `signal` or
  `markers` callables in manifests.
- The capture helper cannot inspect `SceneSetup.info` from the parent workflow
  process because each row capture runs as a child process. The runner emits
  `scene_capture_metadata` through the existing `scene_metrics.jsonl` path, and
  `scripts/capture_py_demo.py` stores the latest metadata event in
  `manifest.json` as `scene_metadata`.
- The completed multibody solver-family Replay timeline slice adds
  `_last_float(...)`,
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_multibody_solver_family.py`. The value
  track is `Residual solve ratio`.
- The row 35 test edit extends
  `test_rigid_multibody_solver_family_routes_solved_closures` with timeline
  label, signal fallback, solve-ratio marker, residual-drift marker,
  tip-error marker, solved-tight marker, and quiet-frame assertions.
- A read-only row 35 explorer recommended computing the signal from latest
  `solve_ratio_history`, falling back to top-level `residual_solve_ratio`, and
  then to the residual-only over solved residual ratio from `last_metrics`. It
  recommended
  markers for ratio at least `1e8`, residual-only drift at least `0.5`, and
  solved-tight frames where solved residual/tip error is at most `1e-8` while
  residual-only rows remain at least `0.25`.
- Sampling during row 35 exploration showed the residual solve ratio climbing
  above `1e12` while solved residuals dropped near machine precision and
  residual-only lanes stayed visibly loose. That sampling was exploratory
  context; final evidence for this slice is recorded in the current validation
  notes.
- The task goal is not just to add examples; it is to make rigid-body GUI demos
  useful as visual debugging and verification surfaces for solvers, backends,
  parameters, contact behavior, constraints, and corner cases.
- Durable planning context lives in
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`.
- That sidecar already documents the 36 rigid workflow rows and current API
  limitations.
- Public `dartpy` now exposes direct rigid-body impulse APIs covered by row 5,
  while sleep/wake, island-activation, and loop-compliance APIs remain
  unavailable. Do not build rows around those deferred features without first
  confirming the API surface changed.
- A read-only explorer subagent was spawned before the stop instruction, but
  the stop instruction superseded waiting for or consuming its result. It later
  returned and identified the workflow-level capture command as the highest
  value bounded follow-up.
- A second read-only explorer identified missing search terms such as
  `RigidBodySolver`, `ContactSolverMethod`, `backend/executor`, and
  `contact model`; those terms are now covered by the workflow search aliases.
- The previous cycle fix changed C++ GUI runner behavior, Python integration
  tests, and docs. It was validated before the stop request, not after.
- The full workflow capture command completed all 36 numbered rigid captures.
  Its top-level manifest reported `status=complete`, `capture_count=36`,
  `completed_count=36`, `failed_count=0`, and all 36 per-scene manifests
  existed.
- The full run exposed parent progress buffering; `scripts/capture_py_demo.py`
  now flushes workflow progress and manifest lines.
- The workflow runner now treats a missing per-scene manifest as
  `failure_reason: missing_manifest` and exits nonzero, even if the child
  process returned 0.
- The in-viewer workflow panel now closes the loop from interactive debugging
  to reproducible visual evidence by printing the row-specific capture command.
- The comparison-contract labels intentionally separate solver family,
  executor-only, contact policy, time-step multiplier, workload shape/contact
  workload, and passive joint parameter family axes so users do not conflate
  backend, solver, policy, and parameter changes.
- The workflow review index is intentionally generated beside the manifest,
  not checked in. `manifest.json` records it under `artifacts.review_index`.
- The related-evidence bundle is intentionally opt-in so default
  `--rigid-workflow` still captures exactly the 36 numbered rows. The
  `--include-related` variant appends the related routes as rows 37-46 in the
  generated review packet.
- The capture-first packet bundle is also opt-in. `--include-packets` appends
  `rigid_ipc_stack_packet` and `rigid_ipc_heavy_stack_packet` after the
  numbered rows and, when present, after related-evidence and direct Rigid IPC
  shelf rows.
- Row-range selection is intended for targeted reruns after a long workflow
  packet fails or needs manual inspection. It keeps absolute row labels and
  output directories, so selected packet row 47 appears as `47/47` and writes to
  `scenes/47_rigid_ipc_stack_packet`.
- The review index now mirrors the numbered rows' in-viewer guidance so the
  capture packet answers "what am I looking at?" without opening the live GUI
  or sidecar table.
- `rigid_ipc_edge_drop` stays outside the numbered World Rigid Body workflow;
  it is an IPC-specific related capability scene, not a new row in the 36-row
  workflow.
- `d5c6de2bee1 Describe optional rigid workflow rows` makes optional
  related-evidence, direct Rigid IPC shelf, and capture-first packet rows
  self-describing in generated manifests and review indexes. It is local and
  unpushed at this checkpoint.
- The current guidance-audit worktree adds `guidance_complete`,
  `guidance_missing_count`, and `guidance_missing_rows` to workflow manifests,
  plus a `guidance` badge and missing-guidance warning in the review index.
- A later read-only explorer recommended this guidance-audit shape:
  non-failing completeness reporting over selected captures, required fields
  for role label, user question, try-first action, inspect signals, healthy
  signal, and scope, plus manifest fields and review-index warning surfacing
  missing guidance.
- The completed live open-command slice adds
  `_rigid_workflow_viewer_command(scene_id, width, height)` in
  `python/examples/demos/runner.py`, writes `viewer_command` entries from
  `scripts/capture_py_demo.py`, renders `open live` and `capture evidence`
  command blocks in generated review cards, and updates
  `python/tests/unit/test_py_demo_panels.py` and
  `python/tests/unit/test_capture_py_demo.py` for panel, manifest,
  review-index, and backend-propagation coverage.
- The latest read-only explorer returned after the stop instruction and was
  later acted on. It recommended a bounded slice for
  `rigid_stack_stability`: add replay-guided timeline metadata for a
  `Top x divergence` value track and diagnostic markers for overlap/collapse,
  low clearance, or visible top-block drift.
- The completed stack Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_stack_stability.py`, with tests covering
  the `Top x divergence` label, signal, and marker thresholds.
- The completed contact-manipulation Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_contact_manipulation.py`, with tests and
  capture evidence for the `Travel divergence` value track and
  contact/proximity/progress markers.
- The completed kinematic-driver Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_kinematic_driver.py`, with tests and
  capture evidence for the `IPC grip box travel` value track and contact,
  carry, slip, and caveat markers.
- The completed normal-push Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_kinematic_normal_push.py`, with tests and
  capture evidence for the `Target travel divergence` value track and contact,
  penetration, SI push, and divergence markers.
- The completed fixed-joint Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_fixed_joint.py`, with tests and capture
  evidence for the `Fixed-joint offset error` value track and pose-error or
  residual-motion recovery markers.
- The completed joint-breakage Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`,
  `replay_capture_state`, `replay_restore_state`, and
  `info["replay_timeline"]` metadata to the shared AVBD breakable fixed-joint
  builder used by `rigid_joint_breakage`, with tests and capture evidence for
  the `Payload release distance` value track and broken/released markers.
- The completed distance-spring Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_distance_spring.py`, with tests for the
  `Max spring stretch` value track and high-stretch or off-center-spin markers.
  It has focused pytest, drift-guard, docked-capture, lint, and diff-check
  evidence.
- The completed limited-joints Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_limited_joints.py`, with tests for the
  `Locked-axis error` value track and locked-error/free-axis-motion markers. It
  has focused pytest, drift-guard, docked-capture, lint, and diff-check
  evidence.
- The completed motor-limits Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_joint_motor_limits.py`, with tests for the
  `Force travel gap` value track and velocity-clamp, position-stop, and
  effort-cap markers. It has focused pytest, drift-guard, docked-capture, lint,
  and diff-check evidence.
- The completed passive-parameters Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_joint_passive_parameters.py`, with tests
  for the `Armature position gap` value track and damping, Coulomb slip, and
  armature-lag markers. It has focused pytest, drift-guard, docked-capture,
  lint, and diff-check evidence.
- The completed screw-joint pitch Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_screw_joint_pitch.py`, with tests for the
  `Coarse/fine travel gap` value track and pitch-spread, zero-pitch contrast,
  and reverse-sign markers. It has focused pytest, drift-guard,
  docked-capture, lint, and diff-check evidence.
- The completed link-Jacobian Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_link_jacobian.py`, with tests for the
  `Link-origin speed` value track and high-twist, wrench-load, world/body-gap,
  and residual-alert markers.
- The completed multibody solver-family Replay timeline slice adds
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata to
  `python/examples/demos/scenes/rigid_multibody_solver_family.py`, with tests
  for the `Residual solve ratio` value track and solve-advantage,
  residual-only-drift, tip-error, solved-tight, fallback, and quiet-frame
  behavior. Its real docked capture wrote 71 PNG frames and 72 scene-metric
  events under
  `/tmp/dart_capture_multibody_solver_family_timeline_1781281303`.
- The completed row 36 slice in
  `python/examples/demos/scenes/rigid_loop_closure.py` adds `_last_float(...)`,
  `replay_timeline_signal(...)`, `replay_timeline_marker(...)`, and
  `info["replay_timeline"]` metadata. The intended value track is
  `Max closure residual ratio`.
- The row 36 test edit in
  `python/tests/integration/test_demos_cycle.py` extends
  `test_rigid_loop_closure_compares_closure_families` with label, current
  snapshot signal, top-level and `last_metrics` signal fallback, solve-ratio
  marker, residual marker, rigid-orientation marker, distance-tip marker, and
  quiet-frame assertions.
- Row 36 marker intent: flag large closure solve advantage, residual-versus-
  solved closure separation, distance-family tip drift with near-zero distance
  error, and rigid-orientation solve frames.

## How To Resume

```bash
git checkout feature/rigid-body-gui-visual-verification
git status -sb
git log -3 --oneline
```

Then read:

```bash
docs/ai/principles.md
docs/dev_tasks/README.md
docs/dev_tasks/rigid_body_visual_verification/README.md
docs/plans/103-examples-strategy/rigid-body-visual-verification.md
```

If committing in the next session, first run the repository-mandated lint:

```bash
pixi run lint
```

Current multibody solver-family validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures -q
pixi run py-demo-capture -- --scene rigid_multibody_solver_family --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_multibody_solver_family_timeline_1781281303
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_multibody_solver_family_routes_solved_closures python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
```

The row 35 pytest reported `1 passed`; the focused Replay/row 35 pytest plus
sidecar/README/capture-command drift guards reported `6 passed`; and the
capture wrote a nonblank 960x540 docked screenshot, 71 PNG frames, and
72 scene-metric events under
`/tmp/dart_capture_multibody_solver_family_timeline_1781281303`. `pixi run
lint` passed.

Current loop-closure validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families -q
pixi run py-demo-capture -- --scene rigid_loop_closure --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_loop_closure_timeline_1781285000
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_loop_closure_compares_closure_families python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

The row 36 focused pytest reported `1 passed`; the focused Replay/row 36
pytest plus sidecar/README/capture-command drift guards reported `6 passed`;
the real docked capture wrote a nonblank 960x540 docked screenshot, 71 PNG
frames, and 72 scene-metric events under
`/tmp/dart_capture_loop_closure_timeline_1781285000`. The manifest recorded row
`rigid_loop_closure`, solver
`variational_rigid_multibody_loop_closure`, scope
`point_distance_rigid_closure_family_selection`, executor `Sequential`, gravity
scale `1.0`, six cases, point/distance/rigid residual ratios about
`7.595e11`, `7.458e11`, and `7.740e11`, rigid residual orientation error about
`0.149`, solved orientation error near `2.8e-17`, distance solved distance
error near `9.4e-15`, and distance solved tip error about `0.439`.
`pixi run lint` passed and `git diff --check` was clean.

Focused validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
```

The focused pytest reported `6 passed`. The dry-run manifest reported workflow
`rigid_visual_verification`, status `planned`, and 36 planned captures.

Current-continuation validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
pixi run lint
git diff --check
```

The focused pytest reported `7 passed`. The dry-run command printed all
36 planned capture commands. `pixi run lint` passed and `git diff --check` was
clean.

Latest focused validation already run:

```bash
pixi run lint
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_guides_expose_capture_commands python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
test -f /tmp/dart_capture_rigid_workflow_dry_run/review_index.html && jq -r '.artifacts.review_index, .capture_count, .status' /tmp/dart_capture_rigid_workflow_dry_run/manifest.json && rg -n "DART rigid workflow review index|rigid_loop_closure" /tmp/dart_capture_rigid_workflow_dry_run/review_index.html
git diff --check
```

The focused pytest reported `14 passed`. The dry-run printed all 36 planned
capture commands. The dry-run manifest reported
`/tmp/dart_capture_rigid_workflow_dry_run/review_index.html`, `36`, and
`planned`, and the generated index contained the final
`36/36 rigid_loop_closure` row. `pixi run lint` passed and `git diff --check`
was clean.

Full workflow capture with the review index already run:

```bash
pixi run py-demo-capture -- --rigid-workflow --output-dir /tmp/dart_capture_rigid_workflow_full_review_index_1781259714
```

The command exited 0. The manifest at
`/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/manifest.json`
reported `status=complete`, `capture_count=36`, `completed_count=36`,
`failed_count=0`, and `elapsed_s=314.278`. Every capture entry had
`return_code=0`, `status=captured`, and `manifest_exists=true`. The generated
review index at
`/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/review_index.html`
contained 36 screenshot thumbnails and linked the first/last screenshots plus
comparison-axis and held-fixed metric summaries for representative rows.

Current related-search routing validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves -q
pixi run lint
git diff --check
```

The focused pytest reported `3 passed`. `pixi run lint` passed and
`git diff --check` was clean.

Current related-evidence capture-bundle validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_include_related_requires_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-related --dry-run --output-dir /tmp/dart_capture_rigid_workflow_related_dry_run_current
jq -r '.include_related, .capture_count, .captures[36].workflow_group, .captures[36].scene, .captures[45].scene, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_related_dry_run_current/manifest.json
rg -n "related_evidence|avbd_rigid_prismatic_motor|46/46" /tmp/dart_capture_rigid_workflow_related_dry_run_current/review_index.html
```

The focused pytest reported `5 passed`. The public dry-run completed with
46 planned capture commands, the manifest reported `include_related=true` and
`capture_count=46`, and the generated review index contained the final
`46/46 avbd_rigid_prismatic_motor` related-evidence row.

Current capture-first packet bundle validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_packets_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --dry-run --output-dir /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run
jq -r '.include_related, .include_packets, .capture_count, .captures[46].workflow_group, .captures[46].scene, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run/manifest.json
rg -n "capture_first_packet|rigid_ipc_stack_packet|47/47" /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run/review_index.html
```

The focused pytest reported `8 passed`. The public dry-run completed with
47 planned capture commands, the manifest reported `include_related=true`,
`include_packets=true`, `capture_count=47`, and the generated review index
contained the final `47/47 rigid_ipc_stack_packet` capture-first packet row.

Current row-range workflow rerun validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --workflow-start-row 47 --workflow-end-row 47 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run
jq -r '.capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].order, .captures[0].count, .captures[0].scene, .captures[0].workflow_group' /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run/manifest.json
rg -n "47/47|rigid_ipc_stack_packet|capture_first_packet" /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run/review_index.html
```

The focused pytest reported `11 passed`. The public dry-run completed with
one selected row-47 command, the manifest reported `capture_count=1`,
`workflow_total_count=47`, selected row order/count `47/47`, and the generated
review index contained the absolute `47/47 rigid_ipc_stack_packet` row.

Current review-index guidance validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_guidance_dry_run
jq -r '.captures[0].workflow_label, .captures[0].user_question, .captures[0].try_first, .captures[0].healthy_signal, .captures[0].scope' /tmp/dart_capture_rigid_workflow_guidance_dry_run/manifest.json
rg -n "How do the rigid method families differ visually|Healthy: wall clearance|Generic thin-wall comparison" /tmp/dart_capture_rigid_workflow_guidance_dry_run/review_index.html
```

The focused pytest reported `11 passed`. The public row-15 dry-run completed,
the manifest reported the `Solver family` role plus the maintained user
question, try-first guidance, healthy signal, and scope note, and the generated
review index contained the same solver-family guidance beside the row card.

Current rigid IPC edge-drop validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_edge_drop_reports_degenerate_contact_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_routes_publish_self_describing_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches -q
```

The focused pytest reported `9 passed`. The new edge-drop route reports
`row=rigid_ipc_edge_drop`, `related_source_row=rigid_solver_compare`,
`solver=rigid_ipc`, `scope=degenerate_edge_contact_capability`, near-barrier
clearance, and nonzero angular/tilt motion. The related route table, capture
command list, capture-helper specs, panel coverage, search routing, direct
related route, and unnumbered-route guard all include `rigid_ipc_edge_drop`.
The public `--include-related` dry-run reported `capture_count=46`,
`40/46 rigid_ipc_edge_drop`, and final `46/46 avbd_rigid_prismatic_motor`. The
public row-range packet dry-run reported `workflow_total_count=47` and
`47/47 rigid_ipc_stack_packet`. The real docked capture wrote a nonblank
960x540 screenshot with docked UI, 71 converted PNG frames, latest status
`edge-barrier`, minimum barrier gap about `0.000384` m, maximum tilt about
`55.33` degrees, and maximum angular speed about `0.555` rad/s.

Current Rigid IPC shelf metrics validation already run:

```bash
pixi run lint
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_shelf_scenes_report_capture_metrics python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q
pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_pile_metrics_current
git diff --check
```

`pixi run lint` passed. The focused pytest reported `2 passed`. The direct
Rigid IPC shelf scenes
`rigid_ipc`, `rigid_ipc_slide`, `rigid_ipc_incline`, and `rigid_ipc_pile` now
report scene-owned capture metrics for row identity, IPC solver label, scope,
time-step/world-time, friction, status, contact counts, step timing, and
scene-specific gap, speed, travel, height, span, or pile summaries while
preserving shared replay controls.
A real docked
`pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_pile_metrics_current`
capture wrote a 960x540 screenshot with docked UI detected, 23 converted PNG
frames, 24 scene-metric events, latest row `rigid_ipc_pile`, scope
`multi_box_barrier_pile`, 25 history samples, `box_count=3`, maximum history
speed about `1.177` m/s, and minimum history clearance about `0.276` m.
`git diff --check` was clean.

Current direct Rigid IPC shelf capture-bundle validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_direct_ipc_shelf python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_direct_ipc_shelf_captures_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-ipc-shelf --dry-run --output-dir /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .capture_count, .workflow_total_count, .captures[36].workflow_group, .captures[36].scene, .captures[39].scene' /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current/manifest.json
rg -n "37/40 rigid_ipc|40/40 rigid_ipc_pile|rigid_ipc_shelf" /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current/review_index.html
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 47 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .capture_count, .workflow_total_count, .captures[0].order, .captures[0].workflow_group, .captures[0].scene, .captures[3].scene, .captures[4].workflow_group, .captures[4].scene' /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current/manifest.json
rg -n "47/51 rigid_ipc|50/51 rigid_ipc_pile|51/51 rigid_ipc_stack_packet|rigid_ipc_shelf" /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current/review_index.html
```

The focused pytest reported `11 passed`. The public `--include-ipc-shelf`
dry-run completed with 40 planned captures, the manifest reported
`include_ipc_shelf=true`, `capture_count=40`, `workflow_total_count=40`, first
IPC shelf scene `rigid_ipc`, final IPC shelf scene `rigid_ipc_pile`, and
`workflow_group=rigid_ipc_shelf`. The generated review index contained
`37/40 rigid_ipc`, `40/40 rigid_ipc_pile`, and the `rigid_ipc_shelf` group. The
combined row-range dry-run with related evidence, IPC shelf rows, and packets
selected rows 47-51; it reported the direct IPC shelf rows as absolute 47-50
and `rigid_ipc_stack_packet` as the explicit combined-mode row 51.

Current workflow manifest and review index metadata validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_direct_ipc_shelf python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_range_preserves_requested_extra_groups python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 47 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_requested_groups_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .selected_include_related, .selected_include_ipc_shelf, .selected_include_packets, .capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].workflow_group, .captures[0].scene, .captures[4].workflow_group, .captures[4].scene' /tmp/dart_capture_rigid_workflow_requested_groups_dry_run_current/manifest.json
rg -n "47/51 rigid_ipc|50/51 rigid_ipc_pile|51/51 rigid_ipc_stack_packet|rigid_ipc_shelf|capture_first_packet" /tmp/dart_capture_rigid_workflow_requested_groups_dry_run_current/review_index.html
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_range_preserves_requested_extra_groups python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 47 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_review_index_groups_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .selected_include_related, .selected_include_ipc_shelf, .selected_include_packets, .capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end' /tmp/dart_capture_rigid_workflow_review_index_groups_dry_run_current/manifest.json
rg -n "<strong>rows</strong> 47-51 / 51|<strong>requested groups</strong> numbered, related, ipc shelf, packets|<strong>selected groups</strong> ipc shelf, packets|47/51 rigid_ipc|51/51 rigid_ipc_stack_packet" /tmp/dart_capture_rigid_workflow_review_index_groups_dry_run_current/review_index.html
pixi run lint
git diff --check
```

The focused pytest reported `16 passed` for manifest metadata and `4 passed`
for the review-index metadata follow-up. The public combined row-range dry-run
selected rows 47-51 and wrote requested flags `include_related=true`,
`include_ipc_shelf=true`, and `include_packets=true` while separately
reporting `selected_include_related=false`,
`selected_include_ipc_shelf=true`, and `selected_include_packets=true`.
The manifest also reported `capture_count=5`, `workflow_total_count=51`,
`workflow_row_start=47`, `workflow_row_end=51`, first selected scene
`rigid_ipc`, and final selected scene `rigid_ipc_stack_packet`. The generated
review index header contained row-span `47-51 / 51`, requested-groups
`numbered, related, ipc shelf, packets`, and selected-groups
`ipc shelf, packets`. `pixi run lint` passed and `git diff --check` was clean.

Current workflow panel packet-command validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_guides_expose_capture_commands python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_route_rows_request_scene_switches -q
pixi run lint
```

The focused panel pytest reported `3 passed`. The tested `Rigid Workflow`
panel now emits the per-row capture command plus a `Review packet` section
containing the full numbered workflow command, the current-row row-range rerun
command, and the extended related/IPC-shelf/packet command. `pixi run lint`
passed. `git diff --check` was not rerun after the final handoff edits because
the user explicitly requested no further verification.

Current workflow video packet validation already run:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_visual_capture_manifest_records_video_artifact python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_request_video_commands python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/integration/test_demos_cycle.py::test_rigid_visual_motion_capture_video_flags_are_documented -q
DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --video --fps 24 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_video_dry_run_current
jq -r '.capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].command' /tmp/dart_capture_rigid_workflow_video_dry_run_current/manifest.json
rg -n "15/36 rigid_solver_compare|--video --fps 24|selected groups|requested groups" /tmp/dart_capture_rigid_workflow_video_dry_run_current/review_index.html
pixi run lint
git diff --check
```

The focused pytest reported `5 passed`. The public row-15 workflow dry-run
completed with exit code 0, generated a selected-row command ending in
`--show-ui --video --fps 24`, and wrote a manifest with `capture_count=1`,
`workflow_total_count=36`, `workflow_row_start=15`, and `workflow_row_end=15`.
The generated review index contained row `15/36 rigid_solver_compare`,
requested/selected group badges, and the `--video --fps 24` command.
`pixi run lint` passed and `git diff --check` was clean.

Current guidance-audit validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_direct_ipc_shelf python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_range_preserves_requested_extra_groups python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_manifest_reports_missing_guidance -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 37 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_guidance_audit_dry_run_current
jq -r '.guidance_complete, .guidance_missing_count, (.guidance_missing_rows | length), .capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].workflow_label, .captures[10].workflow_label, .captures[14].workflow_label' /tmp/dart_capture_rigid_workflow_guidance_audit_dry_run_current/manifest.json
rg -n "<strong>guidance</strong> complete|Rows Missing Guidance|Related evidence|Rigid IPC shelf|Capture-first packet|37/51 floating_base|51/51 rigid_ipc_stack_packet" /tmp/dart_capture_rigid_workflow_guidance_audit_dry_run_current/review_index.html
```

The focused pytest reported `7 passed`. The public dry-run selected rows 37-51
from the fully extended packet and wrote `capture_count=15`,
`workflow_total_count=51`, `workflow_row_start=37`, and `workflow_row_end=51`.
The generated manifest reported `guidance_complete=true`,
`guidance_missing_count=0`, and an empty `guidance_missing_rows` list, with
row labels for `Related evidence`, `Rigid IPC shelf`, and
`Capture-first packet`. The generated review index contained
`<strong>guidance</strong> complete`, the absolute `37/51 floating_base` and
`51/51 rigid_ipc_stack_packet` rows, and no `Rows Missing Guidance` warning.

Current live open-command validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_request_video_commands python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_full_extended_plan_has_complete_guidance python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_manifest_reports_missing_guidance -q
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_live_command_dry_run_current_1781271290
jq -r '.capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].scene, .captures[0].viewer_command, .captures[0].command' /tmp/dart_capture_rigid_workflow_live_command_dry_run_current_1781271290/manifest.json
rg -n "open live|capture evidence|pixi run py-demos -- --scene rigid_solver_compare --width 960 --height 540|pixi run py-demo-capture -- --scene rigid_solver_compare" /tmp/dart_capture_rigid_workflow_live_command_dry_run_current_1781271290/review_index.html
pixi run lint
```

The focused pytest reported `5 passed` before and after `pixi run lint`
reformatted `scripts/capture_py_demo.py`. The public row-15 dry-run wrote
`capture_count=1`, `workflow_total_count=36`, `workflow_row_start=15`,
`workflow_row_end=15`, scene `rigid_solver_compare`, viewer command
`pixi run py-demos -- --scene rigid_solver_compare --width 960 --height 540`,
and the paired `pixi run py-demo-capture -- --scene rigid_solver_compare`
capture command. The generated `review_index.html` contained `open live`,
`capture evidence`, the live viewer command, and the capture command.
`pixi run lint` passed.

Current stack Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_stack_stability_keeps_ipc_stack_ordered python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_stack_stability --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_stack_stability_timeline_1781271696
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.top_x_divergence, .scene_metrics.latest.metrics.history.max_top_x_divergence, .scene_metrics.latest.metrics.history.ipc_min_clearance' /tmp/dart_capture_stack_stability_timeline_1781271696/manifest.json
```

The focused pytest reported `2 passed`. The real docked capture completed with
exit code 0 and wrote a nonblank 960x540 screenshot with docked UI detected,
23 PNG frames, and 24 scene-metric events. The capture manifest reported row
`rigid_stack_stability`, current and historical maximum top-x divergence about
`0.00405`, and IPC minimum clearance `0.0`.

Current contact-manipulation Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_contact_manipulation_pushes_target_toward_goal python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_contact_manipulation --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_contact_manipulation_timeline_1781272246
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.travel_divergence, .scene_metrics.latest.metrics.history.max_travel_divergence, .scene_metrics.latest.metrics.history.ipc_max_target_travel, .scene_metrics.latest.metrics.history.sequential_impulse_min_gap' /tmp/dart_capture_contact_manipulation_timeline_1781272246/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

The focused Replay/contact pytest reported `2 passed`. The real docked capture
completed with exit code 0 and wrote a nonblank 960x540 screenshot with docked
UI detected, 71 PNG frames, and 72 scene-metric events. The manifest reported
row `rigid_contact_manipulation`, current and historical maximum travel
divergence about `0.0217`, IPC target travel about `0.2243` m, and
sequential-impulse minimum gap about `-0.0017` m. The
sidecar/README/capture-command drift guard reported `4 passed`.

Current kinematic-driver Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_driver_carries_box_with_ipc python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_kinematic_driver --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_kinematic_driver_timeline_1781273002
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.ipc_grip_box_travel, .scene_metrics.latest.metrics.history.ipc_grip_max_box_travel, .scene_metrics.latest.metrics.history.ipc_slip_max_slip, .scene_metrics.latest.metrics.history.si_caveat_max_abs_driver_travel' /tmp/dart_capture_kinematic_driver_timeline_1781273002/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

The focused Replay/kinematic pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 71 PNG frames, and 72 scene-metric events. The manifest
reported row `rigid_kinematic_driver`, current and historical IPC grip box
travel about `0.0896` m, IPC slip maximum about `0.1008` m, and
sequential-impulse caveat driver travel `0.0`. The sidecar/README/capture
command drift guard reported `4 passed`. `pixi run lint` passed and
`git diff --check` was clean.

Current normal-push Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_kinematic_normal_push_exposes_normal_pusher_caveat python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_kinematic_normal_push --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_kinematic_normal_push_timeline_1781273413
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.lanes.ipc_normal.status, .scene_metrics.latest.metrics.lanes.ipc_normal.max_depth, .scene_metrics.latest.metrics.lanes.ipc_normal.target_travel, .scene_metrics.latest.metrics.lanes.si_caveat.status, .scene_metrics.latest.metrics.lanes.si_caveat.target_travel, .scene_metrics.latest.metrics.lanes.si_caveat.analytic_gap' /tmp/dart_capture_kinematic_normal_push_timeline_1781273413/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

The focused Replay/normal-push pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 71 PNG frames, and 72 scene-metric events. The manifest
reported row `rigid_kinematic_normal_push`, IPC normal status
`ipc penetration caveat`, IPC normal max depth about `0.125` m, near-zero IPC
target travel, SI status `pushed`, SI target travel about `0.123` m, and SI
analytic gap about `-0.00055` m. The sidecar/README/capture-command drift
guard reported `4 passed`.

Current fixed-joint Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_fixed_joint_verifier_restores_captured_transform python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_fixed_joint --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_fixed_joint_timeline_1781274052
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.metrics.translation_error, .scene_metrics.latest.metrics.metrics.orientation_error, .scene_metrics.latest.metrics.metrics.payload_speed, .scene_metrics.latest.metrics.history.max_translation_error, .scene_metrics.latest.metrics.history.max_orientation_error' /tmp/dart_capture_fixed_joint_timeline_1781274052/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
```

The focused Replay/fixed-joint pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 23 PNG frames, and 24 scene-metric events. The manifest
reported row `rigid_fixed_joint`, final translation error about `2.17e-10` m,
final orientation error `0.0` rad, final payload speed `0.0` m/s, historical
maximum translation error about `1.09e-8` m, and historical maximum orientation
error about `2.67e-7` rad. The sidecar/README/capture-command drift guard
reported `4 passed`. `pixi run lint` passed and `git diff --check` was clean.

Current joint-breakage Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_breakage_marks_and_resets_breakage python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_joint_breakage --frames 48 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_breakage_timeline_1781274802
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.metrics.status, .scene_metrics.latest.metrics.metrics.broken, .scene_metrics.latest.metrics.metrics.payload_release_distance, .scene_metrics.latest.metrics.metrics.payload_speed, .scene_metrics.latest.metrics.history.saw_broken, .scene_metrics.latest.metrics.history.max_payload_release_distance' /tmp/dart_capture_joint_breakage_timeline_1781274802/manifest.json
```

The focused Replay/joint-breakage pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 47 PNG frames, and 48 scene-metric events. The manifest
reported row `rigid_joint_breakage`, status `broken`, broken flag `1.0`, final
payload release distance about `0.413` m, final payload speed about `1.332`
m/s, `saw_broken` `1.0`, and historical maximum payload release distance about
`0.590` m. The sidecar/README/capture-command drift guard reported `4 passed`.
`pixi run lint` passed and `git diff --check` was clean.

Current distance-spring Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_distance_spring_reduces_stretch_and_spins_offset_anchor python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_distance_spring --frames 72 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_distance_spring_timeline_1781275675
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.lanes.free.stretch, .scene_metrics.latest.metrics.lanes.soft.stretch, .scene_metrics.latest.metrics.lanes.stiff.stretch, .scene_metrics.latest.metrics.lanes.offset.stretch, .scene_metrics.latest.metrics.lanes.offset.angular_speed, .scene_metrics.latest.metrics.history.max_sprung_abs_stretch, .scene_metrics.latest.metrics.history.max_offset_angular_speed' /tmp/dart_capture_distance_spring_timeline_1781275675/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

The focused Replay/distance-spring pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 71 PNG frames, and 72 scene-metric events. The manifest
reported row `rigid_distance_spring`, final unsprung stretch `0.330` m,
soft/stiff stretch about `-0.113` m and `-0.125` m, offset stretch about
`-0.001` m, offset angular speed about `1.956` rad/s, historical maximum spring
stretch about `0.346` m, and historical maximum offset angular speed about
`21.04` rad/s. The sidecar/README/capture-command drift guard reported
`4 passed`. `pixi run lint` passed and `git diff --check` was clean.

Current limited-joints Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_one_dof_joint_verifier_preserves_locked_directions python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata -q
pixi run py-demo-capture -- --scene rigid_limited_joints --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_limited_joints_timeline_1781276042
jq -r '.scene, .capture.converted_frames, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.metrics.hinge_radius_error, .scene_metrics.latest.metrics.metrics.hinge_z_error, .scene_metrics.latest.metrics.metrics.hinge_yaw, .scene_metrics.latest.metrics.metrics.slider_orthogonal_error, .scene_metrics.latest.metrics.metrics.slider_axis_travel, .scene_metrics.latest.metrics.history.max_hinge_radius_error, .scene_metrics.latest.metrics.history.max_hinge_z_error, .scene_metrics.latest.metrics.history.max_slider_orthogonal_error, .scene_metrics.latest.metrics.history.max_abs_hinge_yaw, .scene_metrics.latest.metrics.history.max_slider_axis_travel' /tmp/dart_capture_limited_joints_timeline_1781276042/manifest.json
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run lint
git diff --check
```

The focused Replay/limited-joints pytest reported `2 passed`. The real docked
capture completed with exit code 0 and wrote a nonblank 960x540 screenshot with
docked UI detected, 23 PNG frames, and 24 scene-metric events. The manifest
reported row `rigid_limited_joints`, hinge radius error `0.0`, hinge z error
`0.0`, hinge yaw about `0.168` rad, slider orthogonal error `0.0`, slider axis
travel about `0.604` m, zero historical locked-axis error, historical maximum
hinge yaw about `0.168` rad, and historical maximum slider axis travel about
`0.604` m. The sidecar/README/capture-command drift guard reported `4 passed`.
`pixi run lint` passed and `git diff --check` was clean.

Current motor-limits Replay timeline validation:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_joint_motor_limits_clamp_commands_and_effort python/tests/unit/test_py_demo_panels.py::test_shared_replay_panel_uses_scene_replay_timeline_metadata python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_guidance_matches_sidecar python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --scene rigid_joint_motor_limits --frames 96 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_joint_motor_limits_timeline_1781276821
jq -r '.scene, .capture.requested_frames, .capture.converted_frames, .show_ui, .visual_evidence.screenshot.docked_workspace, .visual_evidence.screenshot.unique_rgb_count, .scene_metrics.event_count, .scene_metrics.latest.metrics.row, .scene_metrics.latest.metrics.motor_speed, .scene_metrics.latest.metrics.motor_expected_speed, .scene_metrics.latest.metrics.motor_speed_error, .scene_metrics.latest.metrics.position_limit_angle, .scene_metrics.latest.metrics.position_limit_upper, .scene_metrics.latest.metrics.position_limit_error, .scene_metrics.latest.metrics.force_position_gap, .scene_metrics.latest.metrics.force_acceleration_gap, .scene_metrics.latest.metrics.history.max_force_position_gap, .scene_metrics.latest.metrics.history.max_force_acceleration_gap' /tmp/dart_capture_joint_motor_limits_timeline_1781276821/manifest.json
pixi run lint
git diff --check
```

The focused Replay/motor-limits pytest and drift guards reported `6 passed`.
The real docked capture completed with exit code 0 and wrote a nonblank
960x540 screenshot with docked UI detected, 95 PNG frames, and
96 scene-metric events. The maintained workflow capture budget was raised from
72 to 96 frames because the 72-frame packet showed velocity clamp and effort
gap, but did not reach the position stop. The 96-frame packet reached the stop
with `position_limit_angle` equal to `position_limit_upper` (`0.35` rad), motor
speed and expected speed `0.3` m/s, motor speed error `0.0`, position-limit
error `0.0`, force travel gap about `0.6984` m, and force acceleration gap
about `6.0` m/s^2. `pixi run lint` passed and `git diff --check` was clean.
