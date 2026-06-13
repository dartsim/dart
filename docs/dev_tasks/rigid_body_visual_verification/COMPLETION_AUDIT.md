# Rigid Body Visual Verification Completion Audit

Date: 2026-06-13

This audit maps the active rigid-body GUI visual-verification objective to
current repository evidence. It is intentionally conservative: the dev task is
locally review-ready, but not complete until maintainer acceptance and the
completion PR cleanup happen.

Latest implementation slice: rigid workflow search now covers more terms
users type while diagnosing performance, contact/material parameters,
collision queries, dynamics terms, impulses, stack behavior, and closed
chains. `throughput`/`perf`/`fps` route to `rigid_contact_scale_budget`;
`latency`, `wall time`, `step timing`, and `worker threads` route to
`rigid_step_diagnostics`; `dt`, `substeps`, and `gravity tuning` route to
`rigid_timestep_sensitivity`; friction and restitution coefficient terms route
to the material/restitution rows; ray/sweep/time-of-impact terms route to
`rigid_collision_casts`; `CollisionQueryOptions` and collision-filter terms
route to `rigid_collision_query_options`; static-friction terms route to
`rigid_friction_threshold`; kinetic/dynamic-friction terms route to
`rigid_spin_roll_coupling`; `compute_impulse_response`, `impulse response`,
`generalized force`, `coriolis`, and `mass_matrix` route to
`rigid_multibody_dynamics_terms`; direct `RigidBody` impulse/load API terms
route to `rigid_external_loads`; stack-jitter/resting-stack terms route to
`rigid_stack_stability`; and closed-chain/closed-loop terms route to
`rigid_loop_closure`. The search now folds punctuation, underscores, hyphens,
dotted API names, CamelCase, compact API tokens, and simple plurals, so
`RigidBody.applyLinearImpulse`, `Multibody.computeImpulseResponse`,
`ray-cast`, `shape-cast`, `body kind filters`, and `resting contacts` route
like their documented snake_case or singular forms. Search-result tooltips now
name whether the row matched by maintained alias, row number, user question,
related evidence, or scope caveat, and no-result feedback suggests row
numbers, scene ids, solver, contact, backend, or API names. Focused
validation:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_question_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_filters_rows_by_row_id_and_requests_scene_switch python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_explains_empty_search_results python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_user_terminology_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_routes_deferred_api_terms python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `11 passed`.

Latest pushed implementation slice: rigid workflow search now routes GPU/CUDA
shorthand (`GPU`, `CUDA`, `GPU backend`, `CUDA backend`, `GPU acceleration`,
`CUDA acceleration`) to `rigid_step_diagnostics`, where users can inspect
backend status and fallback without implying a separate rigid CUDA solver row.
It also routes multibody solver-family terms (`semi implicit`,
`semi-implicit`, `variational solver`) to `rigid_multibody_solver_family`, and
passive-joint parameter terms (`joint damping`, `joint friction`,
`joint stiffness`, `joint armature`) to `rigid_joint_passive_parameters`.
Focused validation:
`PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_multibody_and_passive_parameter_aliases python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order -q`
reported `3 passed`. This slice was pushed at `cb5deee7bfd`.

Latest local implementation slice: row 5, `rigid_external_loads`, now covers
public direct rigid-body impulse behavior. The C++ `RigidBody` API and dartpy
bindings expose `apply_linear_impulse()` and `apply_angular_impulse()`;
the scene adds direct linear/angular impulse lanes and sliders; capture metrics
publish impulse magnitudes, momentum, and speed; and the PLAN-103 sidecar plus
py-demos README route `direct rigid body impulse` to the public load/impulse
row while preserving the sleep/wake, island activation, and loop-closure
compliance deferrals. Focused validation: `pixi run build-py-dev-docking`
rebuilt `dartpy`; the focused pytest set for the impulse API, row-5 scene
metrics, API/docs drift guard, panel search routing, and capture-card latest
signals reported `6 passed`; and
`/tmp/dart_capture_rigid_external_loads_impulses_1781345833` contains a
nonempty 640x360 screenshot, 11 PNG frames, 12 scene-metrics events, complete
resolved solver identity (`solver=sequential_impulse`, `executor=Sequential`),
and latest metrics with `scope=external_load_and_direct_impulse_response`,
`linear_impulse_momentum_x=0.8`, `linear_impulse_speed=0.8`,
`angular_impulse_momentum_z=0.09`, `angular_impulse_speed=2.0`, and
`static_drift=0.0`. This slice has been pushed at `fbbd5de0005` and covered by
the broad default and CUDA wrapper validations recorded below.

Latest local implementation slice: workflow review-card `latest signals` now
promote optional-row health signals for World related, focused Rigid IPC,
direct IPC shelf, capture-first stack packet, differentiable contact-gradient,
and AVBD related joint/motor rows. Focused capture-helper guards reported
`9 passed`. Fresh optional rows 37-53 workflow evidence lives at
`build/captures/rigid_workflow_optional_signal_highlights_1781338541`: the
packet completed with `status=complete`, `capture_count=17`,
`completed_count=17`, `failed_count=0`, `guidance_complete=true`,
`scene_metrics_complete=true`, `scene_metrics_count=17`,
`resolved_solver_identity_complete=true`, `resolved_solver_identity_count=17`,
and `review_index.html` showing promoted optional signals such as linear
speed, min tunnel margin, capture-first/frame-budget stack metrics,
differentiable fallback metrics, and AVBD target/measured speed. A static
review-index audit found 86/86 local links present.

Latest local implementation slice: the full numbered rows 01-36 workflow
packet was regenerated after the backend-diagnostics and contact-query
review-card improvements. `build/captures/rigid_workflow_rows_01_36_1781335894`
reports `status=complete`, `capture_count=36`, `completed_count=36`,
`failed_count=0`, `guidance_complete=true`, `scene_metrics_complete=true`,
`scene_metrics_count=36`, `resolved_solver_identity_complete=true`,
`resolved_solver_identity_count=36`, and 2388 PNG frames. A read-only static
review audit checked 181 local `href`/`src` targets in `review_index.html` with
0 missing links, and the refreshed review cards show row-8
`backend diagnostics` plus row 12-14 contact-query/collision-cast
`latest signals`. The in-viewer resilient packet command also now matches the
durable docs at `/tmp/dart_capture_rigid_workflow_resilient`.

Latest local implementation slice: workflow review-card `latest signals` now
surface the baseline and free-flight values needed for first-screen static
review. Row 1 promotes baseline max speed, minimum height, kinetic energy,
contact count, step time, and dynamic body count from capture metrics; row 3
promotes zero-g drift error, momentum drift, drift speed, gravity-arc position
error, momentum residual, energy drift, height, and spin momentum ratio.
Focused capture-helper and baseline scene guards reported `6 passed`. Fresh
rows 1-3 workflow evidence lives at
`build/captures/rigid_workflow_baseline_freeflight_signals_1781337861`: the
packet completed with `status=complete`, `capture_count=3`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`resolved_solver_identity_complete=true`, and `review_index.html` showing the
new baseline/free-flight latest signals.

Latest local implementation slice: workflow review-card `latest signals` now
surface the core body/frame/load values needed for static review of rows 2 and
4-6. Row 2 promotes dynamic displacement/height/speed, kinematic path error/x,
and static drift; row 4 promotes transform residuals, sensor position, and
parent frame; row 5 promotes force- and inertia-scaled acceleration plus static
drift; and row 6 promotes centered/doubled/local-frame acceleration,
off-center yaw acceleration, and pulse count. Focused capture-helper guards
reported `6 passed`. Fresh rows 2-6 workflow evidence lives at
`build/captures/rigid_workflow_core_signal_highlights_1781337019`: the packet
completed with `status=complete`, `capture_count=5`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`resolved_solver_identity_complete=true`, and `review_index.html` showing the
new row-specific latest signals.

Latest local implementation slice: workflow review-card `latest signals` now
surface the contact-query and collision-cast values needed for static review of
rows 12-14. The cards promote selected/total contact counts, selected depth,
selected pair and shape indices, query-filter active/baseline/filtered counts,
ignored-pair state, ray hit count/fraction, swept sphere/capsule hit counts,
time of impact, and cast margins instead of showing only solver/executor
metadata. Focused capture-helper guards reported `2 passed`. Fresh rows 12-14
workflow evidence lives at
`build/captures/rigid_workflow_query_signal_highlights_1781335507`: the packet
completed with `status=complete`, `capture_count=3`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`resolved_solver_identity_complete=true`, and `review_index.html` showing the
new row-specific latest signals.

Latest local implementation slice: workflow review cards now summarize nested
`rigid_step_diagnostics` lane profile data as `backend diagnostics`. The static
row-8 card now names each workload lane's profile status, accelerated-backend
status, accelerated-stage count, max worker count, top stage, top-stage time,
and total stage time, making backend/fallback/timing status reviewable without
opening raw manifest JSON. Focused capture-helper guards reported `2 passed`.
Fresh single-row workflow evidence lives at
`build/captures/rigid_workflow_backend_diagnostics_1781335057`: row 8 captured
with `status=complete`, `capture_count=1`, `failed_count=0`,
`guidance_complete=true`, `scene_metrics_complete=true`,
`resolved_solver_identity_complete=true`, and `review_index.html` containing
the new `backend diagnostics` field.

Latest local implementation slice: rigid workflow search now separates
backend-status discovery from executor-equivalence discovery. Backend terms
such as `compute backend`, `backend comparison`, `parallel backend`, and
`backend/executor` route to `rigid_step_diagnostics`, where users can inspect
backend status, acceleration masks, worker count, ECS, scratch, contact, and
profile-stage diagnostics. Executor terms such as `compute executor`,
`executor comparison`, `parallel executor`, and `Taskflow executor` still route
to the same-solver `rigid_executor_equivalence` row. The py-demos README and
PLAN-103 sidecar now describe this split, preventing the GUI search from
presenting an executor timing comparison as an accelerated-backend comparison.
Focused search guards reported `2 passed`. This slice does not add a new
visual packet; it improves how users find the existing backend and executor
debug rows.

Latest local implementation slice: non-numbered World related shelf rows now
publish user-editable control metadata in capture metrics. `floating_base`
records `controls.spin_command` from the live floating-joint velocity, while
`articulated` records `controls.shoulder_damping` and
`controls.wrist_damping` from the live two-link arm joints. Focused
World-related shelf guards reported `2 passed`. This slice does not add a new
visual packet; the existing rows 01-36 and optional rows 37-53 packets remain
the latest broad static review artifacts.

Latest local implementation slice: optional Rigid IPC shelf and capture-first
packet rows now publish user-editable control metadata in capture metrics. The
direct shelf routes (`rigid_ipc`, `rigid_ipc_slide`, `rigid_ipc_incline`, and
`rigid_ipc_pile`) record `controls.friction`; the capture-first stack packet
variants record `controls.friction` and `controls.frame_budget_ms`; and the
stack packet reset path records a fresh initial metrics sample after a
friction-triggered reset. Focused shelf/packet guards reported `7 passed`.
This slice does not add a new visual packet; the existing rows 01-36 and
optional rows 37-53 packets remain the latest broad static review artifacts.

Latest local implementation slice: row 13,
`rigid_collision_query_options`, now records its body-kind query toggles and
ignored-pair selector under capture-metrics `controls`, matching the replay
state for the same GUI controls. The focused panel guard edits two query
checkboxes plus the ignored-pair selector and proves capture metrics identify
the selected options alongside the derived active, option-filtered, and ignored
contact counts. Focused query-options panel guards reported `1 passed`. This
slice does not add a new visual packet; the existing rows 01-36 and optional
rows 37-53 packets remain the latest broad static review artifacts.

Latest local implementation slice: solver-select rigid rows now have the same
run-defining panel contract as executor-select rows. The new focused guard
covers all seven current solver-selector panels and proves solver-only edits
reset the visual run and publish `controls.solver_index` in capture metrics.
The baseline `rigid_body` row now records `controls.solver_index` in capture
metrics, matching its replay state. Focused solver-switch panel guards reported
`7 passed`. This slice does not add a new visual packet; the existing rows
01-36 and optional rows 37-53 packets remain the latest broad static review
artifacts.

Previous local harness-hardening slice: rows 28-34 now have a stronger
panel-contract guard for the editable controls that make the later joint and
multibody rows useful during visual debugging. The comparison-panel test now
asserts the public Perturbation, motor/limit, screw-pitch, dynamics-term, COM,
and Jacobian slider labels and ranges. Row 29, `rigid_joint_motor_limits`, also
has a focused panel-edit guard proving GUI sliders update the controller, joint
command velocity, velocity limits, position upper limit, requested force,
effort cap, and capture controls. Focused panel guards reported `2 passed`,
and the adjacent row-29 runtime/replay guards reported `2 passed`. This slice
does not add a new visual packet; the existing rows 01-36 packet remains the
latest broad visual artifact.

Latest local implementation slice: executor-driven rigid GUI panels now treat
`Executor` as a run-defining visual-control edit. Twenty-four executor-select
panels reset the visual run on executor-only edits instead of mixing prior
timelines and world time with a newly selected executor, and rows that
previously omitted the executor control now publish `controls.executor_index`
in capture metrics. Focused executor-switch panel guards reported `24 passed`,
and the adjacent panel/replay/docs guard reported `23 passed`. This slice does
not add a new visual packet; the existing rows 01-36 and optional rows 37-53
packets remain the latest broad static review artifacts.

Latest local implementation slice: rows 35-36 now close the editable-control
guard for the numbered World Rigid Body workflow. Row 35,
`rigid_multibody_solver_family`, and row 36, `rigid_loop_closure`, have panel
guards for their gravity controls and focused panel-edit tests proving executor
and gravity changes update runtime state, reset simulation time, and publish
capture controls. Row 36 also resets/clears replay on executor-only edits,
matching the other executor-driven verifier rows, and its panel shows the active
executor. Focused panel guards reported `3 passed`, and the adjacent row 35-36
behavior guards reported `2 passed`. This slice does not add a new visual
packet; the existing rows 01-36 packet remains the latest broad visual artifact.

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

Latest local implementation slice: row 27, `rigid_distance_spring`, now exposes
the public distance-spring controls promised by the workflow. `World` has
narrow named-spring accessors for `has`, `get`, and live `set` of rest length
plus stiffness; dartpy exposes those as snake_case methods; and the row 27 GUI,
replay state, capture metrics, and docs now cover rest length plus soft, stiff,
and off-center stiffness controls. Focused API/panel/replay/row guards
reported `8 passed`, adjacent workflow-doc guards reported `3 passed`,
`pixi run check-api-boundary-inventory` reported the inventory is current, and
fresh single-scene visual evidence lives at
`build/captures/rigid_distance_spring_controls_1781326877`: 72 requested UI
frames, 71 PNG frames written, screenshot written,
`scene_metrics.event_count=72`, and latest scene metrics containing
`rest_length=0.45`, `soft_stiffness=45.0`, `stiff_stiffness=220.0`, and
`offset_stiffness=120.0`.

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
date`), pushed commit `fbbd5de0005` passed the default command
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run test-all`
with all six wrapper gates green: linting, build, unit tests, simulation
tests, Python tests, and documentation. The follow-up CUDA command
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 timeout 7200s pixi run -e cuda test-all`
ran on a visible `NVIDIA GeForce RTX 4080 Laptop GPU` host and passed all seven
CUDA wrapper gates: linting, build, unit tests, simulation tests, Python tests,
documentation, CUDA tests, and benchmark smoke. These broad wrapper runs cover
the row-5 direct-impulse implementation state at `fbbd5de0005`; later
review-prep/search-discovery commits have focused guards but have not rerun the
full default/CUDA wrappers. After the backend/executor search-routing,
backend-diagnostics, contact-query review-card, and optional-signal slices, the
full rows 01-36 packet remains current at
`build/captures/rigid_workflow_rows_01_36_1781335894`, and the optional rows
37-53 packet is current at
`build/captures/rigid_workflow_optional_signal_highlights_1781338541`. Both now
report complete scene metrics plus complete resolved solver identity under the
DART 7 work-packet harness. The prior stopped wrapper attempt remains recorded
below as historical context, not as current verification state.

Latest remote publication state: use `git status -sb` as the source of truth
for local/remote parity. The latest user-approved push advanced
`origin/feature/rigid-body-gui-visual-verification` to `cb5deee7bfd` after a
fresh `git fetch origin` and `git merge --no-edit origin/main` reported
`Already up to date`. The branch contains the PR #2986 DART 7
architecture/work-packet harness. This session may have local
search-discovery follow-up commits above that remote checkpoint until the next
explicit push approval. No PR exists for this branch in the latest local
checks. The approved push did not approve PR creation, milestone mutation, CI
reruns, review comments, thread resolution, or other GitHub review-state
changes.

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

Latest DART 7 work-packet harness dry-run: the current command
`DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dryrun_current`
completed successfully without rendering. The dry-run manifest reports
`status=planned`, `dry_run=true`, `capture_count=53`,
`guidance_complete=true`, `guidance_missing_count=0`, all requested optional
groups enabled, all selected optional groups enabled, and the generated
`review_index.html` header shows `rows 1-53 / 53`, requested groups
`numbered, related, ipc shelf, packets`, selected groups
`numbered, related, ipc shelf, packets`, `solver identity not required`, and
`scene metrics not required`.

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

Historical stopped-validation handoff: an earlier session was redirected to
stop code changes and stop further verification, update only the handoff
state, merge latest `origin/main`, push to `origin`, and stop. `git fetch
origin main && git merge --no-edit origin/main` reported `Already up to date`.
The running `test-all` wrapper was terminated by request during the
simulation-labeled ctest stage; it is not a completed validation pass. Visible
progress before termination included a successful direct `pixi run build-tests
ON Release` re-run, 219/219 C++ unit tests passed, and simulation output
through 52/65 visible tests. CUDA validation was not run. This remains
historical context only and is superseded by the current evidence above.

## Objective Requirements

| Requirement                                                                                      | Evidence inspected                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Status                                        |
| ------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------- |
| Curate rigid-body `py-demos` as the first DART 7 visual debugging path.                          | PLAN-103 sidecar owns a 36-row World Rigid Body workflow from `rigid_body` through `rigid_loop_closure`; `python/examples/demos/README.md` documents the same ordered path; branch default py-demos front door is guarded by tests.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Proven locally                                |
| Cover key rigid-body features, performance, corner cases, and deal-breaker behavior.             | The sidecar rows cover baseline, modes, free flight, frames, external and point loads, timestep sensitivity, step diagnostics, contact budget, restitution/material/friction/spin/stack behavior, contact inspection, query options, casts, solver family comparison, executor equivalence, contact-policy comparison, multibody contact, manipulation, kinematic drivers, fixed/breakable/distance/limited joints, motor/limit/passive/screw-joint behavior, dynamics terms, COM offsets, Jacobians, multibody solver routing, and loop-closure family selection.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Proven locally for current scoped public APIs |
| Let users switch or compare solvers/backends/parameters where it answers a practical question.   | Rows label comparison axes and held-fixed controls in the panel and capture metrics; examples include solver family, executor, contact policy, timestep, contact workload, passive parameter family, multibody solver family, and loop-closure family/policy rows. Backend-status search terms now route to `rigid_step_diagnostics`, while executor-equivalence terms route to `rigid_executor_equivalence`, so the GUI does not conflate compute backend status with same-solver executor timing.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Proven locally                                |
| Follow the DART 7 harness solver-identity and scene-metrics evidence rule for generated packets. | `py-demo-capture` now promotes validated `resolved_solver_identity` from latest scene metrics into successful per-scene manifests, requires solver-family plus context fields for workflow identity completeness, attaches identity and scene-metrics evidence to captured workflow rows, summarizes both completeness checks in workflow manifests, shows warning blocks in `review_index.html`, and returns failure status for non-dry-run workflow packets that capture rows without latest metrics or valid solver identity. Focused capture guards reported `11 passed`. The native `SIGBUS` was traced to Filament's Linux `/tmp`-backed CircularBuffer path under tmpfs user-quota pressure, and Linux headless engine creation now forces Filament's anonymous soft CircularBuffer fallback. The regenerated rows 01-36 packet and refreshed optional rows 37-53 packet both report `resolved_solver_identity_complete=true`, zero missing solver identities, zero scene-metrics gaps, and zero failed rows; a read-only per-scene audit found latest scene metrics for every captured row in both packet directories.           | Proven locally                                |
| Keep unsupported or misleading GUI claims out of the workflow.                                   | PLAN-103 public API audit records public direct `RigidBody.apply_linear_impulse()` and `RigidBody.apply_angular_impulse()` coverage in row 5, while sleep/wake, island activation, and loop-closure compliance/stiffness/damping still lack public surfaces; the workflow search routes supported impulse queries to `rigid_external_loads` and unsupported activation/compliance terms to nearest rows with explicit caveats instead of adding speculative rows.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Proven locally                                |
| Provide reviewer-facing visual evidence.                                                         | `build/captures/rigid_workflow_rows_01_36_1781335894/manifest.json` reports `status=complete`, `capture_count=36`, `completed_count=36`, `failed_count=0`, `guidance_complete=true`, `scene_metrics_complete=true`, `scene_metrics_count=36`, `resolved_solver_identity_complete=true`, `resolved_solver_identity_count=36`, and 2388 PNG frames. Its `review_index.html` includes the latest backend-diagnostics and contact-query/collision-cast review-card summaries. `build/captures/rigid_workflow_optional_signal_highlights_1781338541/manifest.json` reports `status=complete`, `capture_count=17`, `completed_count=17`, `failed_count=0`, `guidance_complete=true`, `scene_metrics_complete=true`, `scene_metrics_count=17`, `resolved_solver_identity_complete=true`, `resolved_solver_identity_count=17`, related/IPC-shelf/packet groups enabled, and a review card packet where rows 37-53 now surface optional health signals such as min tunnel margin, frame budget, top drift, differentiable fallback, and AVBD target/measured speed. Static review-index asset audits found 181/181 and 86/86 local links present. | Proven locally                                |
| Make static review artifacts self-contained enough for maintainer scan.                          | Current review-index asset audit found 181/181 local assets present for rows 01-36 and 86/86 present for optional rows 37-53; manifests and review headers record the exact top-level workflow commands.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Proven locally                                |
| Keep durable docs current.                                                                       | PLAN-103 sidecar, `python/examples/demos/README.md`, `CHANGELOG.md`, `docs/dev_tasks/rigid_body_visual_verification/README.md`, `RESUME.md`, and `PR_DRAFT.md` record the current workflow, evidence, caveats, and PR-ready state. PLAN-103 now includes a formal `WP-103.1` work-packet section with objective, scope, non-goals, acceptance evidence, gates, dependencies, and evidence pointers.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Proven locally                                |
| Provide automated drift guards.                                                                  | Focused docs/API drift guards, scene-metrics route guards, capture/review-index guards, current-head `pixi run lint`, and focused workflow-search/latest-signal guards are recorded in the PR draft and recent local commits. Broad default `pixi run test-all` passed under `DART_SAFE_JOBS=5` at pushed commit `fbbd5de0005`, and CUDA `pixi run -e cuda test-all` passed on the visible RTX 4080 Laptop GPU at the same implementation state.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Proven locally                                |
| Record the AI principle audit for substantial AI-assisted work.                                  | This file and `PR_DRAFT.md` record the objective, assumptions, source-of-truth docs, public `pixi run ...` paths, evidence packets, and shared-state safety status.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Proven locally                                |
| Open/publish maintainer review with DART 7.0 milestone.                                          | `gh pr list --head "$(git branch --show-current)"` and `gh pr status` still reported no PR in the latest recorded checks. The explicit push requests covered publishing the branch only, not PR creation, milestone mutation, CI rerun, or review-thread mutation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | PR/milestone external action still missing    |
| Maintainer accepts the 36-row workflow plus optional 53-row packet as the completed scope.       | `PR_DRAFT.md` includes the explicit maintainer acceptance checkbox. No maintainer acceptance has been recorded in the current repository state.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Missing external decision                     |
| Retire the dev-task folder in the completing PR.                                                 | `docs/dev_tasks/README.md` requires deletion in the completing PR after durable artifacts are promoted. This folder remains active because acceptance is not recorded yet.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Not yet done by design                        |

## Current Local Evidence

- Branch: `feature/rigid-body-gui-visual-verification`
- Latest merge check: `git fetch origin && git merge origin/main --no-edit`
  reported `Already up to date`, and `origin/main` is an ancestor of `HEAD`.
- Current remote publication state: `git status -sb` shows
  whether the local branch has follow-up commits above
  `origin/feature/rigid-body-gui-visual-verification`, whose latest approved
  pushed checkpoint is `cb5deee7bfd`. No PR for this branch from the latest
  recorded `gh pr list --head "$(git branch --show-current)"` or
  `gh pr status` checks.
- Latest broad default validation for pushed commit `fbbd5de0005`:
  `pixi run test-all` passed all six wrapper gates with `DART_SAFE_JOBS=5`;
  documentation built with the known generated-stub warnings for
  `dartpy._world_render_bridge`.
- Latest CUDA validation for pushed commit `fbbd5de0005`:
  `pixi run -e cuda test-all` passed all seven wrapper gates with
  `DART_SAFE_JOBS=5` on a visible `NVIDIA GeForce RTX 4080 Laptop GPU`,
  including CUDA runtime smoke tests and benchmark smoke.
- Latest focused validation for this slice: the workflow search/docs guard
  reported `3 passed`; the prior optional latest-signal guard reported
  `9 passed`.
- Latest harness dry-run: the full requested 53-row packet shape completed in
  plan mode with `guidance_complete=true` and no missing guidance rows.
- Current packet evidence: the full rows 01-36 packet and optional rows 37-53
  packet regenerated successfully after the latest control-metadata,
  search-routing, and optional-signal slices. Both report complete scene
  metrics and complete resolved solver identity.
- Historical stopped validation state: the interrupted `test-all` wrapper is
  not a pass and has been superseded by current green runs.
- Full review packet:
  `build/captures/rigid_workflow_rows_01_36_1781335894/review_index.html`
- Optional review packet:
  `build/captures/rigid_workflow_optional_signal_highlights_1781338541/review_index.html`
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
- Assumptions: direct rigid-body impulse is no longer deferred because public
  `RigidBody.apply_linear_impulse()` and `apply_angular_impulse()` APIs exist
  and are covered by row 5; sleep/wake, island activation, and loop-closure
  compliance rows stay deferred until public `dartpy` APIs exist.
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
  guards, recorded broad default/CUDA wrapper passes, and `pixi run lint` are
  recorded in the PR draft and this audit.
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
