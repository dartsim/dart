# Rigid Body Visual Verification - Dev Task

## Current Handoff (2026-06-12)

This checkpoint adds a workflow-manifest semantics follow-up after the direct
Rigid IPC shelf capture bundle. Row-range workflow manifests now keep the
requested optional packet groups in `include_related`, `include_ipc_shelf`, and
`include_packets`, while separate `selected_include_*` fields report which
optional groups are present in the selected row range. The generated
`review_index.html` mirrors that context with row-span, requested-groups, and
selected-groups badges.

Expected repository state after this hand-off:

- Branch: `feature/rigid-body-gui-visual-verification`.
- The branch should be pushed to
  `origin/feature/rigid-body-gui-visual-verification` with these latest
  implementation commits:
  `50e671590c8 Promote rigid IPC edge drop evidence`.
  `a95687dc628 Promote rigid IPC shelf metrics`.
  `d6ccdd38c4e Bundle direct rigid IPC shelf captures`.
- This continuation adds a local follow-up commit for the manifest
  requested-versus-selected optional-group split unless a future session has
  not committed it yet.
- There is no PR associated with this branch at checkpoint time.
- Before any future commit, rerun the repository-mandated `pixi run lint`.

## Current Status

- [x] Local branch contains the first rigid-body GUI reliability fix:
      `0e38e3e807d Fix py-demos cycle scene frame budget`.
- [x] Full `py-demos --cycle-scenes --headless --frames 1 --hide-ui`
      completed locally across 151 scenes before this handoff.
- [x] PLAN-103 command guidance was updated to use the bounded one-frame
      cycle gate.
- [x] Handoff docs were restored because the task is still active and spans
      sessions.
- [x] The `Rigid Workflow` panel search now recognizes more DART 7 solver,
      contact-policy, backend, executor, and worker-count terminology.
- [x] `py-demo-capture -- --rigid-workflow` now plans or runs the full numbered
      rigid capture workflow and writes a workflow-level manifest.
- [x] The full `py-demo-capture -- --rigid-workflow` path completed all
      36 numbered rigid captures locally.
- [x] The workflow runner now treats a missing per-scene manifest as a failed
      capture even when the child process exits 0.
- [x] The in-viewer `Rigid Workflow` panel now shows the exact per-row capture
      command, frame count, resolution, and docked-UI mode.
- [x] Adjacent comparison/parameter rows now label the comparison axis and
      held-fixed controls in both the panel and capture metrics.
- [x] Workflow captures now write a top-level `review_index.html` contact
      sheet so all 36 row screenshots, manifests, commands, and metric
      summaries can be reviewed from one page.
- [x] Related-evidence `Find row` matches now open the matched non-numbered
      shelf scene directly instead of requiring an intermediate stop on the
      numbered source row.
- [x] `py-demo-capture -- --rigid-workflow --include-related` now appends the
      non-numbered related-evidence shelf routes to the same workflow manifest
      and review index.
- [x] `py-demo-capture -- --rigid-workflow --include-packets` now appends the
      capture-first rigid IPC stack packet to the same workflow manifest and
      review index without changing the default 36-row workflow.
- [x] `py-demo-capture -- --rigid-workflow --workflow-start-row N --workflow-end-row M`
      now supports targeted workflow reruns while
      preserving absolute row numbers and output directories.
- [x] Numbered rigid workflow captures now carry the in-viewer row guidance
      into `manifest.json` and `review_index.html`: role label, user question,
      try-first action, inspect signals, healthy signal, and scope note.
- [x] `rigid_ipc_edge_drop` is now a self-describing related-evidence route
      from `rigid_solver_compare`, with a panel and capture metrics for
      degenerate edge-barrier gap, tilt, angular speed, contact count, step
      timing, and status.
- [x] The remaining Rigid IPC shelf scenes (`rigid_ipc`, `rigid_ipc_slide`,
      `rigid_ipc_incline`, and `rigid_ipc_pile`) now publish scene-owned
      capture metrics for direct docked captures while preserving shared replay
      controls.
- [x] `py-demo-capture -- --rigid-workflow --include-ipc-shelf` now appends the
      four direct metric-backed Rigid IPC shelf scenes to the workflow manifest
      and `review_index.html` without changing the default 36-row workflow.
- [x] Row-range workflow manifests now preserve the requested optional packet
      groups in `include_related`, `include_ipc_shelf`, and `include_packets`
      while exposing `selected_include_*` fields for the selected slice.
- [x] The workflow `review_index.html` header now shows row-span,
      requested-groups, and selected-groups badges for row-range packets.

## Goal

Make DART's Python GUI demos a practical visual-debugging surface for rigid
body simulation. The rigid category should expose the important solver,
backend, parameter, contact, constraint, and corner-case behavior in ways that
are easy to inspect, cycle, capture, and regression-test.

## Non-Goals For The Current Slice

- Do not start a broad public API expansion unless the missing API is first
  confirmed and scoped.
- Do not implement direct rigid-body impulse, sleep/wake, island activation, or
  loop-compliance rows until public `dartpy` APIs exist for them.
- Do not add new C++ demos for this task unless a future plan explicitly calls
  for it.

## Branch Snapshot

- Branch: `feature/rigid-body-gui-visual-verification`
- Latest implementation commits being handed off and pushed:
  `50e671590c8 Promote rigid IPC edge drop evidence`.
  `a95687dc628 Promote rigid IPC shelf metrics`.
  `d6ccdd38c4e Bundle direct rigid IPC shelf captures`.
- This continuation adds a local manifest-semantics follow-up commit. It has
  not been pushed unless the user explicitly approves a push.
- There is no PR associated with this branch at checkpoint time.

## What The Local Commit Changed

- `dart/gui/detail/application.cpp`
  - Treats `--frames N` as the per-scene budget when `--cycle-scenes` is used.
  - Stops forwarding that value as the global `RunOptions.maxFrames` in cycle
    mode.
  - Sets `appOptions.run.maxFrames = -1` for cycle mode so headless defaults do
    not end the whole catalog after one global frame.
  - Prints and flushes progress lines such as
    `Cycling demo scene 1/151: rigid_body`.
- `python/tests/integration/test_demos_cycle.py`
  - Adds coverage that the cycle frame budget applies per scene.
  - Keeps the basic cycle success test bounded with `--frames 1`.
- `CHANGELOG.md`, `docs/plans/dashboard.md`,
  `docs/plans/103-examples-strategy.md`, and
  `python/examples/demos/README.md`
  - Document the bounded cycle command, the 151-scene catalog state, and the
    per-scene meaning of `--frames`.

## What The Latest Pushed Commit Changed

- `python/examples/demos/runner.py`
  - Adds aliases so users can find the intended rigid workflow rows using
    terms such as `RigidBodySolver`, `ContactSolverMethod`, `Taskflow
executor`, `backend/executor`, `worker count`, `contact model`, and
    `solver policy`.
  - Adds maintained capture specs to each `RigidWorkflowGuide`, so the
    in-viewer workflow panel shows the exact command for regenerating that
    row's visual evidence.
- `python/tests/unit/test_py_demo_panels.py`
  - Locks the new search routes to `rigid_solver_compare`,
    `rigid_executor_equivalence`, `rigid_step_diagnostics`, and
    `rigid_contact_solver_compare`.
  - Locks the per-row workflow capture command panel text and comparison-axis
    labels.
- `python/examples/demos/scenes/rigid_timestep_sensitivity.py`,
  `python/examples/demos/scenes/rigid_step_diagnostics.py`,
  `python/examples/demos/scenes/rigid_contact_scale_budget.py`, and
  `python/examples/demos/scenes/rigid_joint_passive_parameters.py`
  - Add explicit `comparison_axis` and `held_fixed` panel/capture-metrics
    labels so users can tell solver, executor, workload, and parameter-family
    changes apart.
- `scripts/capture_py_demo.py`
  - Adds maintained `rigid_workflow_capture_specs()` and
    `--rigid-workflow`.
  - `--rigid-workflow --dry-run` writes a top-level plan manifest without
    rendering.
  - `--rigid-workflow` runs each numbered rigid capture into
    `scenes/NN_<scene_id>/` and updates a workflow-level manifest with each
    per-scene manifest path and status.
  - Missing per-scene manifests are hard failures with
    `failure_reason: missing_manifest`, so the top-level workflow cannot report
    complete without evidence for every row.
  - Every dry-run, progress update, failure, and successful run also rewrites
    `review_index.html`, a static contact sheet that links each row's
    manifest, screenshot, frames directory, command, and metrics summary.
- `python/tests/unit/test_capture_py_demo.py`
  - Covers dry-run plan output, fake scene-manifest aggregation, and child
    process isolation for per-scene workflow captures.
  - Adds a regression that a child process returning 0 without a scene manifest
    fails the workflow and stops at the missing-evidence row.
  - Checks that the workflow review index exists and lists planned, captured,
    metric-bearing, and failed rows.
- `python/tests/integration/test_demos_cycle.py`
  - Keeps the helper's maintained capture spec synchronized with the sidecar
    and README command lists.
  - Checks the workflow-panel capture commands and comparison-axis metadata
    remain synchronized with the maintained row specs.
- `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`,
  `python/examples/demos/README.md`, and `CHANGELOG.md`
  - Document the workflow capture command, review index contact sheet, and new
    search terminology.

## Verified Before The Stop Request

These commands were run before the user requested no further verification:

```bash
pixi run build
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_runner_cycle_frames_budget_applies_per_scene python/tests/integration/test_demos_cycle.py::test_runner_cycle_returns_zero -q
PYTHONPATH=build/default/cpp/Release-docking/python:build/default/cpp/Release-docking/python/dartpy:python DART_PARALLEL_JOBS=3 CTEST_PARALLEL_LEVEL=3 CMAKE_BUILD_PARALLEL_LEVEL=3 timeout 180s pixi run py-demos -- --cycle-scenes --headless --frames 1 --hide-ui
pixi run lint
git diff --check
```

Observed results before the stop request:

- Build passed.
- Focused cycle tests passed: `2 passed`.
- The public cycle command completed all 151 demo scenes with exit code 0.
- Lint passed.
- `git diff --check` was clean.

## Verified In The Latest Session

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
```

Observed results:

- Focused pytest initially reported `5 passed`.
- The public dry-run command wrote
  `/tmp/dart_capture_rigid_workflow_dry_run/manifest.json`.
- The dry-run manifest reports workflow `rigid_visual_verification`, status
  `planned`, `capture_count` 36, first scene `rigid_body`, and last scene
  `rigid_loop_closure`.
- The dry-run command triggered Pixi's `py-demo-capture` build path and rebuilt
  the GUI Python target because the branch includes the earlier C++ GUI change.

After the full workflow capture exposed buffered parent progress, the progress
flush and child-process capture updates were added. These focused checks were
then rerun before the final stop request:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
```

Observed results:

- Focused pytest reported `6 passed`.
- The public dry-run command completed with exit code 0 and printed the full
  36-command plan.
- `pixi run lint` had been started after those updates, but the user then
  requested no further verification. Do not claim a latest lint or
  `git diff --check` result until rerun in a future session.

## Verified In The Current Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
pixi run lint
git diff --check
```

Observed results:

- Focused pytest reported `7 passed`.
- The public dry-run command completed with exit code 0 and printed the full
  36-command plan.
- `pixi run lint` passed; Black reformatted `scripts/capture_py_demo.py`.
- `git diff --check` was clean.

After two read-only explorer passes, the current continuation also added the
per-row capture command to the `Rigid Workflow` panel and the comparison
contract labels for adjacent parameter/profiling rows. This focused check
covered the workflow panel, capture helper failure paths, capture-spec sync,
and comparison metrics:

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_guides_expose_capture_commands python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
pixi run lint
git diff --check
```

Observed result:

- Focused pytest reported `14 passed`.
- The public dry-run command completed with exit code 0 and printed the full
  36-command plan.
- `pixi run lint` passed.
- `git diff --check` was clean.

The current continuation then added the workflow review index/contact sheet.
After `pixi run lint` reformatted `scripts/capture_py_demo.py`, this focused
check was rerun:

```bash
pixi run lint
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_comparison_panels_label_the_compared_axis python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_renders_guidance_for_numbered_rows python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_guides_expose_capture_commands python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_prioritizes_user_intent_over_scope_caveats python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_backend_and_profile_aliases python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_fails_when_scene_manifest_is_missing python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_scene_capture_runs_in_child_process python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_capture_commands_match_workflow python/tests/integration/test_demos_cycle.py::test_rigid_timestep_sensitivity_orders_freefall_error_by_step_size python/tests/integration/test_demos_cycle.py::test_rigid_step_diagnostics_reports_profile_and_memory_counters python/tests/integration/test_demos_cycle.py::test_rigid_contact_scale_budget_orders_contact_loads python/tests/integration/test_demos_cycle.py::test_rigid_joint_passive_parameters_order_passive_response -q
pixi run py-demo-capture -- --rigid-workflow --dry-run --output-dir /tmp/dart_capture_rigid_workflow_dry_run
test -f /tmp/dart_capture_rigid_workflow_dry_run/review_index.html && jq -r '.artifacts.review_index, .capture_count, .status' /tmp/dart_capture_rigid_workflow_dry_run/manifest.json && rg -n "DART rigid workflow review index|rigid_loop_closure" /tmp/dart_capture_rigid_workflow_dry_run/review_index.html
git diff --check
```

Observed result:

- Focused pytest reported `14 passed`.
- The public dry-run completed with exit code 0 and printed all 36 planned
  capture commands.
- The dry-run manifest reported
  `/tmp/dart_capture_rigid_workflow_dry_run/review_index.html`, `36`, and
  `planned`; the generated review index contained the
  `DART rigid workflow review index` heading and the final
  `rigid_loop_closure` row.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Full Workflow Capture Session

```bash
pixi run py-demo-capture -- --rigid-workflow --output-dir /tmp/dart_capture_rigid_workflow_full_review_index_1781259714
```

Observed results:

- The command completed with exit code 0.
- The top-level manifest
  `/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/manifest.json`
  reported workflow `rigid_visual_verification`, status `complete`,
  `capture_count` 36, `completed_count` 36, `failed_count` 0, and elapsed time
  `314.278` seconds.
- Every capture entry had `return_code=0`, `status=captured`, and
  `manifest_exists=true`.
- The first per-scene manifest was
  `/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/scenes/01_rigid_body/manifest.json`.
- The last per-scene manifest was
  `/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/scenes/36_rigid_loop_closure/manifest.json`.
- The top-level manifest recorded
  `/tmp/dart_capture_rigid_workflow_full_review_index_1781259714/review_index.html`
  under `artifacts.review_index`.
- The generated review index existed, contained 36 screenshot thumbnails, and
  linked the first and last screenshots plus comparison-axis/held-fixed metric
  summaries for representative workflow rows.

## Verified In The Related-Search Routing Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves -q
pixi run lint
git diff --check
```

Observed results:

- Focused pytest reported `3 passed`.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Verified In The Related-Evidence Capture Bundle Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_include_related_requires_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-related --dry-run --output-dir /tmp/dart_capture_rigid_workflow_related_dry_run_current
jq -r '.include_related, .capture_count, .captures[36].workflow_group, .captures[36].scene, .captures[45].scene, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_related_dry_run_current/manifest.json
rg -n "related_evidence|avbd_rigid_prismatic_motor|46/46" /tmp/dart_capture_rigid_workflow_related_dry_run_current/review_index.html
```

Observed results:

- Focused pytest reported `5 passed`.
- The public dry-run completed with exit code 0 and printed all 46 planned
  capture commands: 36 numbered rows plus 10 related-evidence routes.
- The dry-run manifest reported `include_related=true`, `capture_count=46`,
  `captures[36].workflow_group=related_evidence`, first related scene
  `floating_base`, final scene `avbd_rigid_prismatic_motor`, and a
  `review_index.html` path.
- The generated review index contained related-evidence cards and the final
  `46/46 avbd_rigid_prismatic_motor` row.

## Verified In The Capture-First Packet Bundle Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_packets_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --dry-run --output-dir /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run
jq -r '.include_related, .include_packets, .capture_count, .captures[46].workflow_group, .captures[46].scene, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run/manifest.json
rg -n "capture_first_packet|rigid_ipc_stack_packet|47/47" /tmp/dart_capture_rigid_workflow_packets_edge_drop_dry_run/review_index.html
```

Observed results:

- Focused pytest reported `8 passed`.
- The public dry-run completed with exit code 0 and printed all 47 planned
  capture commands: 36 numbered rows, 10 related-evidence routes, and the
  capture-first rigid IPC stack packet.
- The dry-run manifest reported `include_related=true`,
  `include_packets=true`, `capture_count=47`,
  `captures[46].workflow_group=capture_first_packet`, final scene
  `rigid_ipc_stack_packet`, and a `review_index.html` path.
- The generated review index contained the final
  `47/47 rigid_ipc_stack_packet` packet row.

## Verified In The Row-Range Rerun Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q
pixi run py-demo-capture -- --rigid-workflow --include-related --include-packets --workflow-start-row 47 --workflow-end-row 47 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run
jq -r '.capture_count, .workflow_total_count, .workflow_row_start, .workflow_row_end, .captures[0].order, .captures[0].count, .captures[0].scene, .captures[0].workflow_group' /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run/manifest.json
rg -n "47/47|rigid_ipc_stack_packet|capture_first_packet" /tmp/dart_capture_rigid_workflow_row_rerun_edge_drop_dry_run/review_index.html
```

Observed results:

- Focused pytest reported `11 passed`.
- The public dry-run completed with exit code 0 and printed only the selected
  row-47 packet capture command.
- The dry-run manifest reported `capture_count=1`,
  `workflow_total_count=47`, `workflow_row_start=47`, `workflow_row_end=47`,
  selected row order/count `47/47`, scene `rigid_ipc_stack_packet`, and
  `workflow_group=capture_first_packet`.
- The generated review index contained the absolute
  `47/47 rigid_ipc_stack_packet` row.

## Verified In The Review-Index Guidance Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_select_row_range python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_can_resume_from_selected_row python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_row_selection_validates_bounds -q
pixi run py-demo-capture -- --rigid-workflow --workflow-start-row 15 --workflow-end-row 15 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_guidance_dry_run
jq -r '.captures[0].workflow_label, .captures[0].user_question, .captures[0].try_first, .captures[0].healthy_signal, .captures[0].scope' /tmp/dart_capture_rigid_workflow_guidance_dry_run/manifest.json
rg -n "How do the rigid method families differ visually|Healthy: wall clearance|Generic thin-wall comparison" /tmp/dart_capture_rigid_workflow_guidance_dry_run/review_index.html
```

Observed results:

- Focused pytest reported `11 passed`.
- The public row-15 dry-run completed with exit code 0.
- The dry-run manifest reported the `Solver family` role, the maintained
  solver-family user question, try-first guidance, healthy signal, and scope
  note.
- The generated review index contained the same solver-family question,
  healthy signal, and scope note beside the row card.

## Verified In The Rigid IPC Edge-Drop Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_edge_drop_reports_degenerate_contact_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_related_evidence_routes_are_valid python/tests/integration/test_demos_cycle.py::test_rigid_visual_routes_publish_self_describing_capture_metrics python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_related_evidence_routes_to_other_shelves python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_search_finds_related_evidence_targets python/tests/unit/test_py_demo_panels.py::test_rigid_workflow_panel_opens_related_evidence_search_matches -q
```

Observed results:

- Focused pytest reported `9 passed`.
- The new edge-drop route reports `row=rigid_ipc_edge_drop`,
  `related_source_row=rigid_solver_compare`, `solver=rigid_ipc`,
  `scope=degenerate_edge_contact_capability`, near-barrier clearance, and
  nonzero angular/tilt motion.
- The related-evidence route table, capture command list, capture-helper specs,
  panel coverage, search routing, direct related route, and unnumbered-route
  guard all include `rigid_ipc_edge_drop`.
- The public `--include-related` dry-run reported `capture_count=46`,
  `40/46 rigid_ipc_edge_drop`, and final
  `46/46 avbd_rigid_prismatic_motor`.
- The public row-range packet dry-run reported `workflow_total_count=47` and
  `47/47 rigid_ipc_stack_packet`.
- The real docked capture wrote a nonblank 960x540 screenshot with docked UI,
  71 converted PNG frames, latest status `edge-barrier`, minimum barrier gap
  about `0.000384` m, maximum tilt about `55.33` degrees, and maximum angular
  speed about `0.555` rad/s.

## Verified In The Rigid IPC Shelf Metrics Continuation

```bash
pixi run lint
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_ipc_shelf_scenes_report_capture_metrics python/tests/unit/test_py_demo_panels.py::test_high_value_world_scenes_expose_custom_panels -q
pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_pile_metrics_current
git diff --check
```

Observed results:

- `pixi run lint` passed.
- Focused pytest reported `2 passed`.
- The direct Rigid IPC shelf scenes `rigid_ipc`, `rigid_ipc_slide`,
  `rigid_ipc_incline`, and `rigid_ipc_pile` now report scene-owned capture
  metrics for row identity, IPC solver label, scope, time-step/world-time,
  friction, status, contact counts, step timing, and scene-specific gap, speed,
  travel, height, span, or pile summaries. The focused test also checks that
  each scene preserves `replay_sync` and declares
  `replay_live_step_is_stateless`.
- A real docked
  `pixi run py-demo-capture -- --scene rigid_ipc_pile --frames 24 --width 960 --height 540 --show-ui --output-dir /tmp/dart_capture_rigid_ipc_pile_metrics_current`
  capture wrote a 960x540 screenshot with docked UI detected, 23 converted PNG
  frames, 24 scene-metric events, latest row `rigid_ipc_pile`, scope
  `multi_box_barrier_pile`, 25 history samples, `box_count=3`, maximum history
  speed about `1.177` m/s, and minimum history clearance about `0.276` m.
- `git diff --check` was clean.

## Verified In The Direct Rigid IPC Shelf Capture Bundle Continuation

```bash
PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_writes_capture_plan python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_related_evidence python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_direct_ipc_shelf python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_include_capture_first_packets python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_extra_groups_require_workflow python/tests/integration/test_demos_cycle.py::test_rigid_visual_related_evidence_capture_commands_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_capture_first_packets_are_documented python/tests/integration/test_demos_cycle.py::test_rigid_visual_direct_ipc_shelf_captures_are_documented -q
pixi run py-demo-capture -- --rigid-workflow --include-ipc-shelf --dry-run --output-dir /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .capture_count, .workflow_total_count, .captures[36].workflow_group, .captures[36].scene, .captures[39].scene' /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current/manifest.json
rg -n "37/40 rigid_ipc|40/40 rigid_ipc_pile|rigid_ipc_shelf" /tmp/dart_capture_rigid_workflow_ipc_shelf_only_dry_run_current/review_index.html
pixi run py-demo-capture -- --rigid-workflow --include-related --include-ipc-shelf --include-packets --workflow-start-row 47 --workflow-end-row 51 --dry-run --output-dir /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current
jq -r '.include_related, .include_ipc_shelf, .include_packets, .capture_count, .workflow_total_count, .captures[0].order, .captures[0].workflow_group, .captures[0].scene, .captures[3].scene, .captures[4].workflow_group, .captures[4].scene' /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current/manifest.json
rg -n "47/51 rigid_ipc|50/51 rigid_ipc_pile|51/51 rigid_ipc_stack_packet|rigid_ipc_shelf" /tmp/dart_capture_rigid_workflow_ipc_shelf_dry_run_current/review_index.html
```

Observed results:

- Focused pytest reported `11 passed`.
- The public `--include-ipc-shelf` dry-run completed with 40 planned captures:
  36 numbered rows plus direct Rigid IPC shelf rows 37-40.
- The dry-run manifest reported `include_ipc_shelf=true`,
  `capture_count=40`, `workflow_total_count=40`, first IPC shelf scene
  `rigid_ipc`, final IPC shelf scene `rigid_ipc_pile`, and
  `workflow_group=rigid_ipc_shelf`.
- The generated review index contained `37/40 rigid_ipc`,
  `40/40 rigid_ipc_pile`, and the `rigid_ipc_shelf` group.
- The combined row-range dry-run with related evidence, IPC shelf rows, and
  packets selected rows 47-51. It reported the direct IPC shelf rows as
  absolute 47-50 and `rigid_ipc_stack_packet` as the explicit combined-mode
  row 51.

## Verified In The Workflow Manifest And Review Index Metadata Continuations

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

Observed results:

- Focused pytest reported `16 passed` for manifest metadata and `4 passed` for
  the review-index metadata follow-up.
- The public combined row-range dry-run selected rows 47-51 and wrote a
  workflow manifest where requested flags were all true:
  `include_related=true`, `include_ipc_shelf=true`, and
  `include_packets=true`.
- The same manifest reported the selected slice separately:
  `selected_include_related=false`, `selected_include_ipc_shelf=true`, and
  `selected_include_packets=true`, with `capture_count=5`,
  `workflow_total_count=51`, `workflow_row_start=47`, and
  `workflow_row_end=51`.
- The generated review index contained the absolute `47/51 rigid_ipc`,
  `50/51 rigid_ipc_pile`, and `51/51 rigid_ipc_stack_packet` rows.
- The updated review index header also contained row-span
  `47-51 / 51`, requested-groups
  `numbered, related, ipc shelf, packets`, and selected-groups
  `ipc shelf, packets`.
- `pixi run lint` passed.
- `git diff --check` was clean.

## Key Context

- The durable rigid workflow sidecar is
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`.
- The sidecar already records the rigid visualization scope, API-gap audit, and
  36 workflow rows.
- The latest API-gap audit in that sidecar says public `dartpy` currently lacks
  direct rigid-body impulse, sleep/wake/island activation, and loop-compliance
  APIs, so those rows should remain deferred unless that changes.
- Two read-only explorer subagents informed the latest work:
  one recommended a single rigid workflow capture mode, and one recommended
  additional solver/backend/contact-policy search aliases.

## Immediate Next Steps

1. Resume from `git status -sb` and `git log -5 --oneline`. Expect the
   direct Rigid IPC shelf capture-bundle slice to be committed as
   `d6ccdd38c4e Bundle direct rigid IPC shelf captures`, followed by the
   manifest requested-versus-selected optional-group follow-up if this
   continuation was committed.
2. Rerun the repository-mandated `pixi run lint` before any further commit.
3. Choose the next bounded rigid visual-verification gap from the durable
   sidecar, or retire this dev-task folder only if the maintainer explicitly
   accepts the current scope as complete.
4. Do not push unless the user explicitly approves pushing in that session.

## Commit And Push Notes

- Do not commit without following the repository pre-commit rule:
  `pixi run lint` is mandatory before every commit.
- Do not push unless the user explicitly approves pushing in that session.
- Before pushing an existing PR branch, merge the latest base branch into the
  branch first; do not rebase a published PR branch.
