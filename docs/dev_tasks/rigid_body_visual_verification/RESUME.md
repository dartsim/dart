# Resume: Rigid Body Visual Verification

## Current Handoff (2026-06-12)

This checkpoint is hand-off documentation only. The user explicitly stopped
all further implementation and verification, asked for the current work to be
resumable from a fresh Claude/Codex session, and previously asked for the
hand-off to be pushed. No code, tests, lint, dry-runs, or diff checks were run
for this checkpoint.

The newest implementation checkpoint remains `cc8431ef6f9 Add stack stability
replay timeline`, which added replay-guided timeline metadata to
`rigid_stack_stability` after the live open-command workflow follow-up. The
stack row uses top-x divergence as its Replay value track and marks frames
where overlap, low clearance, visible top-block drift, or solver-family
divergence make the stack worth scrubbing.

Expected repository state after this hand-off:

- Branch: `feature/rigid-body-gui-visual-verification`.
- Latest implementation commits covered by this hand-off:
  `4c9f367bcd0 Preserve requested rigid workflow packet groups`,
  `f48187d6ce2 Summarize rigid workflow packet groups in review index`, and
  `f01f471bae7 Expose rigid workflow packet commands in the panel`, followed
  by `3c5b9e517d3 Enable rigid workflow video packets`,
  `d5c6de2bee1 Describe optional rigid workflow rows`,
  `5a4529f0083 Audit rigid workflow guidance coverage`, and
  `ad013e62069 Refresh rigid guidance audit handoff`, followed by the current
  live open-command and stack Replay timeline checkpoints.
- `6bbed86f397 Refresh rigid workflow stop handoff` is a docs-only pushed
  checkpoint after the workflow-video packet slice.
- Before this hand-off doc was committed, the branch was ahead of
  `origin/feature/rigid-body-gui-visual-verification` by five local commits:
  `d5c6de2bee1 Describe optional rigid workflow rows`,
  `5a4529f0083 Audit rigid workflow guidance coverage`,
  `ad013e62069 Refresh rigid guidance audit handoff`,
  `4395ba51f35 Expose rigid workflow live open commands`, and
  `cc8431ef6f9 Add stack stability replay timeline`.
- This docs-only hand-off commit is intended to be pushed together with those
  five parent commits because the user explicitly requested pushing the
  hand-off and then stopping.
- There is no PR associated with this branch at checkpoint time.
- The current continuation resumed implementation from the active persistent
  goal and finished the pending guidance-audit checks after the previous
  hand-off-only stop checkpoint.
- The live open-command continuation finished the previously documented WIP,
  added capture-helper tests and public docs, and reran focused tests, a public
  dry-run artifact inspection, and `pixi run lint`.
- The stack Replay timeline continuation added `replay_timeline` metadata to
  `rigid_stack_stability`, updated tests and docs, and ran focused tests plus a
  real docked capture.
- After this pushed hand-off, do not push any new implementation or handoff
  commit without explicit approval in a future session.
- Before any future commit, rerun the repository-mandated `pixi run lint`.

## Last Session Summary

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

The final action in this checkpoint was hand-off documentation only, after the
user explicitly requested stopping all further work and verification.

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

The final action in this session is hand-off documentation only. The user
explicitly instructed the agent to stop working further, ensure the hand-off
docs, push them, and fully stop. No additional implementation or verification
was performed after that instruction.

## Current Branch

`feature/rigid-body-gui-visual-verification`

Current snapshot:

- Latest implementation commits covered by this hand-off:
  `4c9f367bcd0 Preserve requested rigid workflow packet groups`,
  `f48187d6ce2 Summarize rigid workflow packet groups in review index`, and
  `f01f471bae7 Expose rigid workflow packet commands in the panel`, followed
  by `3c5b9e517d3 Enable rigid workflow video packets`,
  `d5c6de2bee1 Describe optional rigid workflow rows`,
  `5a4529f0083 Audit rigid workflow guidance coverage`, and
  `ad013e62069 Refresh rigid guidance audit handoff`, followed by the current
  live open-command and stack Replay timeline checkpoints.
- `6bbed86f397 Refresh rigid workflow stop handoff` is a pushed docs-only
  checkpoint.
- Immediately before this hand-off doc was committed, the branch was ahead of
  origin by five commits: `d5c6de2bee1`,
  `5a4529f0083`, `ad013e62069`, `4395ba51f35`, and
  `cc8431ef6f9`.
- This docs-only hand-off commit is expected to be pushed with those five
  parents because the user explicitly requested it.
- There is no PR associated with this branch at checkpoint time.

## Immediate Next Step

Inspect `git status -sb` and `git log -5 --oneline` first. Expect the latest
implementation commits to include the requested/selected manifest metadata
slice, the review-index group-summary slice, the workflow-panel review-packet
command slice, `3c5b9e517d3 Enable rigid workflow video packets`, and the
optional-row metadata plus guidance-coverage audit slices, followed by the
live open-command and stack Replay timeline slices. If this hand-off push
succeeded, expect `origin/feature/rigid-body-gui-visual-verification` to
contain the docs-only stop hand-off on top of `cc8431ef6f9`. Do not continue
implementation or push again without a new explicit user request.

## Context That Would Be Lost

- The task goal is not just to add examples; it is to make rigid-body GUI demos
  useful as visual debugging and verification surfaces for solvers, backends,
  parameters, contact behavior, constraints, and corner cases.
- Durable planning context lives in
  `docs/plans/103-examples-strategy/rigid-body-visual-verification.md`.
- That sidecar already documents the 36 rigid workflow rows and current API
  limitations.
- Public `dartpy` is currently believed to lack direct rigid-body impulse,
  sleep/wake/island activation, and loop-compliance APIs. Do not build rows
  around those features without first confirming the API surface changed.
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
  `rigid_ipc_stack_packet` after the numbered rows and, when present, after the
  related-evidence rows.
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

No lint, tests, dry-runs, or diff checks were run for the final docs-only
handoff checkpoint because the user explicitly requested no further
verification.

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
