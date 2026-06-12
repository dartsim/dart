# Resume: Rigid Body Visual Verification

## Current Handoff (2026-06-12)

This checkpoint resumes after the earlier stop-state hand-off and captures the
related-evidence bundle as local work.

Expected repository state after this checkpoint:

- Branch: `feature/rigid-body-gui-visual-verification`.
- The branch is two local commits ahead of origin unless a future session has
  pushed it:
  - `1e3fd63bc04 Route rigid related search directly`.
  - `Capture rigid related evidence bundle`.
- There is no PR associated with this branch at checkpoint time.
- The latest checkpoint has not been pushed; do not push without explicit
  maintainer/user approval in that session.
- Before any further commit, rerun the repository-mandated `pixi run lint`.

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
`py-demo-capture -- --rigid-workflow --include-related` appends the nine
non-numbered related shelf routes after the 36 numbered rows and records them
in the same manifest/review index with `workflow_group=related_evidence`.

## Current Branch

`feature/rigid-body-gui-visual-verification`

Current snapshot:

- At the start of the current continuation, clean and aligned with
  `origin/feature/rigid-body-gui-visual-verification`.
- Latest pushed commits:
  `0e38e3e807d Fix py-demos cycle scene frame budget`.
  `e8278b6fb53 Improve rigid workflow capture evidence`.
- Current branch is expected to be two local commits ahead of origin after this
  checkpoint: `1e3fd63bc04 Route rigid related search directly` plus
  `Capture rigid related evidence bundle`.
- The latest checkpoint is local only and should not be pushed without explicit
  maintainer/user approval.

## Immediate Next Step

Inspect `git status -sb` and confirm whether the local checkpoint has been
pushed or is still ahead of origin. Do not push without explicit approval in
that session, and rerun `pixi run lint` before committing further changes.

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
  `contact model`; those terms are now covered in the uncommitted alias patch.
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
  `--include-related` variant appends the related routes as rows 37-45 in the
  generated review packet.

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
jq -r '.include_related, .capture_count, .captures[36].workflow_group, .captures[36].scene, .captures[44].scene, .artifacts.review_index' /tmp/dart_capture_rigid_workflow_related_dry_run_current/manifest.json
rg -n "related_evidence|avbd_rigid_prismatic_motor|45/45" /tmp/dart_capture_rigid_workflow_related_dry_run_current/review_index.html
```

The focused pytest reported `5 passed`. The public dry-run completed with
45 planned capture commands, the manifest reported `include_related=true` and
`capture_count=45`, and the generated review index contained the final
`45/45 avbd_rigid_prismatic_motor` related-evidence row.
