# Rigid Body Visual Verification PR Draft

Use this as the PR body seed after the branch is pushed with maintainer
approval. Keep the durable source of truth in
`docs/plans/103-examples-strategy/rigid-body-visual-verification.md`,
`python/examples/demos/README.md`, and the generated capture manifests. Use
`docs/dev_tasks/rigid_body_visual_verification/COMPLETION_AUDIT.md` for the
current evidence-to-requirement map and remaining external gates.

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
  produces a 52-row review packet.
- Adds or revises rigid scenes for baseline behavior, modes, free flight,
  frame hierarchy, external loads, point loads, timestep sensitivity,
  diagnostics, contact budget, material response, query/cast inspection, solver
  comparison, executors, multibody contact, friction/spin/stack behavior,
  kinematic drivers, rigid constraints, passive joint parameters, screw joints,
  generalized dynamics terms, COM offsets, link Jacobians, multibody solver
  family routing, and loop-closure family selection.
- Extends the workflow panel and capture helper with open-live commands,
  row-range packet commands, video packet commands, guidance completeness,
  failure summaries, replay metadata, latest-signal summaries, top-level
  command provenance, and static review-index links.
- Fixes Linux headless Filament engine creation to use Filament's anonymous
  soft CircularBuffer fallback, avoiding `/tmp` tmpfs-quota `SIGBUS` crashes
  during native capture startup.
- Documents the workflow in the durable PLAN-103 sidecar and the py-demos
  README, with dev-task notes kept only for active handoff and cleanup.
- Updates the DART 7 changelog with the rigid workflow, capture packet,
  review-index, replay metadata, related-evidence, and scene additions.

## Testing

- `pixi run lint`
- Focused docs/API drift guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_deferred_api_gaps_are_documented -q`
- Focused capture/review-index guard:
  `PYTHONPATH=build/default/cpp/Release/python:build/default/cpp/Release/python/dartpy:python DART_PARALLEL_JOBS=$JOBS CTEST_PARALLEL_LEVEL=$JOBS CMAKE_BUILD_PARALLEL_LEVEL=$JOBS pixi run python -m pytest python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_dry_run_can_request_video_commands python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_aggregates_scene_manifests python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_review_links_resolve_workspace_relative_artifacts python/tests/unit/test_capture_py_demo.py::test_rigid_workflow_run_links_scene_videos python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_readme_matches_sidecar_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_verification_sidecar_matches_registry_order python/tests/integration/test_demos_cycle.py::test_rigid_visual_workflow_docs_use_current_navigator_count -q`
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
  `build/captures/rigid_workflow_rows_01_36_1781312968`
  (`status=complete`, `capture_count=36`, `failed_count=0`,
  `guidance_complete=true`, `resolved_solver_identity_complete=true`,
  `resolved_solver_identity_count=36`, 2388 PNG frames).
- Optional rows 37-52 packet:
  `build/captures/rigid_workflow_optional_rows_37_52_1781313357`
  (`status=complete`, `capture_count=16`, `failed_count=0`,
  `guidance_complete=true`, `resolved_solver_identity_complete=true`,
  `resolved_solver_identity_count=16`, 1004 PNG frames).
- Static review-index asset audit: 0 missing local assets for both current
  review indexes.
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
- [ ] Maintainer accepts the current 36-row workflow plus optional 52-row
      packet as the completed scope for this dev task
