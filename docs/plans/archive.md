# Plan Archive

Completed DART living plans, moved here from
[`dashboard.md`](dashboard.md) in the PR that completed them. The dashboard shows
only operating (non-`Complete`) plans; this file preserves the closing narrative
and PR references for finished work, ordered by plan number.

Each entry records `**Final status:** Complete`, its durable owner doc, the
completion outcome (the former dashboard next step), and the closing evidence
(the former dashboard gate). See [`README.md`](README.md) for the plan
lifecycle.

### PLAN-001: Living Plan System

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`README.md#plan-update-workflow`](README.md#plan-update-workflow)

**Dimension:** AI-native execution · **Horizon at completion:** Later

**Outcome:** Use the recorded structural checks whenever plan workflows,
generated adapters, required-reading paths, or dev-task shape rules change.

**Closing evidence:** `check-ai-commands`, `check-docs-policy`, and `sync-ai-commands` cover
adapter sync, required-reading coverage, and dev-task shape.

### PLAN-010: Easy-Start API And Package Readiness

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../../README.md#quick-start`](../../README.md#quick-start)

**Dimension:** Easy start · **Horizon at completion:** Later

**Outcome:** During DART 7 package publication, rerun the artifact recheck and
DART 7 README first-success verification against the published artifacts,
then remove or demote the current-package fallback snippets.

**Closing evidence:** README first-success snippets pass for current published packages,
source checkout, and local DART 7 package installs; DART 7 public package
publication remains tracked by `check-dart7-artifacts`.

### PLAN-020: Algorithm Extension Contracts

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../design/algorithm_extension_contracts.md`](../design/algorithm_extension_contracts.md)

**Dimension:** Algorithm extensibility · **Horizon at completion:** Later

**Outcome:** The DART 7 LCP evidence campaign is complete in
[#2962](https://github.com/dartsim/dart/pull/2962); use its LCP contract,
tests, benchmark packets, and `lcp_physics` example as the template when the
next algorithm family is selected. For solver or multi-physics papers, first
apply the solver-family intake checklist in
[`solver-family-intake.md`](solver-family-intake.md)
so the work routes to an existing family, shares common collision,
kinematics, and optimization components, and defines apples-to-apples
evidence plus a user-facing configuration shape.

**Closing evidence:** LCP contract docs, focused tests, smoke benchmark, API-boundary
exclusions, baseline example evidence, and the solver-family intake checklist
in [`solver-family-intake.md`](solver-family-intake.md) are recorded
before a new solver family or paper implementation starts; for solvers, that
checklist includes simple `World` defaults, method-specific advanced options,
validation, serialization expectations, and diagnostics.

### PLAN-031: Shared Experimental CUDA Device Substrate

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../design/shared_cuda_device_substrate.md`](../design/shared_cuda_device_substrate.md)

**Dimension:** Scalable compute · **Horizon at completion:** Later

**Outcome:** The build-now substrate landed on `main` in PR #2875. Use
[`../design/shared_cuda_device_substrate.md`](../design/shared_cuda_device_substrate.md)
as the extraction contract when AVBD-CUDA (PLAN-104), rigid-IPC GPU
(PLAN-082), PD-IPC GPU (PLAN-081), or broadened Phase-6 compute work
introduces a real second consumer for deferred rollout, graph-capture,
precision, colored-sweep, accelerator-registry, device IPC math, residency, or
reduction helpers. No new general library is planned: reuse `dart/math`,
`dart/optimizer`, `detail/newton_barrier`, and
`deformable_psd_backend`, and promote the same PLAN-083 cores
host/device-portable only when a GPU IPC consumer lands.

**Closing evidence:** Future substrate changes must keep shared device code `.cuh`/`.cpp`-only
under `compute/cuda/` (plus scanner-skipped `detail/` kernel cores), leave the
CUDA sidecar build-only and uninstalled, preserve the default no-GPU path, and
keep `pixi run -e cuda test-cuda`,
`pixi run test-simulation`,
`pixi run check-api-boundaries`,
`pixi run check-compute-backend-boundaries`, and
`pixi run check-no-gpu-runtime-dependencies` green for any promoted second-use
extraction.

### PLAN-036: Native Collision Performance Wave

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`035-native-collision/benchmark-manifest.md`](035-native-collision/benchmark-manifest.md)

**Dimension:** Scalable compute · **Horizon at completion:** Later

**Outcome:** Use the durable benchmark manifest and follow-up plan to choose
the next profiled hot-path slice without restoring reference-engine runtime
dependencies.

**Closing evidence:** The durable manifest records 45 comparable native leads, 0 behind rows,
24 native-only rows, and 0 rerun-needed rows from the local single-worker
benchmark packet; multi-core CPU and single-GPU work remain behind prototype
gates.

### PLAN-037: Native Collision Visual Verification

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../onboarding/gui-rendering.md#native-collision-visual-checks`](../onboarding/gui-rendering.md#native-collision-visual-checks)

**Dimension:** Algorithm extensibility · **Horizon at completion:** Later

**Outcome:** Use `examples/collision_sandbox` and
`EXAMPLE_collision_sandbox_headless_sweep` whenever native collision pair
support, broad-phase diagnostics, or Filament GUI rendering changes.

**Closing evidence:** The Filament-backed sandbox covers every supported native pair or
explicit unsupported placeholder, contact/manifold overlays, native
group/mask filtering, broad-phase overlays, focused registry/debug-snapshot
tests, and CTest-gated headless screenshot smoke coverage.

### PLAN-050: DART 7 World Binding Transition

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../onboarding/python-bindings.md#dart-7-world-bindings-and-transition`](../onboarding/python-bindings.md#dart-7-world-bindings-and-transition)

**Dimension:** Algorithm extensibility · **Horizon at completion:** Later

**Outcome:** Keep this item complete. Current DART 7 promotion work is tracked
by PLAN-040, PLAN-041, and PLAN-042; the ECS World is promoted to
`dart::simulation`, `dartpy.simulation`, and `dartpy.World`.

**Closing evidence:** The old split is retired: generated stubs no longer
publish `dartpy.simulation_experimental`, `check-dartpy-import-layout` guards
the promoted import shape, and any remaining `experimental` references are
compatibility or transition checks tracked by PLAN-041 rather than PLAN-050.

### PLAN-060: Backend-Hidden GUI Roadmap

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../onboarding/gui-rendering.md`](../onboarding/gui-rendering.md)

**Dimension:** Easy start · **Horizon at completion:** Later

**Outcome:** Keep future public GUI API promotion aligned with the durable
backend-hidden renderer guidance.

**Closing evidence:** GUI promotion stays backend-hidden and aligned with the maintained
Filament renderer onboarding guidance.

### PLAN-090: Filament Renderer Performance

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../onboarding/gui-rendering.md#performance-profiling-and-backend-selection`](../onboarding/gui-rendering.md#performance-profiling-and-backend-selection)

**Dimension:** Scalable compute · **Horizon at completion:** Later

**Outcome:** Use the renderer performance/profiling/backend guidance in
`../onboarding/gui-rendering.md` and the fidelity-profile design in
[`../design/filament_fidelity_profile.md`](../design/filament_fidelity_profile.md)
when extending the renderer; the latter is the seam to implement for
offline/high-fidelity sensor rendering (egocentric views, lens distortion,
depth/segmentation outputs). For material/lighting/texture fidelity and for
promoting debug visuals into the GUI component (per-shape PBR, IBL reflections,
texture mipmaps, an app `debugProvider`, richer debug primitives), see
[`../design/renderer_fidelity_and_debug_visuals.md`](../design/renderer_fidelity_and_debug_visuals.md).

**Closing evidence:** Per-phase profiling plus a toggleable in-app HUD (`--perf-hud`) expose
CPU+GPU frame time; the Filament backend is runtime-selectable (`--backend` /
`DART_FILAMENT_BACKEND`) with graceful fallback and no public backend-type
leak; the per-shape geometry cache reduced scene extraction without changing
output (`UNIT_gui_FilamentSceneExtraction`, headless scene smoke, and
`pixi run lint` green).

### PLAN-091: DART 7 Architecture Hardening

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../design/dart7_architecture_assessment.md`](../design/dart7_architecture_assessment.md);
follow-up owner docs:
[`solver-family-intake.md`](solver-family-intake.md) and
[`042-dart7-public-api-and-source-layout.md`](042-dart7-public-api-and-source-layout.md)

**Dimension:** Algorithm extensibility · **Horizon at completion:** Now

**Outcome:** Complete. Completed packets include WP-091.3 via PR #2999;
WP-091.5, WP-091.10, WP-091.11, WP-091.14, and WP-091.23 via PR #3003;
WP-091.12 via PR #3020; WP-091.20, WP-091.22, WP-091.30, and WP-091.33 via PR
#3029; WP-091.33a via PR #3042; WP-091.21 via PR #3044; WP-091.31,
WP-091.33b, and WP-091.33c via PR #3052; WP-091.32 via PR #3058; WP-091.33d
plus WP-091.34 via PR #3061; WP-091.15 via PR #3067; WP-091.40 via PR #3077;
WP-091.41 through WP-091.44 via PR #3072; the WP-091.24 standalone
harness/packet slice, bounded WP-091.13 shared-helper cleanup, and
WP-091.33e packet-reporting checker via PR #3083; WP-091.13 and WP-091.24
closeout via PR #3091; and the remaining WS0 packet, WP-091.4 legacy freeze,
via PR #3095. PLAN-042 Decision 5 now records freeze/quarantine first,
release-6.20+ and later release-lane ports next, then DART 7 public-contract
removal after those needs settle. The freeze gate blocks untagged legacy
surface growth while current open release-6.20 PRs (#3071, #3086, #3092,
#3105, and #3107) and any later release-lane ports finish. Final deletion of
DART 6 legacy is not a WP-091.4 prerequisite; track that in PLAN-042
follow-up work after the release lane is settled. No active PLAN-091 packets
remain; new solver-family work routes through
[`solver-family-intake.md`](solver-family-intake.md), unresolved
architecture findings remain in
[`../design/dart7_architecture_assessment.md`](../design/dart7_architecture_assessment.md),
and follow-up implementation belongs to the active owner plan that selects
the work.

**Closing evidence:** Closed. Durable evidence is the merged packet PR list above, the
standing solver-family intake gate, the moved cross-family metrics manifest
in `docs/design/dart7_cross_family_metrics_corpus.json`, and PLAN-042
Decision 5 for DART 6 legacy quarantine/removal sequencing. Future work uses
the gates of its active owner plan instead of PLAN-091 packet gates.

### PLAN-092: Performance Dashboard

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../readthedocs/community/performance_dashboard.rst`](../readthedocs/community/performance_dashboard.rst)

**Dimension:** Scalable compute · **Horizon at completion:** Later

**Outcome:** Live at `dartsim.github.io/dart/performance/` via
`benchmark-action/github-action-benchmark` and embedded in the Read the Docs
page. Per-PR regression comments are added as an opt-in extension (pending one
real-PR validation run): `.github/workflows/benchmark_pr_comparison.yml` runs
the dashboard's benchmark slice on a PR and `scripts/benchmark_pr_compare.py`
diffs it against the dashboard's published gh-pages history, then posts a
marker-based "moved by N%" comment in the PR review thread via
`actions/github-script`. It is a read-only consumer of the dashboard data (it
never writes to gh-pages — no second pipeline), gated to same-repo PRs carrying
the `benchmark` label so ordinary PRs pay no benchmark cost. This is the
reconciliation decision from the retired `benchmark_pr_comparison` dev task:
extend the existing dashboard infra (chosen) over the prototype's parallel
committed-JSON pipeline. The compare/render logic is unit-tested
(`test_benchmark_pr_compare.py`); the only step that needs a live PR to confirm
is the comment post itself. Gradual rollout: add the `benchmark` label to one
PR to confirm the comment posts, tune `--alert-threshold`, then consider
widening the trigger and the benchmark slice once review-noise tradeoffs are
visible. A secondary backend (Bencher/CodSpeed) remains optional future work.
The tracked slice now covers the new DART 7 solver families' end-to-end
`World::step` surfaces (rigid-body sequential-impulse/IPC, VBD + default
deformable grid, FEM bar, AVBD fixed-joint, motor, and breakable-joint rows)
alongside the original core step/scaling rows, and
`scripts/benchmark_display_names.py` rewrites the raw Google Benchmark names
into readable chart titles (merge `--humanize`) with family-grouped,
axis-labelled local preview. Add new surfaces by extending both
`run_performance_dashboard_benchmarks.py` (CPU-only, end-to-end filter) and
the `SURFACES` map.

**Closing evidence:** `pixi run bm-dashboard-preview` renders the dashboard locally from real
Google Benchmark JSON; each `main` publish updates the hosted per-benchmark
history.

### PLAN-100: DART 7 Lie-Group Consolidation

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../onboarding/architecture.md#12-math-module-dartmath`](../onboarding/architecture.md#12-math-module-dartmath)

**Dimension:** Release transition · **Horizon at completion:** Later

**Outcome:** Use `SO3`, `SE3`, tangents, `SpatialInertia`, batch operations,
and `GroupProduct` as the DART 7 typed Lie-group surface; keep future
composite work aligned with the componentwise direct-product pattern.

**Closing evidence:** `GroupProduct` source and tests are in `dart/math/lie_group/` under
snake_case headers, with inverse/map machinery matching the `SO3`/`SE3`
expression-template pattern and `ctest -R UNIT_math_lie_group` green.

### PLAN-102: Demos App

**Final status:** Complete (archived 2026-07-03).

**Owner doc:** [`../design/demos_app.md`](../design/demos_app.md); user
instructions in [`../../examples/README.md`](../../examples/README.md).

**Dimension:** Easy start · **Horizon at completion:** Now

**Outcome:** Implemented — `dart-demos` is now the smaller C++ World demo app
with `rigid_body`, `deformable_body`, and `vbd_deformable` runtime-switchable
scenes plus lightweight `Planned World Ports` placeholders for high-value DART
6 concepts that still need World-native ports (IK, SIMBICON walking,
operational-space control, robot puppets, mobile manipulation); the retired
collision sandbox route now points to concrete Python collision-debugging rows
instead of a C++ placeholder. The
`dart/gui/detail` `ExampleScene` set is intentionally kept as the renderer's
internal test fixtures (see the design doc's "examples vs renderer fixtures"
decision), so PLAN-101's `--scene` smoke gate is unaffected.

**Closing evidence:** `dart-demos` launches and switches across the World scenes in one window
without window recreation; the headless cycle smoke renders every registered
scene; only CLI/headless support programs stay standalone;
`pixi run lint` and `check-docs-policy` green.
