# Plan Dashboard

This is the one-screen operating view for DART's living plans.

This file is the single source of truth for plan priority, status, horizon,
north-star dimension, next step, and gate. Each entry may link to a detailed
numbered initiative file or to the authoritative owner document for initiatives
that are tracked elsewhere until they warrant a dedicated plan file. Detailed
plan files own workstreams and acceptance criteria. External owner documents
own the referenced scope until a detailed initiative file is added.
`north-star-roadmap.md` owns strategic framing.

## Operating View

Priority order follows document order. Keep each frequently changed field on
its own line so status updates remain git-history friendly.

### PLAN-001: Living Plan System

- Owner doc: [`README.md#plan-update-workflow`](README.md#plan-update-workflow)
- Status: Complete
- Horizon: Later
- Dimension: AI-native execution
- Next step: Use the recorded structural checks whenever plan workflows,
  generated adapters, required-reading paths, or dev-task shape rules change.
- Gate: `check-ai-commands`, `check-docs-policy`, and `sync-ai-commands` cover
  adapter sync, required-reading coverage, and dev-task shape.

### PLAN-010: Easy-Start API And Package Readiness

- Owner doc: [`../../README.md#quick-start`](../../README.md#quick-start)
- Status: Complete
- Horizon: Later
- Dimension: Easy start
- Next step: During DART 7 package publication, rerun the artifact recheck and
  DART 7 README first-success verification against the published artifacts,
  then remove or demote the current-package fallback snippets.
- Gate: README first-success snippets pass for current published packages,
  source checkout, and local DART 7 package installs; DART 7 public package
  publication remains tracked by `check-dart7-artifacts`.

### PLAN-012: Cloud Dartpy Tutorials

- Owner doc:
  [`012-cloud-dartpy-tutorials.md`](012-cloud-dartpy-tutorials.md)
- Status: Active
- Horizon: Next
- Dimension: Easy start
- Next step: Run a Colab runtime spike from a DART 7 dartpy wheel artifact and
  the Filament GUI branch to prove import, headless simulation, and one
  Filament-backed inline image or video before publishing stable tutorial links.
- Gate: Cloud tutorial support is not complete until the notebook rejects DART
  6 artifacts, installs only DART 7 dartpy artifacts in the cloud runtime,
  runs a headless simulation, renders a nonblank Filament-backed image or
  video inline, and passes both an automated Jupyter notebook smoke and a
  documented Colab runtime smoke.

### PLAN-020: Algorithm Extension Contracts

- Owner doc:
  [`../design/algorithm_extension_contracts.md`](../design/algorithm_extension_contracts.md)
- Status: Complete
- Horizon: Later
- Dimension: Algorithm extensibility
- Next step: Use the LCP v0 contract, tests, benchmark, and `lcp_physics`
  example as the template when the next algorithm family is selected.
- Gate: LCP contract docs, focused tests, smoke benchmark, API-boundary
  exclusions, and baseline example evidence are recorded.

### PLAN-030: Compute Scalability Roadmap

- Owner doc:
  [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md)
- Status: Active
- Horizon: Now
- Dimension: Scalable compute
- Next step: Follow the multiphase plan in
  `docs/dev_tasks/experimental_world_scalable_compute/`; land the Phase 0
  foundations (executor-parity test, EnTT concurrency contract, benchmark corpus
  with a contact-shaped proxy) before the resource-access metadata milestone,
  and keep both ahead of dependency inference, contact scheduling, or
  GPU/rendering backend work.
- Gate:
  [`030-compute-resource-access/evaluator.md`](030-compute-resource-access/evaluator.md)
  records the focused proof: graph/world tests and the compute-graph benchmark
  stay green; Taskflow remains behind the experimental executor boundary; graph
  metadata/profiling/DOT output remain backend-neutral; classic World behavior
  stays untouched.

### PLAN-080: Rigid-Body Dynamics Solver

- Owner doc:
  [`080-rigid-body-dynamics-solver.md`](080-rigid-body-dynamics-solver.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: Land Phase 0.1 (World gravity in the rigid-body integration stage)
  on the experimental World, then articulated-body forward dynamics; track
  slice-level work in `docs/dev_tasks/rigid_body_dynamics_solver/`.
- Gate: Each slice keeps focused experimental tests and `check-api-boundaries`
  green, holds DART 6 parity on shared scenes before any promotion claim, and
  never exposes solver/coupler/domain/backend types or ECS storage publicly.

### PLAN-035: Native Collision Feature Dashboard

- Owner doc:
  [`035-native-collision-dashboard.md`](035-native-collision-dashboard.md)
- Status: Parked
- Horizon: Later
- Dimension: Algorithm extensibility
- Next step: Use the durable feature dashboard and coverage matrix as the
  feature-complete baseline for the native collision performance wave.
- Gate: Coverage matrix has no active feature-level GAP/PARTIAL rows, runtime
  comparison-only implementations remain test/benchmark-only, and
  compatibility/package isolation audits remain part of `pixi run lint`.

### PLAN-036: Native Collision Performance Wave

- Owner doc:
  [`035-native-collision/benchmark-manifest.md`](035-native-collision/benchmark-manifest.md)
- Status: Complete
- Horizon: Later
- Dimension: Scalable compute
- Next step: Use the durable benchmark manifest and follow-up plan to choose
  the next profiled hot-path slice without restoring reference-engine runtime
  dependencies.
- Gate: The durable manifest records 45 comparable native leads, 0 behind rows,
  24 native-only rows, and 0 rerun-needed rows from the local single-worker
  benchmark packet; multi-core CPU and single-GPU work remain behind prototype
  gates.

### PLAN-037: Native Collision Visual Verification

- Owner doc:
  [`../onboarding/gui-rendering.md#native-collision-visual-checks`](../onboarding/gui-rendering.md#native-collision-visual-checks)
- Status: Complete
- Horizon: Later
- Dimension: Algorithm extensibility
- Next step: Use `examples/collision_sandbox` and
  `EXAMPLE_collision_sandbox_headless_sweep` whenever native collision pair
  support, broad-phase diagnostics, or Filament GUI rendering changes.
- Gate: The Filament-backed sandbox covers every supported native pair or
  explicit unsupported placeholder, contact/manifold overlays, native
  group/mask filtering, broad-phase overlays, focused registry/debug-snapshot
  tests, and CTest-gated headless screenshot smoke coverage.

### PLAN-080: Performance Dashboard

- Owner doc:
  [`../readthedocs/community/performance_dashboard.rst`](../readthedocs/community/performance_dashboard.rst)
- Status: Complete
- Horizon: Later
- Dimension: Scalable compute
- Next step: Live at `dartsim.github.io/dart/performance/` via
  `benchmark-action/github-action-benchmark` and embedded in the Read the Docs
  page. Optional future work: per-PR regression comments and a secondary backend
  (Bencher/CodSpeed).
- Gate: `pixi run bm-dashboard-preview` renders the dashboard locally from real
  Google Benchmark JSON; each `main` publish updates the hosted per-benchmark
  history.

### PLAN-040: DART 7 Release Hardening

- Owner doc: [`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md)
- Status: Complete
- Horizon: Later
- Dimension: Release transition
- Next step: Use the DART 7 gate table and packaging checklist during release
  packaging or release-hardening passes.
- Gate: Release gate evidence names the relevant local or CI command, blocker,
  or external compatibility check for each touched release surface.

### PLAN-050: Experimental World Split

- Owner doc:
  [`../onboarding/python-bindings.md#experimental-world-bindings-and-transition`](../onboarding/python-bindings.md#experimental-world-bindings-and-transition)
- Status: Complete
- Horizon: Later
- Dimension: Algorithm extensibility
- Next step: Track any future DART 8 promotion work under the release roadmap
  once the experimental world has parity gates.
- Gate: `dartpy.simulation_experimental` is separate from legacy
  `dartpy.simulation`, has focused import/API coverage, and the DART 7/8
  transition path is documented in onboarding docs.

### PLAN-060: Backend-Hidden GUI Roadmap

- Owner doc:
  [`../onboarding/gui-rendering.md`](../onboarding/gui-rendering.md)
- Status: Complete
- Horizon: Later
- Dimension: Easy start
- Next step: Keep future public GUI API promotion aligned with the durable
  backend-hidden renderer guidance.
- Gate: GUI promotion stays backend-hidden and aligned with the maintained
  Filament renderer onboarding guidance.

### PLAN-070: DART 8 Compatibility Cleanup

- Owner doc: [`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md)
- Status: Complete
- Horizon: Later
- Dimension: Release transition
- Next step: Use the compatibility-debt inventory and cleanup review checklist
  during DART 7 packaging passes and DART 8 removal planning.
- Gate: DART 8 cleanup decisions cite migration notes, changelog entries,
  package/export status, and gz-physics compatibility where relevant.

### PLAN-090: Filament Renderer Performance

- Owner doc:
  [`../onboarding/gui-rendering.md#performance-profiling-and-backend-selection`](../onboarding/gui-rendering.md#performance-profiling-and-backend-selection)
- Status: Complete
- Horizon: Later
- Dimension: Scalable compute
- Next step: Use the renderer performance/profiling/backend guidance in
  `../onboarding/gui-rendering.md` and the fidelity-profile design in
  [`../design/filament_fidelity_profile.md`](../design/filament_fidelity_profile.md)
  when extending the renderer; the latter is the seam to implement for
  offline/high-fidelity sensor rendering (egocentric views, lens distortion,
  depth/segmentation outputs).
- Gate: Per-phase profiling plus a toggleable in-app HUD (`--perf-hud`) expose
  CPU+GPU frame time; the Filament backend is runtime-selectable (`--backend` /
  `DART_FILAMENT_BACKEND`) with graceful fallback and no public backend-type
  leak; the per-shape geometry cache reduced scene extraction without changing
  output (`UNIT_gui_FilamentSceneExtraction`, headless scene smoke, and
  `pixi run lint` green).

### PLAN-100: DART 7 Lie-Group Consolidation

- Owner doc:
  [`../dev_tasks/lie_group_consolidation/README.md`](../dev_tasks/lie_group_consolidation/README.md)
- Status: Parked
- Horizon: Next
- Dimension: Release transition
- Next step: Port the deferred `GroupProduct` (composite Lie group) from
  `7/nested_group_product` into `main` under main conventions, mirroring the
  merged `SO3`/`SE3` inverse machinery and resolving the five known WIP gaps.
- Gate: `GroupProduct` source and tests land in `dart/math/lie_group/` with
  `pixi run lint`, build, and `ctest -R UNIT_math_lie_group` green; the
  Lie-group base API increment already merged via PR #2697.

### PLAN-101: dartsim GUI Simulator

- Owner doc: [`101-dartsim-gui-simulator.md`](101-dartsim-gui-simulator.md);
  durable architecture/as-built in
  [`../design/dartsim_gui_simulator.md`](../design/dartsim_gui_simulator.md),
  developer overview in
  [`../onboarding/gui-rendering.md`](../onboarding/gui-rendering.md).
- Status: Active
- Horizon: Next
- Dimension: Easy start
- Next step: v1 is implemented (engine `dartsim/engine` unit-tested by
  `UNIT_dartsim_engine`; `dartsim/ui` editor builds and renders the
  experimental scene headless). Pursue the remaining follow-ups: viewport
  pick-to-tree selection sync, and co-evolution to adopt experimental
  shape/loader APIs (replace editor-side shape descriptors) per PLAN-050.
- Gate: The headless engine is covered by command/undo, object, selection,
  name-uniqueness, and project-round-trip tests with zero GUI/renderer includes;
  the editor loop (design → run → record → replay) works on the experimental
  World only; the renderer stays backend-hidden (PLAN-060); and the default
  `dartsim` headless smoke (`DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS`) renders a
  non-blank editor frame while legacy `--scene` smokes keep contrast.

### PLAN-102: Demos App

- Owner doc: [`102-demos-app.md`](102-demos-app.md); active architecture in
  [`../dev_tasks/demos_app/01-design.md`](../dev_tasks/demos_app/01-design.md),
  execution tracking in
  [`../dev_tasks/demos_app/README.md`](../dev_tasks/demos_app/README.md).
- Status: Active
- Horizon: Now
- Dimension: Easy start
- Next step: Runtime scene-swap host, `dart-demos` app, and migration of 37 GUI
  examples into categorized scenes are landed and headless-verified (cycle smoke
  `EXAMPLE_dart_demos_cycle_headless_smoke`); tooling/docs/CHANGELOG updated.
  Remaining (deferred, see `../dev_tasks/demos_app/RESUME.md`): collapse the
  `dart/gui/detail` `ExampleScene` fixture catalog into the registry (retarget
  `test_filament_scene_extraction` + per-scene smokes + `dartsim --scene`) and
  promote the design doc — both touch shared GUI test infra (PLAN-101).
- Gate: `dart-demos` launches and switches across several scenes (including an
  asset-backed robot) in one window without leaks or window recreation; the
  example renderable-count and per-scene headless smokes run against the Demos
  registry; no standalone GUI example binary or `--scene` example path remains
  (only `hello_world` stays standalone); `pixi run lint`, `build`, `test-all`,
  and `check-docs-policy` green.

> Cross-plan note: removing the `--scene` example catalog supersedes PLAN-101's
> "legacy `--scene` smokes keep contrast" gate clause; update that clause when
> PLAN-102 Phase 3 lands.
