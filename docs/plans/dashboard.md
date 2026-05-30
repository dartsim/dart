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
  The notebook gallery imports the headless scene modules from PLAN-103
  (`python/examples/demos`) rather than duplicating scene code.
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
- Next step: Phases 0-5 are complete and merged to `main` (PRs #2698, #2710,
  #2712); the dev-task folder has been retired, so PLAN-030 plus
  [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md)
  are now the durable trackers. The default experimental `World::step` path
  preserves the rigid-body contact/multibody solver pipeline, while the batched
  SoA rigid-body stage remains an explicit unconstrained path and
  benchmark/prototype seam. Phase 5 is closed with a GO: `CI CUDA / CUDA Build`
  compiles the CUDA targets on a GitHub-hosted `ubuntu-latest` runner (green on
  `main`), and because the project does not maintain a self-hosted GPU runner,
  the go/no-go runtime packet is measured manually on a CUDA host. The recorded
  GO (2026-05-28, RTX 5000 Ada): speedup 109.6x at 4096/128/100 with final-state
  error 1.78e-15, packet accepted (see the owner doc's "Recorded Phase 5
  Go/No-Go"). Keep CUDA private and non-required. The sidecar package shape,
  go/no-go threshold, `bm-phase5-gpu-packet-check` /
  `check-compute-backend-boundaries` / `check-no-gpu-runtime-dependencies`
  evidence gates, and the `check-phase5-cuda-benchmark-contract` row contract
  are recorded in the owner doc. To refresh the packet on any CUDA host, run
  `bm-phase5-cuda-full` then `bm-phase5-cuda-packet`;
  `check-phase5-cuda-workflow` guards that `ci_cuda.yml` keeps the build/import
  gate and never reintroduces a self-hosted GPU runner. Phase 3's speedup surface
  is the checked contact-island benchmark, not the trivial Euler rigid-body
  rows.
- Phase 6 backlog (unblocked by the Phase 5 GO, unstarted; each item needs its
  own design note and gate before work starts): broaden GPU stage coverage
  beyond the single rigid-body integration stage; promote auto-scheduling from
  resource-access metadata behind a verified scheduler contract (honest
  declarations, deferred structural changes, deterministic reductions, cost
  gate); heterogeneous batches and single-scene contact/constraint GPU work
  (Pattern B, only after Pattern A evidence justifies it); and differentiable
  state types if differentiability is promoted from a deferred to a committed
  capability. Rationale for each lives in
  [`../design/compute_backend_research.md`](../design/compute_backend_research.md).
- Gate: `pixi run test-simulation-experimental` covers graph/world parity for
  the current CPU foundation; `pixi run bm-compute-check` keeps the full
  expected `bm_compute_graph` corpus reproducible for the current Euler and
  contact-shaped workloads; the performance dashboard publishes the
  contact-shaped proxy, contact-island speedup surface, and Phase 5 CPU-baseline
  history; `pixi run -e cuda test-cuda` covers the opt-in CUDA smoke path on CUDA
  hosts; and future compute-bound contact/constraint work must extend the
  checked benchmark gate.
  Taskflow remains behind the experimental executor boundary, metadata remains
  backend-neutral, CUDA remains private/non-required, and classic World behavior
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

### PLAN-081: Deformable Implicit-Barrier Solver

- Owner doc:
  [`081-deformable-implicit-barrier-solver.md`](081-deformable-implicit-barrier-solver.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: Use the PLAN-081 IPC paper/repository gap audit to implement the
  full mesh-backed IPC-class follow-up. Start a dedicated `docs/dev_tasks/`
  folder for that multi-session implementation, then work through
  mesh/material state, scene loading, BE/Newmark integration, PT/EE distance
  derivatives, conservative CCD line search, projected Newton, friction,
  diagnostics, and the complete upstream example/test/benchmark/visual corpus.
- Gate: Full IPC-parity progress is not complete until the implementation
  distinguishes the first point-mass/static-ground slice from full IPC, keeps
  IPC naming backend-neutral, proves mesh contact, barrier, distance, CCD,
  friction, material, boundary-condition, serialization, and diagnostics
  behavior with focused tests, ports upstream paper/tutorial/stress examples,
  records benchmark/profiling JSON for kernels/solver/scenes/scaling, verifies
  long-horizon headless Filament captures for GUI examples, and keeps
  `pixi run lint`, `pixi run build`, focused C++ tests, and
  `check-api-boundaries` green.

### PLAN-104: Vertex Block Descent Solver

- Owner doc:
  [`104-vertex-block-descent-solver.md`](104-vertex-block-descent-solver.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: The DART-owned VBD CPU+CUDA solver landed on `main` (#2781):
  per-vertex block kernels, graph coloring, the colored Gauss-Seidel
  block-descent driver, Stable Neo-Hookean tetrahedra, Chebyshev/Rayleigh
  acceleration, the implicit-Euler stepper, the opt-in World wiring
  (`comps::DeformableVbdConfig` + the `advanceDeformableBody` VBD branch), the
  algorithm-neutral public `configureDeformableSolver` API and dartpy binding,
  static half-space ground contact + Coulomb friction, the CUDA mass-spring and
  tetrahedral rollouts (CUDA-graph capture + mixed precision), the CPU baseline
  benchmark, and the first GUI showcases (cloth/net/beam). Remaining work, in
  order: route VBD tet elasticity through the shared `deformable_elasticity` FEM
  kernels so VBD honors the body's FEM material choice; let VBD honor the merged
  sphere/box obstacle barriers instead of falling back to the default solver;
  Phase 7 surface self-collision/friction (vertex-triangle / edge-edge penalty,
  reusing the PLAN-081 `detail/deformable_contact` kernels); the TinyVBD
  parity/regression example with committed benchmark JSON; then Phase 8b SoA +
  Gaia-CPU benchmark and Phase 9 RTX-4090 same-GPU Table 1 reproduction. Reconcile
  and retire `docs/dev_tasks/vbd_deformable_solver/` once self-collision closes.
- Gate: VBD progress is not complete until the implementation distinguishes
  each internal kernel slice from a wired solver, keeps VBD naming
  backend-neutral, proves per-vertex force/Hessian correctness, PD Hessian
  guarantees, coloring conflict-freedom, and convergence parity with the
  existing solver and the reference implementations, adds FEM/acceleration/
  contact/friction with focused tests, records CPU and GPU benchmark/profiling
  JSON that beats the reference and/or paper numbers before any parity claim,
  verifies headless Filament captures for GUI examples, and keeps
  `pixi run lint`, `pixi run build`, focused C++ tests, and
  `check-api-boundaries` green.

### PLAN-082: Linear-Time Variational Integrator

- Owner doc:
  [`082-variational-integrator-solver.md`](082-variational-integrator-solver.md)
- Status: Proposed
- Horizon: Next
- Dimension: Algorithm extensibility
- Next step: Run the O(n) impulse-ABI de-risking spike first — the experimental
  World has no ABA today (dense `M.ldlt()` solve), and the linear-time claim
  depends on it — then start Phase A1 (fixed-base MVP on a dense-solve
  placeholder) under a new `docs/dev_tasks/variational_integrator_solver/`
  folder. Durable design lives in
  [`../design/simulation_variational_integrator.md`](../design/simulation_variational_integrator.md);
  contact/friction is a deferred go/no-go sidecar.
- Gate: Phase A1 proves symplectic energy behavior (no secular energy drift over
  ≥1e5 steps where semi-implicit Euler drifts), analytic single-DOF correctness,
  bounded RIQN iterations with a defined non-convergence error, and
  determinism/serialization round-trip; Phase A2 proves sub-quadratic O(n)
  scaling via benchmark JSON; `check-api-boundaries` stays green with no
  solver/stage/component/backend leak; contact/friction stays behind the
  contact-roadmap go/no-go.

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
  [`../onboarding/architecture.md#12-math-module-dartmath`](../onboarding/architecture.md#12-math-module-dartmath)
- Status: Complete
- Horizon: Later
- Dimension: Release transition
- Next step: Use `SO3`, `SE3`, tangents, `SpatialInertia`, batch operations,
  and `GroupProduct` as the DART 7 typed Lie-group surface; keep future
  composite work aligned with the componentwise direct-product pattern.
- Gate: `GroupProduct` source and tests are in `dart/math/lie_group/` under
  snake_case headers, with inverse/map machinery matching the `SO3`/`SE3`
  expression-template pattern and `ctest -R UNIT_math_lie_group` green.

### PLAN-101: dartsim GUI Simulator

- Owner doc: [`101-dartsim-gui-simulator.md`](101-dartsim-gui-simulator.md);
  durable architecture/as-built in
  [`../design/dartsim_gui_simulator.md`](../design/dartsim_gui_simulator.md),
  developer overview in
  [`../onboarding/gui-rendering.md`](../onboarding/gui-rendering.md).
- Status: Active
- Horizon: Next
- Dimension: Easy start
- Next step: v1 plus the post-MVP **workbench completion** are landed — every
  panel/menu drives the engine through tested `dartsim/ui/*_actions` seams
  (project, history, outliner, inspector, palette, relationship, simulation,
  console, watch, viewport), selection syncs across viewport/tree/inspector, and
  the editor covers create/edit/relationship/simulation/record-replay/watch and
  a four-view layout. The `dartsim_workbench_completion` dev task is retired into
  the design doc; its remaining work is experimental-API-gated and tracked there:
  runtime sensor output panes + joint render layers/visibility filters, richer
  relationship inspectors/grouping, a Scene Tree context-menu popup affordance,
  extracting the `editor.cpp` project file-dialog flow into its own tested seam,
  and adopting experimental shape/loader APIs (replace editor-side shape
  descriptors) per PLAN-050.
- Gate: The headless engine is covered by command/undo, object, selection,
  name-uniqueness, and project-round-trip tests with zero GUI/renderer includes;
  the filtered `dartsim/engine/*` + `dartsim/ui/*_actions` surface holds ≥95%
  line coverage (`pixi run coverage-report-dartsim`); the editor loop (design →
  run → record → replay) works on the experimental World only; the renderer
  stays backend-hidden (PLAN-060), enforced for `dartsim/engine` and
  `dartsim/ui` by `scripts/check_api_boundaries.py`; and the default `dartsim`
  headless smoke (`DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS`) renders a non-blank
  editor frame while legacy `--scene` smokes keep contrast.

### PLAN-102: Demos App

- Owner doc: [`../design/demos_app.md`](../design/demos_app.md); user
  instructions in [`../../examples/README.md`](../../examples/README.md).
- Status: Complete
- Horizon: Now
- Dimension: Easy start
- Next step: Implemented — `dart-demos` hosts 37 GUI examples as categorized,
  runtime-switchable scenes (cycle smoke `EXAMPLE_dart_demos_cycle_headless_smoke`);
  `hello_world` stays the standalone template; tooling/docs/CHANGELOG updated.
  The `dart/gui/detail` `ExampleScene` set is intentionally kept as the renderer's
  internal test fixtures (see the design doc's "examples vs renderer fixtures"
  decision), so PLAN-101's `--scene` smoke gate is unaffected. C++ `dart-demos`
  is now **frozen**: future example growth is Python-first under PLAN-103, which
  also owns the retire-later checklist for this C++ app.
- Gate: `dart-demos` launches and switches across scenes (including asset-backed
  robots) in one window without window recreation; the headless cycle smoke
  renders every scene; only `hello_world` plus a few special-build/test-coupled
  programs stay standalone; `pixi run lint` and `check-docs-policy` green.

### PLAN-103: Examples Strategy (Python-First)

- Owner doc: [`103-examples-strategy.md`](103-examples-strategy.md); C++ app
  architecture in [`../design/demos_app.md`](../design/demos_app.md).
- Status: Active
- Horizon: Now
- Dimension: Easy start
- Next step: Phases 1–4 landed — `dart-demos` Python (29 scenes incl. 5
  minimal-viable modern scenarios + an interactive Filament viewer via
  `dartpy.gui.run_demos`), cross-language golden parity (Python + C++ both
  assert 3 scenes against shared fixtures within 1e-9),
  `python/tutorials/01_browse_demos.ipynb`. Phase 5 (retire C++ `dart-demos`)
  is explicit "not now": conditions 4–5 of the retire-later checklist are met,
  conditions 1–3 wait on Python-breadth growth (~73%) + PLAN-012 (Colab smoke) +
  PLAN-101 (editor loading example scenes). The `docs/dev_tasks/examples_strategy/`
  folder is retired; residual follow-ups are tracked in PLAN-103's
  Landed State + retire-later checklist.
- Gate: `pixi run py-demos -- --cycle-scenes` cycles all scenes (exit 0);
  Python `test_golden_parity` + C++ `UNIT_gui_DemosGoldenParity` both pass
  against the shared fixture; the notebook imports (not copies) the scene
  modules; C++ `dart-demos` unchanged; `pixi run lint` and `check-docs-policy`
  green.
