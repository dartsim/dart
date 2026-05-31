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
  compiles the CUDA targets for fork PRs on the hosted fallback and runs CUDA
  tests on the trusted `ubuntu-latest-gpu` runner for same-repository PRs,
  protected branch pushes, and manual dispatches. The go/no-go runtime packet is
  still a measured benchmark packet from a CUDA host. The recorded GO
  (2026-05-28, RTX 5000 Ada): speedup 109.6x at 4096/128/100 with final-state
  error 1.78e-15, packet accepted (see the owner doc's "Recorded Phase 5
  Go/No-Go"). Keep CUDA private and non-required. The sidecar package shape,
  go/no-go threshold, `bm-phase5-gpu-packet-check` /
  `check-compute-backend-boundaries` / `check-no-gpu-runtime-dependencies`
  evidence gates, and the `check-phase5-cuda-benchmark-contract` row contract
  are recorded in the owner doc. To refresh the packet on any CUDA host, run
  `bm-phase5-cuda-full` then `bm-phase5-cuda-packet`;
  `check-phase5-cuda-workflow` guards that `ci_cuda.yml` keeps fork PRs on the
  hosted compile fallback and restricts GPU-runtime steps to trusted events.
  Phase 3's speedup surface is the checked contact-island benchmark, not the
  trivial Euler rigid-body rows.
- Phase 6 backlog (unblocked by the Phase 5 GO, unstarted; each item needs its
  own design note and gate before work starts): broaden GPU stage coverage
  beyond the single rigid-body integration stage; promote auto-scheduling from
  resource-access metadata behind a verified scheduler contract (honest
  declarations, deferred structural changes, deterministic reductions, cost
  gate); heterogeneous batches and single-scene contact/constraint GPU work
  (Pattern B, only after Pattern A evidence justifies it), including any PD-IPC
  GPU contact path tracked under
  [`081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md`](081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md);
  and differentiable state types if differentiability is promoted from a
  deferred to a committed capability. Rationale for each lives in
  [`../design/compute_backend_research.md`](../design/compute_backend_research.md).
- Gate: `pixi run test-simulation-experimental` covers graph/world parity for
  the current CPU foundation; `pixi run bm-compute-check` keeps the full
  expected `bm_compute_graph` corpus reproducible for the current Euler and
  contact-shaped workloads; the performance dashboard publishes the
  contact-shaped proxy, contact-island speedup surface, and Phase 5 CPU-baseline
  history; `pixi run -e cuda test-all` is the local full CUDA gate on Linux CUDA
  hosts; `pixi run -e cuda test-cuda` remains the focused CUDA smoke path; and
  future compute-bound contact/constraint work must extend the checked
  benchmark gate.
  Taskflow remains behind the experimental executor boundary, metadata remains
  backend-neutral, CUDA remains private/non-required, and classic World behavior
  stays untouched.

### PLAN-080: Rigid-Body Dynamics Solver

- Owner doc:
  [`080-rigid-body-dynamics-solver.md`](080-rigid-body-dynamics-solver.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: Prioritize the DART 7 release gates for the experimental World:
  model loading beyond the basic legacy bridge, parity scenes, open-chain
  dynamics, contacts/constraints, and serialization evidence; track slice-level
  work in
  `docs/dev_tasks/rigid_body_dynamics_solver/`.
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
  Track Shortest Path to Boundary as a separate self-intersection recovery
  sidecar in
  [`081-deformable-implicit-barrier-solver/spb-gap-audit.md`](081-deformable-implicit-barrier-solver/spb-gap-audit.md):
  first source/code audit, then a standalone tetrahedral query and DCD recovery
  spike before any solver or public API claim.
  Track Penetration-free Projective Dynamics on the GPU as a separate
  GPU-accelerated IPC sidecar in
  [`081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md`](081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md):
  first source/code audit, then a CPU-verifiable projective IPC slice and
  fast-CCD validation before any A-Jacobi, GPU-culling, or speedup claim.
- Gate: Full IPC-parity progress is not complete until the implementation
  distinguishes the first point-mass/static-ground slice from full IPC, keeps
  IPC naming backend-neutral, proves mesh contact, barrier, distance, CCD,
  friction, material, boundary-condition, serialization, and diagnostics
  behavior with focused tests, ports upstream paper/tutorial/stress examples,
  records benchmark/profiling JSON for kernels/solver/scenes/scaling, verifies
  long-horizon headless Filament captures for GUI examples, and keeps
  `pixi run lint`, `pixi run build`, focused C++ tests, and
  `check-api-boundaries` green. SPB additionally needs the sidecar source/code
  matrix, tetrahedral traversal and feasible-region tests, vertex-tet and
  edge-tet DCD candidate tests, pre-intersected recovery evidence, limitation
  coverage, CCD/DCD comparison packets, and no public SPB/reference-project,
  Embree, MeshFrame2, CuMatrix, solver, ECS, or backend-type leak. PD-IPC
  additionally needs the sidecar source/code matrix, CPU reference tests for the
  two-level projective IPC loop, fast-CCD validation against conservative DART
  CCD including public false-negative examples, A-Jacobi residual/convergence
  tests, optional GPU culling and solver packets with setup/transfer/readback
  timing, and no public PD-IPC/A-Jacobi/reference-project/CUDA or backend-type
  leak.

### PLAN-082: Rigid Implicit-Barrier Contact Solver

- Owner doc:
  [`082-rigid-implicit-barrier-contact.md`](082-rigid-implicit-barrier-contact.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: Continue the active dev task from fixture replay, comparison
  script ingestion, curved-trajectory CCD/residual/subdivision slices, local
  rigid barrier derivatives, scene-level sparse barrier assembly, and
  conservative line-search feasibility, and barrier/dynamics Newton solve
  scaffolding, first physical dynamics-term construction, opt-in runtime rigid
  IPC stage, same-domain `World` rigid solver selection, and runtime sphere
  triangulation, durable stage diagnostics, and the first activated-contact
  runtime regression plus vertex-vertex line-search CCD, invalid runtime
  geometry rejection, explicit non-converged-result skipping, and
  primitive-family friction potentials, first lagged friction assembly, and
  bounded outer lagged-friction passes into remaining geometry corpus coverage,
  runtime fixture behavior, production convergence criteria, production-ready
  default activation criteria, mixed-domain coupling, rigorous interval
  arithmetic, direct CCD evaluator parity, remaining comparison script
  commands, and full fixture/test/benchmark/visual parity. Keep the
  simultaneous-impact intake as a PLAN-082 sidecar until solver-neutral scenes
  prove a restitution/order-uncertainty gap not already covered by rigid IPC or
  AVBD-style finite-time contact.
- Gate: Full rigid IPC progress is not complete until the implementation covers
  every manifest row with DART-owned tests, examples, benchmarks, comparison
  packets, CPU/GPU evidence where applicable, and headless Filament visual
  captures; `scripts/check_rigid_ipc_fixture_manifest.py` stays green with an
  audited upstream checkout, public APIs remain backend-neutral and free of
  solver registries/ECS storage, and every promoted runtime slice passes
  `pixi run lint`, `pixi run build`, focused C++ tests, and
  `check-api-boundaries`. A simultaneous-impact operator additionally requires
  the sidecar literature matrix, corpus evidence, IPC/AVBD comparison, and
  public-boundary review before promotion.

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
  benchmark, and the first GUI showcases (cloth/net/beam). PR #2801 extends
  that landed path by routing VBD tetrahedra through the shared
  `deformable_elasticity` FEM kernels, preserving VBD on static sphere/box
  obstacle barriers, adding lagged VT/EE surface self-collision penalties, and
  adding the TinyVBD tilted-strand plus contact showcase py-demos; it also
  retires the temporary `docs/dev_tasks/vbd_deformable_solver/` tracker by
  promoting the gap audit into this plan. Remaining VBD closeout work is:
  self-contact tangential friction, OGC source/code audit and CPU
  proof-of-contact from
  [`104-vertex-block-descent-solver/ogc-gap-audit.md`](104-vertex-block-descent-solver/ogc-gap-audit.md),
  committed benchmark/profiling JSON for the new scenes, paper tetrahedral scene
  reproduction, Phase 8b SoA + Gaia-CPU comparison, and Phase 9 RTX-4090
  same-GPU Table 1 reproduction. Maintainer direction now promotes Augmented VBD
  (`avbd-2025`) as the next PLAN-104
  implementation focus: use
  [`104-vertex-block-descent-solver/avbd-paper-gap-audit.md`](104-vertex-block-descent-solver/avbd-paper-gap-audit.md)
  plus the active [`../dev_tasks/avbd_solver/`](../dev_tasks/avbd_solver/)
  tracker to implement AVBD's hard constraints, bounded contact/friction,
  finite-stiffness ramping, 6-DOF rigid/articulated blocks, soft/rigid coupling,
  all paper/demo scenes, and CPU/GPU benchmark parity. The local AVBD foundation
  now includes the internal scalar-row utility, deterministic row
  key/inventory warm-start cache, a standalone CPU half-space contact-normal
  block-descent kernel for active mass-spring rows, a scalar hard
  point-attachment row kernel/driver, a finite-stiffness spring row
  kernel/driver, a combined serial mass-spring AVBD row driver for those row
  families plus bounded friction tangents and self-contact normal rows, and
  narrow internal World opt-ins in the supported static-contact mass-spring,
  self-contact mass-spring, and pure-tetrahedral envelopes, with explicit
  fallback coverage for unsupported topology mixes, unrequested self-contact
  AVBD rows, Chebyshev, Rayleigh-damped, parallel, and unsupported-row requests;
  it is not a scene-level parity claim. The next local slice is static/dynamic
  friction switching, fuller contact/friction bounds, or self-contact friction
  in the same combined-row solve.
- Gate: VBD progress is not complete until the implementation distinguishes
  each internal kernel slice from a wired solver, keeps VBD naming
  backend-neutral, proves per-vertex force/Hessian correctness, PD Hessian
  guarantees, coloring conflict-freedom, and convergence parity with the
  existing solver and the reference implementations, adds FEM/acceleration/
  contact/friction with focused tests, records CPU and GPU benchmark/profiling
  JSON that beats the reference and/or paper numbers before any parity claim,
  verifies headless Filament captures for GUI examples, and keeps
  `pixi run lint`, `pixi run build`, focused C++ tests, and
  `check-api-boundaries` green. OGC additionally needs the sidecar source/code
  matrix, vertex-facet and edge-edge contact tests, conservative-bound and
  truncation tests, force/Hessian finite-difference evidence, limitation
  coverage, IPC/VBD comparison packets, and no public OGC/Gaia/Newton/Warp or
  backend-type leak. AVBD progress is not complete until DART
  implements every algorithm/feature from the paper, project page, videos, and
  `avbd-demo2d`/`avbd-demo3d`; provides CPU and GPU implementations for all
  solver and benchmark paths; ports every paper/site/video/demo experiment into
  tests, benchmarks, `py-demos`, and visual evidence; and records benchmark JSON
  proving DART beats both the reference demo repositories and the published
  paper numbers for every claimed CPU/GPU case.

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
- Status: Active
- Horizon: Now
- Dimension: Release transition
- Next step: Follow the DART 7 implementation order in the release roadmap:
  finish policy alignment and Gazebo lane split, publish the DART 6.16 support
  packet, then drive the clean-break parity sprint from the gate table:
  world-centric model loading, rigid-body dynamics parity,
  contact/constraint parity, serialization/replay, public API promotion, and
  DART 6.16 support policy. Keep research-solver breadth out of the DART 7
  release blocker set unless a promoted API depends on it.
- Gate: DART 7 is not release-ready until the clean-break gates in the release
  roadmap have direct evidence, package metadata no longer implies DART
  6/gz-physics compatibility, and DART 6.16 support scope plus sunset trigger
  are published.

### PLAN-050: Experimental World Split

- Owner doc:
  [`../onboarding/python-bindings.md#experimental-world-bindings-and-transition`](../onboarding/python-bindings.md#experimental-world-bindings-and-transition)
- Status: Complete
- Horizon: Later
- Dimension: Algorithm extensibility
- Next step: Track DART 7 promotion work under the release roadmap once the
  experimental world has parity gates.
- Gate: `dartpy.simulation_experimental` is separate from legacy
  `dartpy.simulation`, has focused import/API coverage, and the clean-break
  promotion path is documented in onboarding docs.

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

### PLAN-070: Post-DART-7 Compatibility Cleanup

- Owner doc: [`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md)
- Status: Parked
- Horizon: Later
- Dimension: Release transition
- Next step: Reopen only after the DART 7 clean break ships and the next major
  release has concrete DART 7-era compatibility debt to remove.
- Gate: Future DART 8 cleanup decisions cite migration notes, changelog
  entries, package/export status, and downstream support status where relevant.

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

### PLAN-110: Differentiable Simulation

- Owner doc:
  [`110-differentiable-simulation.md`](110-differentiable-simulation.md);
  durable design in
  [`../design/differentiable_simulation.md`](../design/differentiable_simulation.md).
- Status: Active
- Horizon: Next
- Dimension: Algorithm extensibility
- Scope note: WS1–WS5 first slices plus the PLAN-080-WS4 boxed-LCP contact
  prerequisite are implemented and verified (FD-of-step validated; default build
  untouched and bitwise-identical when off). The work is committed on
  `feature/differentiable-simulation` and open as PR #2761 (milestone DART 7.0,
  CHANGELOG included). Remaining for that path is hardening/examples/promotion,
  not new workstreams. Dojo is now tracked as a separate planned evaluation
  track under PLAN-110: maximal-coordinate variational hard-contact NCP/IPM with
  implicit gradients, using Dojo.jl as method evidence and a comparison baseline
  rather than a dependency.
- Next step: Drive PR #2761 to merge (experimental surface, single-branch to
  `main`). Then: run the torch-autograd test (`pip install torch`); add the
  standalone worked trajectory-optimization / system-identification example
  programs (the GUI demo and the convergence tests already ship); harden the
  static-friction Dantzig degenerate-pivot warning (shared `dart/math/lcp`, its
  own PR); and extend the deferred parameters/contacts (CENTER_OF_MASS,
  articulated multibody-link contact). After the active path lands, run the
  Dojo de-risking spike from
  [`110-differentiable-simulation/dojo-gap-audit.md`](110-differentiable-simulation/dojo-gap-audit.md)
  before any public API or runtime dependency promise. Track in
  `docs/dev_tasks/differentiable_simulation/`.
- Gate: differentiability is off by default with bitwise-identical results and
  zero snapshot allocation when off (`test_diff_zero_cost_parity` + on/off
  overhead benchmark against a stated budget); analytic Jacobians agree with
  central finite differences at relative error `< 1e-4` over `h ∈ {1e-5,1e-6,1e-7}`
  on named scenes (Tied-set/boundary configs excluded) before any parity claim;
  a `DART_BUILD_DIFF` build option with CI on _and_ off and a workflow guard;
  serialization round-trip (or documented non-serialization) of the
  `differentiable`/`contact_gradient_mode`/parameter-registration state; the
  public surface never exposes the reverse-pass cache, LCP snapshot,
  solver/coupler/backend types, ECS storage, or a tensor framework in the C++
  core (`check-api-boundaries` green); and mode-switch/limit subgradient and
  elastic-approximation limits are documented with saddle-escape and
  pre-contact surrogates opt-in only (no FD gate by construction).
  Dojo-style work additionally needs an internal NCP/IPM spike, central-path
  smoothness/accuracy contract, finite-difference gradient agreement, one
  reproduced Dojo example packet, benchmark/profiling JSON, and proof that no
  Dojo.jl dependency or solver/cache/backend type leaks into the public surface.

### PLAN-120: Inverse Kinematics And Motion Synthesis

- Owner doc:
  [`120-inverse-kinematics-and-motion.md`](120-inverse-kinematics-and-motion.md)
- Status: Proposed
- Horizon: Later
- Dimension: Algorithm extensibility
- Next step: Run the Phase 0 design inventory before implementation: map
  classic DART `InverseKinematics`/`WholeBodyIK`/`CompositeIK`/`IKFast` behavior
  to experimental `World` concepts; define the shared IK benchmark scene set;
  draft `docs/design/inverse_kinematics_motion.md`; and decide which pose,
  motion-level, and `auto` selection APIs can wait on existing experimental
  state-space, kinematics-only, rollout, collision-query, and model-loading
  seams.
- Gate: PLAN-120 is not implementation-ready until the design inventory proves
  manifold-correct state-space operations for mixed joint spaces, carries
  forward or intentionally retires each DART 6 whole-body IK feature, ranks
  Jacobian/analytical/optimization/heuristic/statistical/learned proposal
  families against shared scenes, defines long-horizon motion IK evidence that
  prevents per-frame discontinuities, singularity stalls, and local-minimum
  traps from being hidden, and specifies deterministic diagnostics for the
  `auto` policy.
