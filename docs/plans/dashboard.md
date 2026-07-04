# Plan Dashboard

This is the one-screen operating view for DART's living plans. It lists only
operating (non-`Complete`) plans in priority order. Completed plans move to
[`archive.md`](archive.md) in the same PR that completes them, so the dashboard
stays scannable.

This file is the single source of truth for the priority, status, horizon,
north-star dimension, next step, and gate of each operating plan. Each entry may
link to a detailed numbered initiative file or to the authoritative owner
document for initiatives tracked elsewhere until they warrant a dedicated plan
file. Detailed plan files own workstreams, acceptance criteria, and any
relocated progress log. External owner documents own the referenced scope until
a detailed initiative file is added. `north-star-roadmap.md` owns strategic
framing.

Each entry stays bounded: at most 40 lines per `### PLAN-` block and at most 15
lines in its `- Next step:` field. When an entry outgrows that, move the
historical or evidence narrative into the owner plan file's `## Progress log`
section and leave only the current next action plus a `History:` pointer.

## Operating View

Priority order follows document order. Keep each frequently changed field on
its own line so status updates remain git-history friendly.

### PLAN-122: DART 7 Simulation-Loop Allocation Hardening

- Owner doc:
  [`122-simulation-loop-allocation-hardening.md`](122-simulation-loop-allocation-hardening.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: Keep closing rows through the PLAN-122 coverage matrix. Current
  progress is 14/18 rows closed; `L-001` is now closed by first-post-bake
  world-base/global-heap/raw-malloc gates over a model imported through the
  `addSkeleton` bridge. There are no remaining implementation-capacity rows for
  currently selectable DART 7 CPU `World::step()` paths. `M-004`, `F-002`, and
  `G-001` are promotion-gated by their owner plans before they can become final
  allocation claims. Classic DART 6 step paths are excluded unless they are
  being migrated into the DART 7 `World` pipeline.
- Gate: A row closes only with cited tests proving same-shape DART 7
  `World::step()` after bake does not grow the World base allocator or allocate
  through global heap/raw malloc paths on measured hosts; migrated DART 7 paths
  must add the gate before promotion.

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

### PLAN-030: Compute Scalability Roadmap

- Owner doc:
  [`030-compute-scalability-roadmap.md`](030-compute-scalability-roadmap.md)
  (progress log and Phase 6 backlog); rationale owner
  [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md)
- Status: Active
- Horizon: Now
- Dimension: Scalable compute
- Next step: Phases 0-5 are complete and merged to `main`; the default DART 7
  `World::step` path preserves the rigid-body contact/multibody pipeline while
  the batched SoA rigid-body stage stays an explicit unconstrained/benchmark
  seam, and Phase 5 recorded a GO for the private, non-required CUDA path. The
  Phase 6 backlog (broaden GPU stage coverage, verified auto-scheduling,
  heterogeneous/single-scene GPU contact, differentiable state types) is
  unstarted; each item needs its own design note and gate first. See the Phase 6
  backlog and progress log in
  [`030-compute-scalability-roadmap.md`](030-compute-scalability-roadmap.md).
- Gate: `pixi run test-simulation-quick` covers graph/world parity for
  the current CPU foundation; `pixi run bm-compute-check` keeps the full
  expected `bm_compute_graph` corpus reproducible for the current Euler and
  contact-shaped workloads; the performance dashboard publishes the
  contact-shaped proxy, contact-island speedup surface, and Phase 5 CPU-baseline
  history, plus a bounded LCP solver/contact comparison slice for the contact
  foundation; `pixi run -e cuda test-all` is the local full CUDA gate on Linux
  CUDA hosts; `pixi run -e cuda test-cuda` remains the focused CUDA smoke path;
  and future compute-bound contact/constraint work must extend the checked
  benchmark gate.
  Taskflow remains behind the simulation executor boundary, metadata remains
  backend-neutral, CUDA remains private/non-required, and classic World behavior
  parity evidence comes from `release-6.*` branches.

### PLAN-080: Rigid-Body Dynamics Solver

- Owner doc:
  [`080-rigid-body-dynamics-solver.md`](080-rigid-body-dynamics-solver.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: The rigid-body MVP shipped (PR #2705, merged 2026-05-25). The
  active line now carries the model-loading bridge from legacy
  `dynamics::Skeleton` / `simulation::World` into DART 7 `Multibody`
  objects, including the retained `addSkeleton()` URI-loading facade,
  joint-family/property transfer, branching and root offsets, collision shape
  import with local transforms, compound shapes, broad-phase-pruned collision
  queries, and a persistent native collision-query world. The semi-implicit
  default pipeline also now runs one unified boxed-LCP over rigid-rigid and
  articulated link contacts. Continue the remaining Subsystem A polish in
  `docs/dev_tasks/rigid_body_dynamics_solver/`: warm starting, friction-cone
  iteration, and scaling work around the unified contact solve; keep richer
  model-loading diagnostics, visual/material import, the remaining reserved
  actuator modes (`Servo`/`Acceleration`; `Locked` landed), mimic/coupler,
  loop-closure, integrator, and COM-Jacobian work as separate deferred slices.
- Gate: Each slice keeps focused simulation tests and `check-api-boundaries`
  green, sources DART 6 parity evidence from `release-6.*` branch refs before
  any promotion claim, and never exposes solver/coupler/domain/backend types or
  ECS storage publicly.

### PLAN-081: Deformable Implicit-Barrier Solver

- Owner doc:
  [`081-deformable-implicit-barrier-solver.md`](081-deformable-implicit-barrier-solver.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: Implement the full mesh-backed IPC-class follow-up from the PLAN-081
  paper/repository gap audit: open a dedicated `docs/dev_tasks/` folder and work
  through mesh/material state, scene loading, BE/Newmark integration, PT/EE
  distance derivatives, conservative CCD line search, projected Newton, friction,
  diagnostics, and the upstream example/test/benchmark/visual corpus. Track
  Shortest Path to Boundary and Penetration-free Projective Dynamics on the GPU
  as separate sidecars (source audit first, then a CPU-verifiable slice) before
  any solver or public API claim, and route shared-primitive decisions through
  PLAN-083. History: see the progress log in
  [`081-deformable-implicit-barrier-solver.md`](081-deformable-implicit-barrier-solver.md).
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
- Next step: Continue the active rigid-IPC dev task toward production: curved
  CCD/residual/subdivision, local/scene barrier assembly and conservative line
  search, the opt-in runtime rigid-IPC stage and same-domain `World` solver
  selection, friction potentials and lagged passes, and remaining
  geometry/test/benchmark/visual parity. Keep the simultaneous-impact intake as a
  sidecar until solver-neutral scenes prove an uncovered gap, and coordinate any
  shared-primitive or ABD-replacement decision through PLAN-083. History: see the
  progress log in
  [`082-rigid-implicit-barrier-contact.md`](082-rigid-implicit-barrier-contact.md).
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

### PLAN-083: Unified Newton-Barrier Multibody Solver

- Owner doc:
  [`083-unified-newton-barrier-multibody.md`](083-unified-newton-barrier-multibody.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: Continue the active
  [`../dev_tasks/unified_newton_barrier_multibody/`](../dev_tasks/unified_newton_barrier_multibody/)
  consolidated follow-up (PR #2978): remaining private packet work for
  point-triangle/edge-edge contact stencils, broad-phase/candidate/CCD/barrier
  primitives, sparse assembly and solves, reduced scene-owned rows, and external
  surface-CCD diagnostics. Production runtime scene filtering, analytic curved
  CCD, scene-level line search, full sparse Hessian assembly, GPU `World::step`,
  paper-scale assets, and accepted reference timings remain future evidence; the
  completion audit still records PLAN-083 as incomplete, so dev-task retirement
  needs maintainer direction. History: see the progress log in
  [`083-unified-newton-barrier-multibody.md`](083-unified-newton-barrier-multibody.md).
- Gate: Unified Newton-barrier progress is not complete until every cited
  paper/deck figure, unit test, benchmark table, and comparison scene is mapped
  to DART-owned tests, py-demos examples, benchmark/profiling packets, CPU and
  GPU parity evidence, and explicit reference/paper-number comparisons; every
  new IPC-family solver or component records the solver-family intake checklist
  from [`solver-family-intake.md`](solver-family-intake.md) before
  adding a duplicate primitive or user-facing option; public APIs and solver
  options remain DART-owned, easy on the common `World` path, validated,
  serializable where result-affecting, and backend-neutral; `pixi run lint`,
  docs gates, focused C++/Python tests,
  benchmark smokes, and `check-api-boundaries` stay green for each promoted
  slice. PPF-derived cubic barriers, strain-limit rows, ACCD diagnostics,
  solver logs, shell/rod state adapters, or frontend/platform patterns require
  the PPF intake sidecar plus CPU/GPU comparison evidence before promotion.

### PLAN-104: Vertex Block Descent Solver

- Owner doc:
  [`104-vertex-block-descent-solver.md`](104-vertex-block-descent-solver.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: Maintainer direction promotes Augmented VBD (`avbd-2025`) as the
  current implementation focus: continue the active
  [`../dev_tasks/avbd_solver/`](../dev_tasks/avbd_solver/) tracker with
  [`104-vertex-block-descent-solver/avbd-paper-gap-audit.md`](104-vertex-block-descent-solver/avbd-paper-gap-audit.md)
  toward AVBD hard constraints, bounded contact/friction, finite-stiffness
  ramping, rigid/articulated blocks, all paper/demo scenes, and CPU/GPU benchmark
  parity, or implement the next missing row from
  [`104-vertex-block-descent-solver/avbd-demo-corpus.md`](104-vertex-block-descent-solver/avbd-demo-corpus.md).
  Remaining VBD closeout (self-contact tangential friction, OGC audit, benchmark
  JSON, paper tetrahedral reproduction, Phase 8b SoA + Gaia-CPU, Phase 9 same-GPU
  Table 1) stays tracked in the owner plan. History: see the progress log in
  [`104-vertex-block-descent-solver.md`](104-vertex-block-descent-solver.md).
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

### PLAN-105: Simplicits Geometry-Agnostic Elastic Solver

- Owner doc:
  [`105-simplicits-geometry-agnostic-elastic-solver.md`](105-simplicits-geometry-agnostic-elastic-solver.md)
- Status: Active
- Horizon: Next
- Dimension: Algorithm extensibility
- Next step: Use the PLAN-105 source list and solver-family intake to build a
  corpus manifest before implementation. The manifest must classify every
  Simplicits paper/project/video/Kaolin row into tests, benchmark JSON,
  `py-demos`, visual evidence, CPU reference comparison, GPU parity, and DART 7
  pipeline surface. Only after that, start a dedicated `docs/dev_tasks/`
  tracker and prototype the
  first CPU baked-reduced-basis slice without exposing Simplicits, Kaolin, Warp,
  Torch, CUDA, solver-registry, ECS, or backend-resource types through the
  public API.
- Gate: Simplicits progress is not complete until DART implements the
  occupancy/point-sampling pipeline, learned or baked skinning-weight reduced
  basis, reduced mass/deformation-gradient operators, implicit
  time-integration/Newton solve, boundary/floor/gravity controls, contact,
  friction, CPU and GPU runtime paths, all paper/site/video/Kaolin demos and
  supplemental-table benchmark rows, and benchmark JSON proving DART beats
  Kaolin/reference and paper numbers at matched scene parameters. Every slice
  must keep public APIs DART-owned and backend-neutral and keep
  `pixi run lint`, `pixi run build`, focused C++/Python tests, benchmark smokes,
  `check-api-boundaries`, and relevant CUDA gates green.

### PLAN-084: Linear-Time Variational Integrator

- Owner doc:
  [`084-variational-integrator-solver.md`](084-variational-integrator-solver.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: The VI selector, O(n) inverse-mass path, floating/spherical support,
  loop closures, scoped C1-C3 contact/friction, dartpy surface, and supported
  envelope have landed. Continue either by opening the maintainer-owned
  graduation proposal from
  [`084-variational-integrator-solver/graduation-criteria.md`](084-variational-integrator-solver/graduation-criteria.md)
  or by starting the separate arbitrary-geometry contact adapter workstream
  coordinated with the rigid IPC / deformable IPC geometry stack.
- Gate: The implemented family remains opt-in behind `MultibodyOptions`;
  `check-api-boundaries` stays green with no solver/stage/component/backend leak;
  regression evidence keeps symplectic energy behavior, O(n) scaling,
  deterministic serialization, loop closures, and the declared contact/friction
  envelope green. Graduation requires the maintainer-owned `PLAN-` entry and
  adversarial review; arbitrary link geometry and C4 hard-barrier contact remain
  explicit follow-up work, not hidden gaps.

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

### PLAN-040: DART 7 Release Hardening

- Owner doc: [`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md)
- Status: Active
- Horizon: Now
- Dimension: Release transition
- Next step: Follow the DART 7 implementation order in the release roadmap:
  finish policy alignment and Gazebo lane split, publish the `release-6.*` support
  packet, then settle PLAN-042 public API/source-layout topology before freezing
  PLAN-041 official simulation API promotion. Keep research-solver breadth out
  of the DART 7 release blocker set unless a promoted API depends on it.
- Gate: DART 7 is not release-ready until the clean-break gates in the release
  roadmap have direct evidence, package metadata no longer implies DART
  6/gz-physics compatibility, and `release-6.*` support scope plus sunset trigger
  are published.

### PLAN-041: Official Simulation API Promotion

- Owner doc:
  [`041-official-simulation-api-promotion.md`](041-official-simulation-api-promotion.md)
- Status: Active
- Horizon: Now
- Dimension: Release transition
- Next step: Core promotion has landed (`dart::simulation::World`,
  `dartpy.simulation.World`, and `dartpy.World` back the ECS facade; stubs no
  longer publish `dartpy.simulation_experimental`; the classic Python world is
  quarantined as `dartpy.gui.RenderWorld`). Remaining work is hardening and
  cleanup: finish stale docs, keep the public-header/package/import guards green,
  add negative smokes for retired experimental paths/targets, and keep
  strict-final World-promotion checks green with `release-6.*` parity evidence.
  Public scalar-precision selectors stay deferred to a separate
  scalar-instantiation plan. History: see the progress log in
  [`041-official-simulation-api-promotion.md`](041-official-simulation-api-promotion.md).
- Gate: The planning PR passes the docs-only gates; implementation PRs must keep
  promotion-aware API-boundary checks, C++/Python tests, package/export smokes,
  `check-dart7-promotion-surface`,
  `check-dart7-promotion-package-contract`,
  `check-dart7-promotion-installed-package`, `check-dartpy-import-layout`,
  stub/API-doc regeneration, and CUDA/full gates green according to the touched
  scope. The
  promoted public API must hide ECS, component, solver-registry, backend,
  implementation-folder, tensor framework, and unplanned scalar-instantiation
  details, and the installed package must expose only final headers, final CMake
  targets/components, and final dartpy module paths once promotion is claimed.

### PLAN-042: DART 7 Public API And Source Layout

- Owner doc:
  [`042-dart7-public-api-and-source-layout.md`](042-dart7-public-api-and-source-layout.md)
- Status: Active
- Horizon: Now
- Dimension: Easy start
- Next step: Treat the default packet as implemented and guarded
  (`dart.World is dart.simulation.World`, no generated
  `simulation_experimental` stubs, `dart.simulation.diff` / `dart.diff` shared
  module, classic render world isolated under `dart.gui.RenderWorld`,
  `dart::simulation::World` as the C++ owner, and `dart-simulation` as the
  package target/component). The current `dart/simulation` folder tree is the
  accepted guarded post-promotion layout, not the final whole-repo taxonomy; use
  the post-promotion source-layout decision sidecar for compute, IO, state/space,
  diff, and legacy dynamics follow-ups before moving files. Remaining work is
  negative smokes and docs cleanup for removed DART 6 or experimental paths.
  Parity references should come from `release-6.*` branches, not main-branch
  classic World tests. Final local gate: `pixi run
check-dart7-final-world-promotion`.
- Gate: The planning PR passes the docs-only gates; follow-up implementation
  PRs must prove final examples, stubs/docs, package exports, API boundaries,
  C++/Python tests, `check-dart7-promotion-package-contract`,
  `check-dart7-promotion-installed-package`, `check-dartpy-import-layout`,
  feature-off source/wheel behavior, case-insensitive header behavior, and
  negative smokes for removed DART 6 or experimental paths.

### PLAN-070: Post-DART-7 Compatibility Cleanup

- Owner doc: [`../onboarding/release-roadmap.md`](../onboarding/release-roadmap.md)
- Status: Parked
- Horizon: Later
- Dimension: Release transition
- Next step: Reopen only after the DART 7 clean break ships and the next major
  release has concrete DART 7-era compatibility debt to remove.
- Gate: Future DART 8 cleanup decisions cite migration notes, changelog
  entries, package/export status, and downstream support status where relevant.

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
  the design doc; its remaining work is World-API-gated and tracked there:
  runtime sensor output panes + joint render layers/visibility filters, richer
  relationship inspectors/grouping, a Scene Tree context-menu popup affordance,
  extracting the `editor.cpp` project file-dialog flow into its own tested seam,
  and adopting simulation shape/loader APIs (replace editor-side shape
  descriptors) per PLAN-050.
- Gate: The headless engine is covered by command/undo, object, selection,
  name-uniqueness, and project-round-trip tests with zero GUI/renderer includes;
  the filtered `dartsim/engine/*` + `dartsim/ui/*_actions` surface holds ≥95%
  line coverage (`pixi run coverage-report-dartsim`); the editor loop (design →
  run → record → replay) works on the DART 7 World only; the renderer
  stays backend-hidden (PLAN-060), enforced for `dartsim/engine` and
  `dartsim/ui` by `scripts/check_api_boundaries.py`; and the default `dartsim`
  headless smoke (`DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS`) renders a non-blank
  editor frame while legacy `--scene` smokes keep contrast.

### PLAN-103: Examples Strategy (Python-First)

- Owner doc: [`103-examples-strategy.md`](103-examples-strategy.md); C++ app
  architecture in [`../design/demos_app.md`](../design/demos_app.md).
- Status: Active
- Horizon: Now
- Dimension: Easy start
- Next step: Phases 1–4 landed and the catalog has been pruned for DART 7:
  `py-demos` is the World demo surface (World rigid body, Rigid IPC,
  Variational Integrators, Differentiable, Vertex Block Descent, and IPC
  Deformable categories, plus `Planned World Ports` placeholders) running
  through `dartpy.gui.run_demos`; C++ `dart-demos` is the smaller World-only
  companion with matching planned-port placeholders. The old DART 6 demo scenes
  and cross-language golden parity fixtures are removed from the demo surfaces.
  The `docs/dev_tasks/examples_strategy/` folder is retired; residual follow-ups
  are tracked in PLAN-103's Landed State + retire-later checklist.
- Gate: `pixi run py-demos -- --cycle-scenes --headless --frames 1` cycles all
  scenes (exit 0, with per-scene progress); the notebook imports (not copies)
  the scene modules; C++ `dart-demos` cycles its World scenes; `pixi run lint`
  and `check-docs-policy` green.

### PLAN-110: Differentiable Simulation

- Owner doc:
  [`110-differentiable-simulation.md`](110-differentiable-simulation.md);
  durable design in
  [`../design/differentiable_simulation.md`](../design/differentiable_simulation.md).
- Status: Active
- Horizon: Next
- Dimension: Algorithm extensibility
- Next step: The WS1–WS5 differentiability surface is merged to `main` (PR #2761);
  remaining work is hardening/examples plus the Dojo de-risking spike. Run the
  torch-autograd test (`pip install torch`); add standalone
  trajectory-optimization / system-identification example programs; harden the
  static-friction Dantzig degenerate-pivot warning (shared `dart/math/lcp`, its
  own PR); extend deferred parameters/contacts (CENTER_OF_MASS, articulated
  multibody-link contact); then run the Dojo spike from
  [`110-differentiable-simulation/dojo-gap-audit.md`](110-differentiable-simulation/dojo-gap-audit.md)
  before any public API or runtime-dependency promise. Track in
  `docs/dev_tasks/differentiable_simulation/`. History: see the progress log in
  [`110-differentiable-simulation.md`](110-differentiable-simulation.md).
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
  to DART 7 `World` concepts; define the shared IK benchmark scene set;
  draft `docs/design/inverse_kinematics_motion.md`; and decide which pose,
  motion-level, and `auto` selection APIs can wait on existing DART 7
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
