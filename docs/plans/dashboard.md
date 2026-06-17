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

### PLAN-091: DART 7 Architecture Hardening

- Owner doc:
  [`091-architecture-hardening.md`](091-architecture-hardening.md)
- Status: Active
- Horizon: Now
- Dimension: Algorithm extensibility
- Next step: WP-091.20 (Model/State/Control split) is `[done]` via PR #3029,
  WP-091.33a (batch semantics tests) is `[done]` via PR #3042, and WP-091.21
  (baked dense-index Model artifact) is `[done]` via PR #3044. The current
  branch executes WP-091.33b: give the rigid batch seed an internal baked
  Model/State owner that reuses immutable Model storage across rollout
  segments and invalidates on structural edits. The remaining WS0 packet is
  WP-091.4 legacy freeze, which stays **blocked**: PLAN-042 Decision 5 has no
  recorded maintainer direction. Packets are orchestrator-authored per
  [`../ai/orchestration.md`](../ai/orchestration.md) and picked up via
  `dart-execute-packet`; availability follows each packet's own Dependencies
  line. The standing rule applies now: new solver-family work routes through
  [`solver-family-intake.md`](solver-family-intake.md) and does not bypass the
  contracts this plan is landing.
- Gate: A packet is done only when its named acceptance evidence exists and
  its listed `pixi run ...` gates pass, with availability governed by the
  per-packet Dependencies lines; plan completion follows the acceptance
  criteria in the owner doc, including one new solver-family intake exercised
  through the strengthened checklist without bypass.

### PLAN-001: Living Plan System

- Owner doc: [`README.md#plan-update-workflow`](README.md#plan-update-workflow)
- Status: Complete
- Horizon: Later
- Dimension: AI-native execution
- Next step: Use the recorded structural checks whenever plan workflows,
  generated adapters, required-reading paths, or dev-task shape rules change.
- Gate: `check-ai-commands`, `check-docs-policy`, and `sync-ai-commands` cover
  adapter sync, required-reading coverage, and dev-task shape.

### PLAN-121: AI Docs Knowledge Graph Guardrails

- Owner doc:
  [`121-ai-docs-knowledge-graph.md`](121-ai-docs-knowledge-graph.md)
- Status: Active
- Horizon: Now
- Dimension: AI-native execution
- Next step: Land the first guardrail wave as pilot-scoped graph-health
  advisories plus strict low-risk metadata/catalog checks, then watch for one
  clean cycle before deciding whether any advisory should become blocking.
- Gate: `pixi run python -m pytest tests/test_check_docs_policy.py`,
  `pixi run check-docs-policy`, `pixi run check-ai-commands`, and
  `pixi run lint` prove the local checker, AI adapter sync, and docs formatting
  surfaces.

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
- Next step: The DART 7 LCP evidence campaign is complete in
  [#2962](https://github.com/dartsim/dart/pull/2962); use its LCP contract,
  tests, benchmark packets, and `lcp_physics` example as the template when the
  next algorithm family is selected. For solver or multi-physics papers, first
  apply the solver-family intake checklist in
  [`solver-family-intake.md`](solver-family-intake.md)
  so the work routes to an existing family, shares common collision,
  kinematics, and optimization components, and defines apples-to-apples
  evidence plus a user-facing configuration shape.
- Gate: LCP contract docs, focused tests, smoke benchmark, API-boundary
  exclusions, baseline example evidence, and the solver-family intake checklist
  in [`solver-family-intake.md`](solver-family-intake.md) are recorded
  before a new solver family or paper implementation starts; for solvers, that
  checklist includes simple `World` defaults, method-specific advanced options,
  validation, serialization expectations, and diagnostics.

### PLAN-030: Compute Scalability Roadmap

- Owner doc:
  [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md)
- Status: Active
- Horizon: Now
- Dimension: Scalable compute
- Next step: Phases 0-5 are complete and merged to `main` (PRs #2698, #2710,
  #2712); the dev-task folder has been retired, so PLAN-030 plus
  [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md)
  are now the durable trackers. The default DART 7 `World::step` path
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

### PLAN-031: Shared Experimental CUDA Device Substrate

- Owner doc:
  [`../design/shared_cuda_device_substrate.md`](../design/shared_cuda_device_substrate.md)
- Status: Complete
- Horizon: Later
- Dimension: Scalable compute
- Next step: The build-now substrate landed on `main` in PR #2875. Use
  [`../design/shared_cuda_device_substrate.md`](../design/shared_cuda_device_substrate.md)
  as the extraction contract when AVBD-CUDA (PLAN-104), rigid-IPC GPU
  (PLAN-082), PD-IPC GPU (PLAN-081), or broadened Phase-6 compute work
  introduces a real second consumer for deferred rollout, graph-capture,
  precision, colored-sweep, accelerator-registry, device IPC math, residency, or
  reduction helpers. No new general library is planned: reuse `dart/math`,
  `dart/optimizer`, `detail/newton_barrier`, and
  `deformable_psd_backend`, and promote the same PLAN-083 cores
  host/device-portable only when a GPU IPC consumer lands.
- Gate: Future substrate changes must keep shared device code `.cuh`/`.cpp`-only
  under `compute/cuda/` (plus scanner-skipped `detail/` kernel cores), leave the
  CUDA sidecar build-only and uninstalled, preserve the default no-GPU path, and
  keep `pixi run -e cuda test-cuda`,
  `pixi run test-simulation`,
  `pixi run check-api-boundaries`,
  `pixi run check-compute-backend-boundaries`, and
  `pixi run check-no-gpu-runtime-dependencies` green for any promoted second-use
  extraction.

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
  model-loading diagnostics, visual/material import, actuator, mimic/coupler,
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
  fast-CCD validation before any A-Jacobi, GPU-culling, or speedup claim. Use
  PLAN-083 to decide which distance, barrier, tangent, CCD, friction, PSD,
  sparse-Newton, diagnostics, benchmark, and visual-evidence primitives should
  become shared Newton-barrier infrastructure rather than another
  deformable-local variant.
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
  AVBD-style finite-time contact. Coordinate any shared primitive extraction or
  ABD replacement decision through PLAN-083 before changing the rigid IPC
  correctness-oracle role.
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
  consolidated follow-up on
  `simx/plan083-gpu-contact-candidate-packet` / PR #2978. PR #2960 landed
  implementation-roadmap Phases 3-8; PR #2961 measured the private GPU PSD
  projection packet, added the Fig. 17 barrier-force diagnostic, and aligned
  articulation-only figure rows with landed private diagnostics; PR #2970
  landed runtime wiring for point/fixed and hinge constraints, opt-in BDF-2,
  deformable fixed obstacles, and the reduced hanging-bridge py-demo; PR #2971
  landed reduced CPU packets for lying-flat, hanging-bridge, pulley, umbrella,
  terrain vehicle, ragdoll, nunchaku, nunchaku scaling, windmill, Candy,
  precession, reduced timing-breakdown, reduced Table 2, the sparse equality
  change-of-variable rigid IPC path, and the reduced affine point-triangle
  micro-solve diagnostic; PR #2974 added reduced ABD runtime-step evidence for
  `abd-vs-rigid-cards`, `abd-vs-rigid-wreck`, `abd-chain-8`, `abd-chain-16`,
  and `abd-chain-96`; PR #2976 adds reduced ABD gears/Bullet comparison
  packets without claiming gear assets, Bullet/reference baselines, GPU parity,
  or paper-scale parity. PR #2978 is the single consolidated follow-up for the
  remaining private packet work: it adds point-triangle and edge-edge contact
  stencil parity plus brute-force all-pairs point-triangle and edge-edge
  candidate masks and motion-aware swept-AABB point-triangle/edge-edge
  candidate-list packets with device-side compacted candidate ids and distance
  metadata, compact device-sorted sweep-and-prune broad-phase packets, plus
  compact runtime sweep-buffer endpoint-distance packets, reduced
  scene-owned runtime candidate-buffer packets, and reduced scene-owned
  runtime sweep broad-phase packets, plus reduced combined scene runtime
  sweep-filter and candidate-filter rows extracted from the same DART `World`
  surface,
  endpoint-linear point-triangle/edge-edge CCD/line-search parity plus sampled
  rigid-curved point-triangle/edge-edge CCD/line-search parity plus reduced
  scene-owned runtime point-triangle/edge-edge CCD rows plus a reduced combined
  scene runtime CCD line-search row, scalar
  barrier/friction local-kernel parity
  plus point-triangle
  primitive barrier-gradient and point-triangle/edge-edge/point-edge/point-point
  tangent-stencil parity, point-triangle/point-point/point-edge/edge-edge
  primitive barrier-Hessian parity, point-triangle/point-point/point-edge
  primitive barrier-Hessian PSD-projection parity, reduced scene-owned
  point-triangle, point-edge, point-point, and edge-edge barrier-Hessian runtime
  rows plus a reduced combined all-family scene runtime barrier-Hessian row,
  reduced diagonal assembly/solve plus
  pair-slot off-diagonal sparse-block assembly plus a reduced scene-owned
  sparse off-diagonal surface-edge assembly row plus a reduced scene-owned
  sparse graph construction/assembly row plus a reduced scene-owned nonlinear
  distance-equality assembly row plus a reduced scene-owned nonlinear
  distance-equality solve row plus a reduced scene-owned capped nonlinear
  distance-equality convergence row plus sparse block residual matvec plus a
  reduced scene-owned sparse residual row plus fixed-iteration sparse Jacobi
  solve plus a reduced scene-owned sparse Jacobi row plus capped sparse CG solve
  plus a reduced scene-owned sparse CG row plus bounded reduced direct sparse
  factor solve plus reduced scene-owned bounded direct sparse factor solve plus
  sparse equality-reduced diagonal solve parity plus a reduced scene-owned
  equality-reduced diagonal solve row plus a reduced scene-owned sparse graph
  unique-edge dedup row plus a reduced scene-owned diagonal assembly/solve row,
  reduced hanging-bridge scene
  state-batch CPU/GPU parity and speedup, reduced ABD complex-geometry packets,
  an ABD/FEM coupled micro-solve packet with external surface CCD sidecar
  witnesses, built-in deformable `World::step` self-surface candidate/CCD
  diagnostics in the reduced lying-flat, Candy, and ABD/FEM CPU scene packets,
  and public built-in inter-body/static-rigid/
  moving-rigid surface CCD diagnostics, then serializes those external counters
  into the reduced deformable CPU scene packet rows and adds a dedicated reduced
  external surface CCD CPU diagnostic packet with nonzero inter-body/static-rigid/
  moving-rigid counters, including one mixed reduced `World::step` witness that
  activates all three external families, plus reduced lying-flat
  inter-body/static-rigid/moving-rigid surface CCD witness rows, one reduced
  hanging-bridge inter-body/static-rigid/moving-rigid external CCD sidecar row,
  one reduced pulley inter-body/static-rigid/moving-rigid external CCD sidecar
  row,
  one reduced umbrella inter-body/static-rigid/moving-rigid external CCD sidecar
  row, one reduced terrain vehicle inter-body/static-rigid/moving-rigid
  external CCD sidecar row, one reduced ragdoll inter-body/static-rigid/
  moving-rigid external CCD sidecar row, one reduced precession
  inter-body/static-rigid/moving-rigid external CCD sidecar row, one reduced
  Candy static-rigid/moving-rigid surface CCD witness row, and one reduced
  ABD/FEM external sidecar witness row. Other
  broader figure/demo scene rows still have zero external candidate/check/hit
  counts.
  It still keeps
  production runtime scene filtering, production analytic curved CCD,
  production scene-level line search inside
  `World::step`, full runtime
  sparse Hessian graph construction and assembly beyond the reduced dedup row,
  unbounded production direct/global sparse factorization, production nonlinear equality convergence
  policy/solving, GPU `World::step`, paper-scale assets, full runtime
  affine/FEM coupling, production runtime scene filtering inside `World::step`,
  and accepted reference timings as future evidence. The
  completion audit still records PLAN-083 as incomplete while in-progress
  CPU/GPU/scene limitations remain, so dev-task retirement needs maintainer
  direction before deletion.
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
  families plus bounded friction tangents, self-contact normal rows, and
  self-contact friction rows, and narrow internal World opt-ins in the
  supported static-contact mass-spring, self-contact mass-spring, and
  pure-tetrahedral envelopes. Pure-tetrahedral finite-stiffness scenes can now
  also opt into AVBD self-contact normal rows and matching self-contact
  friction rows in the same serial tet solve, with explicit
  fallback coverage for unsupported topology mixes, unrequested self-contact
  AVBD rows, Chebyshev, Rayleigh-damped, parallel, and unsupported-row requests;
  adjacent friction tangent pairs now use lagged-dual static/dynamic switching
  and pairwise circular-cone projection, including supported World generation
  for self-contact friction rows. Static box obstacle row keys now distinguish
  faces, edges, and corners so same-feature rows warm-start while box-manifold
  changes reset normal/friction state; persisting static half-space friction
  rows also project their decayed dual into the current tangent basis when smooth
  obstacle normals change, and persisting self-contact friction rows project
  their generalized tangential dual into the current 12D tangent stencil. The
  first private rigid foundation adds a 6-DOF block accumulator, world-frame
  quaternion tangent update, inertia term, block solve, scalar rigid
  point-attachment row, two-body point-pair row stamping, and private
  point-pair contact/friction row constructors plus paired friction-cone
  projection, with a private serial row driver for point attachments,
  contact-normal point pairs, and paired friction tangent rows. Paired friction
  row updates now reuse the regularized tangent constraint vector for both cone
  projection and stiffness growth, shared-anchor tangent pairs reuse the first
  transformed anchor pair across both tangent rows, and a private
  rigid contact-manifold row builder for active contact points. Private
  dynamic/rigid contact feature IDs, canonical two-endpoint row keys, and
  normal/friction row descriptor helpers have started, and private
  World-contact snapshot/solve/writeback helpers now translate rigid-body
  `World::collide()` contacts into manifold-point inputs, run them through the
  private serial rigid row solve, and write dynamic rigid-body state back to the
  ECS through a combined private wrapper in focused tests. The first
  contact-stage AVBD activation is now available behind the internal
  `RigidAvbdContactConfig`: supported free rigid-body contacts route through
  the private 6-DOF AVBD row solve as a velocity-level projection, with focused
  dynamic/dynamic and static/dynamic contact-stage coverage in
  `World.RigidBodyContactStageAvbdProjectsDynamicDynamicContactVelocity` and
  `World.RigidBodyContactStageAvbdProjectsStaticDynamicContactVelocity`,
  single-config dynamic/dynamic opt-in coverage in
  `World.RigidBodyContactStageAvbdProjectsDynamicPairWithSingleConfig`,
  default `World::step()` dynamic/dynamic coverage in
  `World.RigidBodyContactStageAvbdDynamicDynamicRunsThroughDefaultWorldStep`,
  default `World::step()` single-config dynamic/dynamic coverage in
  `World.RigidBodyContactStageAvbdDynamicPairWithSingleConfigRunsThroughDefaultWorldStep`,
  contact-to-position pipeline coverage in
  `World.RigidBodyContactStageAvbdFeedsRigidBodyPositionStage`,
  default `World::step()` schedule coverage in
  `World.RigidBodyContactStageAvbdRunsThroughDefaultWorldStep`,
  static-owned static/dynamic opt-in coverage in
  `World.RigidBodyContactStageAvbdProjectsStaticOwnedContactConfig`,
  default `World::step()` static-owned static/dynamic opt-in coverage in
  `World.RigidBodyContactStageAvbdStaticOwnedRunsThroughDefaultWorldStep`,
  stored static-body velocity ignore coverage in
  `World.RigidBodyContactStageAvbdIgnoresStoredStaticVelocity`,
  kinematic-as-prescribed endpoint coverage in
  `World.RigidBodyContactStageAvbdTreatsKinematicBodyAsStaticObstacle`,
  default `World::step()` kinematic-prescribed endpoint coverage in
  `World.RigidBodyContactStageAvbdKinematicRunsThroughDefaultWorldStep`,
  enabled-peer/disabled-peer opt-in coverage in
  `World.RigidBodyContactStageAvbdProjectsEnabledPeerWithDisabledConfig`,
  default `World::step()` enabled-peer/disabled-peer coverage in
  `World.RigidBodyContactStageAvbdEnabledPeerWithDisabledConfigRunsThroughDefaultWorldStep`,
  configured multi-contact coverage in
  `World.RigidBodyContactStageAvbdProjectsMultipleConfiguredContacts`, plus
  default `World::step()` configured multi-contact coverage in
  `World.RigidBodyContactStageAvbdMultipleConfiguredContactsRunThroughDefaultWorldStep`,
  mixed-config all-or-nothing fallback coverage in
  `World.RigidBodyContactStageAvbdFallsBackForUnconfiguredContactSet`,
  default `World::step()` mixed-config all-or-nothing fallback coverage in
  `World.RigidBodyContactStageAvbdMixedConfigFallsBackThroughDefaultWorldStep`,
  disabled-config opt-out fallback coverage in
  `World.RigidBodyContactStageAvbdDisabledConfigFallsBack`, default
  `World::step()` disabled-config fallback coverage in
  `World.RigidBodyContactStageAvbdDisabledConfigFallsBackThroughDefaultWorldStep`,
  and
  static/dynamic and dynamic/dynamic warm-started friction slip-reduction
  coverage in `World.RigidBodyContactStageAvbdWarmStartedFrictionReducesSlide`
  and
  `World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionReducesSlip`,
  static-owned warm-started friction coverage in
  `World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionReducesSlide`,
  kinematic-owned warm-started friction coverage in
  `World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionReducesSlide`,
  enabled-peer/disabled-peer warm-started friction coverage in
  `World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionReducesSlide`,
  simultaneous multi-contact warm-started friction coverage in
  `World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionReducesSlide`,
  live box-box manifold warm-started friction coverage in
  `World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionReducesSlide`,
  live dynamic/dynamic box-manifold warm-started friction coverage in
  `World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionReducesSlip`,
  stacked static/dynamic plus dynamic/dynamic box-manifold warm-started
  friction coverage in
  `World.RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionReducesSlip`,
  multi-top stacked box-manifold warm-started friction contact-stage coverage in
  `World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionReducesSlip`,
  plus default `World::step()` warm-started friction schedule coverage in
  `World.RigidBodyContactStageAvbdWarmStartedFrictionRunsThroughDefaultWorldStep`,
  `World.RigidBodyContactStageAvbdWarmStartedStaticOwnedFrictionRunsThroughDefaultWorldStep`,
  `World.RigidBodyContactStageAvbdWarmStartedKinematicOwnedFrictionRunsThroughDefaultWorldStep`,
  `World.RigidBodyContactStageAvbdWarmStartedEnabledPeerWithDisabledConfigFrictionRunsThroughDefaultWorldStep`,
  `World.RigidBodyContactStageAvbdWarmStartedMultipleConfiguredContactsFrictionRunsThroughDefaultWorldStep`,
  `World.RigidBodyContactStageAvbdWarmStartedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
  `World.RigidBodyContactStageAvbdWarmStartedDynamicBoxManifoldFrictionRunsThroughDefaultWorldStep`,
  `World.RigidBodyContactStageAvbdWarmStartedStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
  `World.RigidBodyContactStageAvbdWarmStartedMultiTopStackedBoxManifoldFrictionRunsThroughDefaultWorldStep`,
  and
  `World.RigidBodyContactStageAvbdWarmStartedDynamicFrictionRunsThroughDefaultWorldStep`,
  while
  unsupported envelopes fall back to the existing sequential-impulse path. The
  private rigid contact snapshot now derives box face/edge/corner endpoint
  feature IDs, scopes row ordinals per canonical endpoint pair, and assigns
  same-feature manifold rows from deterministic canonical-local point ordering
  for narrower warm-start persistence, including endpoint-order row-identity
  evidence, actual `World::collide()` sphere/plane contact-point replay,
  sphere/mesh-face, sphere/mesh-edge, mesh-vertex replay, and
  mesh-face/mesh-edge/mesh-vertex small-pose persistence plus endpoint-order
  stability,
  cylinder/capsule cap/side/rim small-pose persistence and endpoint-order
  stability, and live box-box manifold box-face feature evidence. The first
  wider live box-pile friction row-persistence regression now preserves per-pair
  row ordinals, friction coefficients, Coulomb bounds, and warm-started friction
  dual state across a small pose nudge; the multi-top companion preserves the
  same row state across two lower supports and two independent upper dynamic
  bodies; the contact-order replay companion
  keeps those row keys and lambdas stable when the live contact vector is
  reversed; and the endpoint-order regression keeps those per-pair bounds while
  projecting paired tangent duals into the swapped physical tangent basis. The
  live contact-stage/default-step pile companions now carry the same two-lower
  plus spanning-top stack through `RigidBodyContactStage`, reducing top/lower
  tangential slip in both schedules.
  Cylinder side/cap/rim and
  capsule side/top-cap/bottom-cap endpoint
  features extend the same private identity path beyond boxes, and known and
  unknown-index contacts now map world points through body and collision-shape
  local transforms before feature coding,
  while explicit endpoint-A/B compound shape-index feature coding uses
  narrow-phase shape-local contact points; actual `World::collide()`
  sphere/cylinder/capsule/plane/mesh contacts now cover the primitive-feature
  path, including shape-scoped sphere body features, and
  single-shape and uniquely containing compound-shape fallback coverage remains
  in the rigid snapshot path.
  Persisting private rigid contact friction rows now retain their
  previous tangent directions and project the warm-started paired dual into the
  current tangent basis when the contact normal rotates. The private rigid row
  path now has point-joint linear, angular,
  and combined row builders for fixed-anchor
  translation and orientation constraints, with step-start previous values
  seeded for AVBD alpha regularization. Those private point-joint rows can now
  be appended to the World rigid snapshot/solve/apply wrapper and combined step
  helper from world-space point-joint inputs, and a private fixed-joint ECS
  extractor can feed the step helper for rigid-body-linked joint entities. The
  internal contact-stage AVBD opt-in can project those fixed-joint rows with or
  without active contacts. The private point-joint builders now accept per-axis
  linear and angular masks, preserving all-axis fixed-joint behavior while
  enabling limited-DOF configs to reuse the same descriptor and warm-start path.
  Named private revolute and prismatic point-joint configs now build arbitrary
  joint-axis bases, leave one rotational or translational axis free, and
  preserve axes/masks through World point-joint input and solve coverage.
  Simulation-entry current-pose initialization and extraction also cover
  private rigid-body ECS revolute/prismatic joint entities. Public experimental
  `World` facades now expose free rigid-body revolute and prismatic joints
  through C++ and dartpy, backed by generated stubs, focused C++/Python tests,
  and the categorized `sx_rigid_limited_joints` py-demo. The first AVBD
  rigid-constraint `py-demos` scene,
  `avbd_rigid_fixed_joint_contact`, exposes the fixed-joint/contact slice as a
  user-visible showcase. Public free-rigid-body revolute/prismatic velocity
  actuators now extract to private bounded AVBD angular/linear motor rows, with
  persistent contact-stage motor inventory, categorized
  `avbd_rigid_revolute_motor` and `avbd_rigid_prismatic_motor` py-demos,
  dashboard benchmark rows for those public motor paths, and tracked
  visual/benchmark packets.
  Public articulated one-DOF velocity motors now also expose command-update
  behavior through the categorized `avbd_articulated_revolute_motor` and
  `avbd_articulated_prismatic_motor` py-demos, with tracked visual/benchmark
  packets for both paths, while
  `avbd_articulated_motor_breakable_joint` exposes the same-multibody public
  articulated motor break/reset lifecycle with post-reset reversed-command
  plus weak re-arm coverage and a tracked packet,
  `avbd_articulated_prismatic_pair_motor_breakable_joint` exposes the
  same-multibody public articulated prismatic motor break/reset lifecycle with
  post-reset reversed-command plus weak re-arm coverage and a tracked packet,
  and
  `avbd_articulated_prismatic_motor_breakable_joint` exposes the world-anchored
  public articulated prismatic motor break/reset lifecycle with a tracked
  packet plus post-reset reversed-command and weak re-arm coverage, while
  `avbd_articulated_world_revolute_motor_breakable_joint` exposes the
  world-anchored public articulated revolute motor break/reset lifecycle with a
  tracked packet and post-reset reversed-command plus weak re-arm coverage. The
  bounded AVBD World
  dashboard slice now includes matching public articulated
  revolute/prismatic/breakable motor step rows, the active prismatic
  breakable-motor step row, the active world-prismatic breakable-motor step
  row, the active world-revolute breakable-motor step row, plus public
  free-rigid/articulated breakable fixed point-joint step rows backed by
  tracked visual/benchmark packets, plus public free-rigid/world-link/
  same-multibody spherical breakable point-joint rows backed by tracked packets.
  Public free-rigid-body point joints also expose a narrow break-force and
  broken-state lifecycle, mark solved AVBD point joints broken when row load
  reaches the threshold, skip broken joints during later extraction, and expose
  fixed/revolute/prismatic/spherical binary save/load broken-state reset
  coverage plus same-multibody/world-link articulated fixed/revolute/prismatic/
  spherical binary broken-state round-trips preserving one-DOF command/effort-limit
  motor state, dartpy same-multibody/world-link fixed/spherical and one-DOF
  design-mode rebuild checks, and
  free-rigid/articulated design-mode AVBD point-joint stiffness facade binary
  persistence plus direct C++/dartpy validation of public articulated stiffness
  defaults, finite setters, invalid setter rejection, and C++/dartpy
  endpoint-ownership rejection plus the
  `avbd_rigid_breakable_joint` py-demo with reset/re-engagement coverage
  and tracked packet evidence
  plus `avbd_rigid_spherical_breakable_joint` for spherical anchor-only
  reset/re-engagement with tracked packet evidence; both free-rigid breakable
  demo regressions verify weak re-arm breaks again after high-force reset.
  Public articulated fixed
  point-joints now also expose the narrow world-link break/reset lifecycle
  through the categorized `avbd_articulated_breakable_joint` py-demo with
  tracked packet evidence and the
  same-multibody break/reset lifecycle through
  `avbd_articulated_fixed_pair_breakable_joint` with tracked packet evidence,
  both with weak re-arm breakage coverage after reset,
  while public articulated
  spherical point-joints expose the matching linear-only
  anchor break/reset path through
  `avbd_articulated_spherical_breakable_joint` for world-link endpoints and
  `avbd_articulated_spherical_pair_breakable_joint` for same-multibody
  endpoints, both with tracked packet evidence and weak re-arm breakage
  coverage after reset. A narrow
  `avbd_articulated_high_ratio_chain` py-demo,
  `BM_AvbdArticulatedHighRatioChainStep` dashboard row, and
  [`avbd-articulated-high-ratio-chain-packet.json`](104-vertex-block-descent-solver/avbd-articulated-high-ratio-chain-packet.json)
  now also cover a five-link variational-chain smoke with a 200:1 heavy tip,
  and `avbd_paper_scale_high_ratio_chain`,
  `PaperScaleHighRatioChainStaysFiniteAndResets`, plus
  `BM_AvbdPaperScaleHighRatioChainStep` cover a 50-link/50,000:1 finite/reset
  and visual/CPU benchmark smoke through configured `World::step()` solve-budget
  fields with
  [`avbd-paper-scale-high-ratio-chain-packet.json`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-chain-packet.json)
  visual/benchmark evidence, while
  `BM_AvbdPaperScaleHighRatioChainIterationSweep` adds dashboard-selected
  25/50/100/200 max-iteration sweep coverage for that same paper-scale fixture
  with finite replay counters, tracked benchmark/stability evidence in
  [`avbd-paper-scale-high-ratio-iteration-sweep-packet.json`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-packet.json),
  and a rendered
  [`avbd-paper-scale-high-ratio-iteration-sweep-plot.svg`](104-vertex-block-descent-solver/avbd-paper-scale-high-ratio-iteration-sweep-plot.svg).
  The same-hardware comparison and GPU gates remain open. Solver-identity
  relabel (PLAN-091 WP-091.1): no
  `avbd-demo2d`/`avbd-demo3d` benchmark or py-demo scene emplaces the internal
  AVBD rigid-contact opt-in config (`comps::RigidAvbdContactConfig`), because
  AVBD contact is not facade-selectable, so every rigid contact in the
  source-row scenes below ran DART's default sequential-impulse contact path.
  The native-runner timing ratios for contact scenes are whole-pipeline
  `World::step` comparisons, not AVBD-contact-solver comparisons: the
  pure-contact rows (2D Dynamic Friction, Static Friction, Pyramid, Cards,
  Stack, and Stack Ratio; 3D Ground, Dynamic Friction, Static Friction,
  Pyramid, Stack, and Stack Ratio) timed no AVBD rows at all; the
  joint-plus-contact rows (2D Fracture, Soft Body, Joint Grid, and Net; 3D
  Soft Body, Bridge, and Breakable) timed AVBD point-joint/motor/spring rows
  while their ordinary contacts ran sequential impulse; and incidental
  link-link contacts in the chain rows (2D Rod, Rope, Heavy Rope, and Hanging
  Rope; 3D Rope and Heavy Rope) also ran sequential impulse. This relabel
  changes no committed packet bytes and neither closes nor reopens any
  PLAN-104 completion gate; new AVBD evidence packets must machine-record
  `resolved_solver_identity` at AVBD packet schema version 2, enforced by
  `pixi run check-avbd-packets`. Public
  empty-scene corpus baseline coverage is now visible through
  `avbd_empty_baseline`, a focused Python smoke that checks source revisions,
  default source parameters, and the `sceneEmpty` zero-count invariant, and
  `BM_AvbdEmptyWorldStep`; the tracked
  `avbd-empty-baseline-packet.json` records the corresponding headless
  visual-capture hashes and Google Benchmark row. The first non-empty
  2D source row is now visible through `avbd_demo2d_ground`, which matches the
  `avbd-demo2d` Ground source revision, scene index, one static slab, one rigid
  body, one collision shape, no joints, and no dynamic bodies, plus
  `BM_AvbdDemo2dGroundStep`. The tracked
  `avbd-demo2d-ground-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing; after skipping static-only contact queries,
  no-op rigid dynamics stages, clean frame-cache graph execution, and the clean
  no-work default step pipeline with a cheap scratch reset, it records DART
  about 1.51x faster than the native static Ground runner on this host, closing
  that narrow CPU-win gate. The
  first one-DOF motor source-demo row is now visible through
  `avbd_demo2d_motor`, which matches the
  `avbd-demo2d` Motor scene's source revision, scene index, default parameters,
  20 rad/s target speed, and 50 N m effort bound, plus
  `BM_AvbdDemo2dMotorStep`. The tracked
  `avbd-demo2d-motor-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing. After the rigid contact stage skips contact
  queries for worlds with no collision geometry in prepare/execute, the packet
  still records the DART public World row about 6.18x slower than the native
  source runner on this host, so the CPU-win gate remains open. The next
  non-empty
  `avbd-demo2d` source row is now visible through
  `avbd_demo2d_hanging_rope`, which matches the `avbd-demo2d` Hanging Rope
  source revision, scene index, 49 regular links, one 10 m endpoint block, 49
  linear-only point joints, and 50 collision shapes, plus
  `BM_AvbdDemo2dHangingRopeStep`. The tracked
  `avbd-demo2d-hanging-rope-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 7.84x slower
  than the native Hanging Rope runner on this host, keeping that CPU-win gate
  open. The first radial distance-spring source harnesses are now visible
  through `avbd_demo2d_spring` and `avbd_demo2d_spring_ratio`, which match the
  `avbd-demo2d` Spring and Spring Ratio source rows over public free-rigid
  distance springs, plus `BM_AvbdDemo2dSpringStep` and
  `BM_AvbdDemo2dSpringRatioStep`. Their tracked
  visual/DART-benchmark/native-timing packets now record the spring-connected
  ignored-pair counts and DART about 4.02x slower than the native Spring runner
  and about 4.48x slower than the native Spring Ratio runner on this host, so
  CPU performance resolution and GPU parity remain open. The next
  `avbd-demo2d` source row is now visible through
  `avbd_demo2d_fracture`, which matches the `avbd-demo2d` Fracture source
  revision, scene index, 11 chain links, two dynamic supports, 15 falling
  blocks, 10 breakable fixed joints, and 29 collision shapes, plus
  `BM_AvbdDemo2dFractureStep`. Focused integration coverage now also verifies
  that the source-row fixed joints fracture, reset at a high break force, stay
  unbroken, and reduce their anchor residuals again. The tracked
  `avbd-demo2d-fracture-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; after a refreshed same-source
  timing run, it records DART about 1.20x faster than the native Fracture
  runner on this host, closing only that narrow source-row CPU gate. Later
  local cleanup replaces the contact stage's duplicate prepare-time collision
  query with collision-shape-count constraint prewarm and aligns live
  constrained-pair filtering with the native solver, and that refreshed packet
  now captures the comparison.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_dynamic_friction`, which matches the `avbd-demo2d` Dynamic
  Friction source revision, scene index, 11 sliding boxes, source friction
  coefficients from 5.0 down to 0.0, a static ground, and 12 collision shapes,
  plus `BM_AvbdDemo2dDynamicFrictionStep`. The tracked
  `avbd-demo2d-dynamic-friction-packet.json` adds headless visual capture,
  DART benchmark JSON, and native source timing; it records DART about 1.83x
  faster than the native Dynamic Friction runner on this host, closing that
  narrow CPU-win gate while leaving broad friction scenes and GPU packets open.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_static_friction`, which matches the `avbd-demo2d` Static
  Friction source revision, scene index, one rotated static ground slab, 11
  rotated dynamic boxes, uniform source friction 1.0, and 12 collision shapes,
  plus `BM_AvbdDemo2dStaticFrictionStep`. The tracked
  `avbd-demo2d-static-friction-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 2.68x faster
  than the native Static Friction runner on this host, closing that narrow
  CPU-win gate while leaving broad friction scenes and GPU packets open.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_pyramid`, which matches the `avbd-demo2d` Pyramid source
  revision, scene index, a static ground, 210 dynamic boxes in the source
  pyramid layout, and 211 collision shapes, plus
  `BM_AvbdDemo2dPyramidStep`. The tracked
  `avbd-demo2d-pyramid-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing; it records DART about 9.84x faster than the
  10,000-step native Pyramid runner on this host, closing that narrow row only.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_stack`, which matches the `avbd-demo2d` Stack source revision,
  scene index, 20 vertical dynamic boxes over static ground, and 21 collision
  shapes, plus `BM_AvbdDemo2dStackStep`. The tracked
  `avbd-demo2d-stack-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing; it records DART about 2.17x faster than the
  native Stack runner on this host, closing that narrow row only.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_stack_ratio`, which matches the `avbd-demo2d` Stack Ratio
  source revision, scene index, six geometric-size dynamic boxes over static
  ground, and 7 collision shapes, plus `BM_AvbdDemo2dStackRatioStep`. The
  tracked `avbd-demo2d-stack-ratio-packet.json` adds headless visual capture,
  DART benchmark JSON, and native source timing; it records DART about 2.22x
  faster than the native Stack Ratio runner on this host, closing that narrow
  row only.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_rod`, which matches the `avbd-demo2d` Rod source revision,
  scene index, 20 rigid links, one static anchor link, 19 all-axis public fixed
  joints, and 20 collision shapes, plus `BM_AvbdDemo2dRodStep`. The tracked
  `avbd-demo2d-rod-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing; it records DART about 9.58x slower than the
  native Rod runner on this host, so that CPU-win gate remains open.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_soft_body`, which matches the `avbd-demo2d` Soft Body source
  revision, scene index, one static ground slab, two 15x5 dynamic rigid-box
  lattices, 260 finite-stiffness all-axis public fixed joints, 224 diagonal
  ignored collision pairs, and 151 collision shapes, plus
  `BM_AvbdDemo2dSoftBodyStep`. The tracked
  `avbd-demo2d-soft-body-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 6.30x slower
  than the native Soft Body runner on this host, so that CPU-win gate remains
  open. Stable same-order AVBD row inventories now warm-start in place and row
  ordinal counters now use reserved endpoint-pair hash maps. The rigid row
  append paths seed fallback snapshot body-index cache entries, and the rigid
  row driver also skips per-body row-index scratch for row families absent in a
  solve, keeps unchanged source-row index layouts warm across frames, and
  routes single-family point-pair/angular solves without rebuilding combined row
  vectors, while one-new-row point-joint/distance-spring appends skip endpoint
  row-counter hash-map setup. Friction tangent-pair rows now reuse precomputed
  world anchors across force and direction stamping in one pair evaluation. The
  contact-stage AVBD path now reuses a
  scratch contact snapshot through an in-place builder, skips pair-constraint
  extraction when no point-joint/distance-spring configs exist, and
  extracts/appends point-joint and distance-spring families independently when
  only one family exists. The rigid snapshot solve now also clears absent
  row-family inventories directly instead of calling empty
  contact/joint/motor/spring builders, small point-joint/motor/distance-spring
  row builders use stack descriptor/active-row storage for up to 16 candidate
  rows instead of allocating temporary vectors, and the split rigid-body velocity
  stage assembles force batches only for advanceable bodies, but the packet
  CPU-win gate stays open until refreshed same-command evidence beats the native
  runner.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_joint_grid`, which matches the `avbd-demo2d` Joint Grid source
  revision, scene index, 625 rigid boxes, two static top-corner anchors, 1200
  all-axis public fixed joints, and 625 collision shapes, plus
  `BM_AvbdDemo2dJointGridStep`. The tracked
  `avbd-demo2d-joint-grid-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 2.06x slower
  than the native Joint Grid runner on this host after reusing per-body
  row-index scratch, caching snapshot body indices, reusing row-assembly
  world-anchor computations through force/Hessian/Jacobian stamping, and
  caching all-axis angular-row orientation errors, with the source diagonal
  ignore-collision filter configured through DART's public per-pair API.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_cards`, which matches the `avbd-demo2d` Cards source revision,
  scene index, one static ground slab, 40 thin dynamic cards across five
  levels, and 41 collision shapes, plus `BM_AvbdDemo2dCardsStep`. The tracked
  `avbd-demo2d-cards-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing; it records DART about 5.38x slower than the
  native Cards runner on this host. Later friction tangent-pair world-anchor
  reuse, row-state constraint-vector reuse, and shared-anchor reuse removed
  local contact-heavy duplicate work but have not refreshed this packet,
  keeping that CPU-win gate open.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_rope`, which matches the `avbd-demo2d` Rope source revision,
  scene index, 20 rigid links, 19 linear-only public spherical point joints,
  and 20 collision shapes, plus `BM_AvbdDemo2dRopeStep`. The tracked
  `avbd-demo2d-rope-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing; it records DART about 5.20x slower than the
  native Rope runner on this host, so that CPU-win gate remains open.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_heavy_rope`, which matches the `avbd-demo2d` Heavy Rope source
  revision, scene index, 19 regular links, a 30 m endpoint block, 19
  linear-only public spherical point joints, and 20 collision shapes, plus
  `BM_AvbdDemo2dHeavyRopeStep`. The tracked
  `avbd-demo2d-heavy-rope-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 5.85x slower
  than the native Heavy Rope runner on this host after reusing per-body
  row-index scratch, caching snapshot body indices, avoiding redundant
  row-assembly world-anchor transforms, reusing precomputed distance-spring
  Hessian/Jacobian world points, skipping capped finite-row constraint
  recomputation, and caching grouped angular-row orientation errors, so
  that CPU-win gate remains open.
  The next `avbd-demo2d` source row is now visible through
  `avbd_demo2d_net`, which matches the `avbd-demo2d` Net source revision, scene
  index, one static ground slab, 40 endpoint-pinned net links, 50 falling rigid
  blocks, 39 linear-only public spherical point joints, and 91 collision shapes,
  plus `BM_AvbdDemo2dNetStep`. The tracked
  `avbd-demo2d-net-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing; it records DART about 1.23x slower than the
  native Net runner on this host after caching snapshot body indices, reusing
  row-assembly world-anchor computations through force/Hessian/Jacobian
  stamping, reusing friction tangent-pair world anchors, reusing row-state
  tangent constraint values, reusing shared tangent-row anchors, skipping capped
  finite-row constraint recomputation, and caching grouped angular-row
  orientation errors, with local-anchor point-joint extraction now using a
  stable full-rank linear basis for spherical point joints, so that CPU-win gate
  remains open.
  The next 3D source row is now visible through `avbd_demo3d_ground`, which
  matches the
  `avbd-demo3d` Ground source revision, scene index, a static floor, falling
  rigid box, 2 rigid bodies, and 2 collision shapes, plus
  `BM_AvbdDemo3dGroundStep`. The tracked `avbd-demo3d-ground-packet.json` adds
  headless visual capture, DART benchmark JSON, and native source timing; it
  records DART about 1.11x faster than the native Ground runner on this host,
  closing that narrow row only. The next non-empty source row after that is
  visible through `avbd_demo3d_dynamic_friction`, which matches the
  `avbd-demo3d` Dynamic Friction source revision, scene index, 11 sliding rigid
  boxes, a static floor, and 12 collision shapes, plus
  `BM_AvbdDemo3dDynamicFrictionStep`. The tracked
  `avbd-demo3d-dynamic-friction-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 1.41x faster
  than the native Dynamic Friction runner on this host, closing that narrow row
  only. The next non-empty source row after that is visible through
  `avbd_demo3d_static_friction`, which matches the
  `avbd-demo3d` Static Friction source revision, scene index, a static floor,
  inclined static ramp, 11 sliding rigid boxes, and 13 collision shapes, plus
  `BM_AvbdDemo3dStaticFrictionStep`. The tracked
  `avbd-demo3d-static-friction-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 1.08x faster
  than the native Static Friction runner on this host, closing that narrow
  CPU-win gate. The next non-empty source row after that is visible through
  `avbd_demo3d_pyramid`, which matches the `avbd-demo3d` Pyramid source
  revision, scene index, a static ground, 136 dynamic boxes in the triangular
  pile layout, and 137 collision shapes, plus
  `BM_AvbdDemo3dPyramidStep`. The tracked `avbd-demo3d-pyramid-packet.json`
  adds headless visual capture, DART benchmark JSON, and native source timing;
  it records DART about 2.83x faster than the native Pyramid runner on this
  host, closing that narrow row only. The next non-empty source row after that
  is visible through `avbd_demo3d_rope`, which matches the `avbd-demo3d` Rope
  source revision, scene index, 20 rigid links, 19 anchored linear-only point
  joints, and 21 collision shapes, plus `BM_AvbdDemo3dRopeStep`. The tracked
  `avbd-demo3d-rope-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing; it records DART about 3.75x slower than the
  native Rope runner on this host, so that CPU-win gate remains open. The next
  non-empty source row after that is visible through
  `avbd_demo3d_heavy_rope`, which matches the `avbd-demo3d` Heavy Rope source
  revision, scene index, 19 regular links, a 5 m endpoint block, 19 anchored
  linear-only point joints, and 21 collision shapes, plus
  `BM_AvbdDemo3dHeavyRopeStep`. The tracked
  `avbd-demo3d-heavy-rope-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 3.84x slower
  than the native Heavy Rope runner on this host, so that CPU-win gate remains
  open. The next non-empty source row after that is visible through
  `avbd_demo3d_stack`, which matches the `avbd-demo3d` Stack
  source revision, scene index, 10 vertical dynamic boxes over static ground,
  and 11 collision shapes, plus `BM_AvbdDemo3dStackStep`. The tracked
  `avbd-demo3d-stack-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing; it records DART about 1.80x faster than the
  native Stack runner on this host, closing that narrow row only. The next
  non-empty source row after that is visible through
  `avbd_demo3d_stack_ratio`, which matches the `avbd-demo3d` Stack Ratio source
  revision, scene index, four geometric-size dynamic boxes over static ground,
  and 5 collision shapes, plus `BM_AvbdDemo3dStackRatioStep`. The tracked
  `avbd-demo3d-stack-ratio-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 2.32x faster
  than the native Stack Ratio runner on this host, closing that narrow row only.
  The next non-empty source row after that is visible through
  `avbd_demo3d_soft_body`, which matches the `avbd-demo3d` Soft Body source
  revision, scene index, three 4x4x4 dynamic rigid-box lattices, 432
  finite-stiffness all-axis fixed joints, 648 diagonal ignored collision pairs,
  and 193 collision shapes, plus `BM_AvbdDemo3dSoftBodyStep`. The tracked
  `avbd-demo3d-soft-body-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 1.21x faster
  than the native Soft Body runner on this host, closing that narrow CPU row
  only. Stable same-order AVBD row inventories now warm-start in place and row
  ordinal counters now use reserved endpoint-pair hash maps. The rigid row
  append paths seed fallback snapshot body-index cache entries, and the rigid
  row driver also skips per-body row-index scratch for row families absent in a
  solve, keeps unchanged source-row index layouts warm across frames, and
  routes single-family point-pair/angular solves without rebuilding combined
  row vectors, while one-new-row point-joint/distance-spring appends skip
  endpoint row-counter hash-map setup. The contact-stage AVBD path now reuses a
  scratch contact snapshot through an in-place builder, skips pair-constraint
  extraction when no point-joint/distance-spring configs exist, and
  extracts/appends point-joint and distance-spring families independently when
  only one family exists. The rigid snapshot solve now also clears absent
  row-family inventories directly instead of calling empty
  contact/joint/motor/spring builders, small point-joint/motor/distance-spring
  row builders use stack descriptor/active-row storage for up to 16 candidate
  rows instead of allocating temporary vectors, and the split rigid-body velocity
  stage assembles force batches only for advanceable bodies, but the packet
  CPU-win gate stays open until refreshed same-command evidence beats the native
  runner. The next non-empty source row after that is visible through
  `avbd_demo3d_bridge`, which matches the `avbd-demo3d` Bridge source revision,
  scene index, 40 planks, 50 load boxes, 78 paired linear-only point joints,
  and 91 collision shapes, plus `BM_AvbdDemo3dBridgeStep`. The tracked
  `avbd-demo3d-bridge-packet.json` adds headless visual capture, DART benchmark
  JSON, and native source timing; it records DART about 1.61x faster than the
  native Bridge runner on this host, closing that narrow row only. The next
  non-empty source row after that is visible through
  `avbd_demo3d_breakable`, which matches the
  `avbd-demo3d` Breakable source revision, scene index, 19 rigid bodies, 10
  breakable fixed joints, and 19 collision shapes, plus
  `BM_AvbdDemo3dBreakableStep`. Focused integration coverage now also verifies
  that the source-row fixed joints fracture, reset at a high break force, stay
  unbroken, and reduce their anchor residuals again. The tracked
  `avbd-demo3d-breakable-packet.json` adds headless visual capture, DART
  benchmark JSON, and native source timing; it records DART about 1.42x faster
  than the native Breakable runner on this host. The next `avbd-demo3d` source
  rows are now visible through `avbd_demo3d_spring` and
  `avbd_demo3d_spring_ratio`, which match the `avbd-demo3d` Spring and Spring
  Ratio source rows over public free-rigid distance springs, plus
  `BM_AvbdDemo3dSpringStep` and
  `BM_AvbdDemo3dSpringRatioStep`. Their tracked
  visual/DART-benchmark/native-timing packets now record the spring-connected
  ignored-pair counts and DART about 2.47x slower than the native Spring runner
  and about 3.57x slower than the native Spring Ratio runner on this host, so
  CPU performance resolution and GPU parity remain open. Other missing corpus
  items include Spring and Spring Ratio GPU
  gates, the 2D Motor, Hanging Rope, 2D Cards, 2D Rod, 2D Joint Grid,
  2D Rope, 2D Heavy Rope, 2D Net, 3D Rope, and 3D Heavy Rope CPU gaps, remaining
  CPU reference wins, broad 2D stack variants, and GPU packets remain missing.
  multibody/articulated AVBD state is only narrowly wired: the private endpoint
  layer distinguishes free
  rigid-body endpoints from multibody links, fixed multibody-link point-joint
  configs can bridge into the variational articulated solve path, the branch
  has a `BM_AvbdRigidEndpointClassification` benchmark row plus focused C++
  bridge tests, and public same-multibody/world-link articulated
  fixed/revolute/prismatic/spherical facades now feed the current-pose extractor
  through C++/dartpy with focused spherical linear-only pinned-anchor behavior
  including explicit link-link/world-link anchors,
  bounded revolute/prismatic velocity-actuator,
  same-multibody/world-anchored tiny effort-limit and command-update coverage,
  private revolute/prismatic command-update, fixed-row and revolute/prismatic
  break/reset re-engagement,
  same-multibody/world-anchored public one-DOF motor break/skip and
  reset/re-engagement,
  selected direct same-multibody/world-link one-DOF break/skip/reset
  non-cardinal basis checks,
  including focused dartpy stepping coverage for same-multibody/world-link
  revolute and prismatic explicit-anchor cases,
  same-multibody/world-anchored public one-DOF motor break/reset
  re-engagement, movable-movable same-multibody revolute/prismatic motor
  break/reset with explicit off-origin anchors covered in C++/dartpy,
  direct/private movable-pair fixed/revolute/prismatic reset plus
  direct/private and current-pose spherical linear-row reset, current-pose
  movable-pair fixed/revolute/prismatic reset, and non-cardinal motor-axis
  coverage, public
  same-multibody/world-anchored articulated revolute/prismatic
  floating-endpoint plus selected off-origin-anchor facade non-cardinal
  motor-axis coverage,
  current-pose
  fixed/prismatic reset regressions, private current-pose revolute/prismatic
  tiny-limit movable-pair coverage, public same-multibody/world-link one-DOF
  non-cardinal finite-limit coverage, public same-multibody movable-pair
  revolute/prismatic non-cardinal motor-axis and finite-limit coverage, public
  same-multibody movable-pair revolute/prismatic broken-state save/load/reset
  coverage, private generated current-pose movable-pair fixed and
  revolute/prismatic broken-state save/load/reset coverage, private generated current-pose
  movable-pair spherical broken-state save/load/reset coverage, direct private
  world-link one-DOF broken-state save/load/reset coverage preserving private
  `AvbdRigidWorldPointJointConfig` basis/mask/anchor state, private
  current-pose spherical reset regression for linear-only rows, and
  fixed break/save/load/reset with dartpy stepping coverage, explicit local/world anchor projection including
  same-multibody and world-anchored off-origin fixed/revolute/prismatic anchors and
  revolute/prismatic motors,
  world-fixed break/reset, same-multibody fixed break/reset py-demo coverage,
  same-multibody/world-link spherical linear-row break/skip and reset including
  focused dartpy stepping coverage and categorized same-multibody/world-link
  py-demos, same-multibody link-link and world-link fixed/spherical save/load
  rebuilding of the private all-axis and linear rows, including selected dartpy
  same-multibody/world-link fixed/spherical and one-DOF design-mode rebuild
  checks, same-multibody/world-link
  revolute/prismatic motor save/load rebuilding of the private hard rows and
  free-axis motor row with selected non-cardinal axis-basis persistence,
  same-multibody/world-link
  fixed/spherical/revolute/prismatic broken-state save/load/reset persistence
  including movable-movable fixed coverage and selected non-cardinal one-DOF
  motor rows with focused C++ binary round-trips preserving one-DOF
  command/effort-limit motor state plus dartpy same-multibody/world-link
  fixed/spherical and one-DOF binary round-trips and direct
  break/skip/reset non-cardinal basis checks,
  public free-rigid/articulated AVBD point-joint
  stiffness facade binary persistence plus direct C++/dartpy validation of
  articulated stiffness defaults, finite setters, invalid setter rejection, and
  C++/dartpy endpoint-ownership rejection for same-link/cross-multibody/cross-world
  articulated point-joint requests,
  a narrow five-link 200:1 high mass-ratio
  articulated-chain smoke py-demo plus dashboard row, a focused 50-link/50,000:1
  finite/reset stability smoke and matching
  `BM_AvbdPaperScaleHighRatioChainStep` dashboard row plus benchmark packet
  through configured `World::step()` solve-budget fields, the new
  `BM_AvbdPaperScaleHighRatioChainIterationSweep` dashboard-selected
  25/50/100/200 max-iteration sweep row plus benchmark/stability packet and
  rendered SVG plot,
  world-anchor coverage, and a
  per-multibody link-index cache in the
  articulated point-joint extractor so same-multibody/world-link AVBD private
  rows no longer rescan structure membership for every point-joint config.
  Rigid AVBD point-joint and distance-spring extraction now also share a
  projectable-body metadata lookup, avoiding a second transform lookup after
  endpoint classification in the pair-constraint hot path and reusing the
  already-checked projectable transform, mass, and static tag when contact
  snapshots materialize body state. The same metadata path also skips
  static-static rigid point-joint and distance-spring pairs before input or row
  construction.
  The
  related `variational_endpoint_loop_closure` py-demo previews public
  loop-closure behavior but does not exercise the private AVBD config extractor.
  The next local slice should broaden persistent articulated motor/fracture
  lifecycle coverage beyond the current revolute/prismatic command-update,
  movable link-pair motor/fixed reset projection, direct/private and
  current-pose movable-pair fixed/revolute/prismatic break/reset coverage,
  current-pose/public same-multibody and world-anchored
  floating-endpoint plus selected off-origin-anchor facade non-cardinal axis and
  selected save/load and broken-state save/load/reset non-cardinal axis-basis
  persistence plus selected direct break/skip/reset non-cardinal axis-basis
  checks plus
  same-multibody/world-link one-DOF and public movable-pair finite-limit plus
  broken-state save/load/reset coverage with non-cardinal axis-basis checks,
  private generated current-pose movable-pair fixed, one-DOF, and spherical
  broken-state save/load/reset coverage, with narrow breakable benchmark rows,
  world-fixed reset, non-cardinal dartpy same-multibody/world-link one-DOF
  reset endpoint/axis-shape checks, and dartpy fixed/spherical reset
  endpoint-shape checks now treated as covered current evidence,
  matched-metadata empty-scene baseline smoke, the 2D Fracture/3D Breakable
  source-demo fixed-joint break/reset coverage, the first non-empty
  `avbd-demo2d` Ground, Motor, Hanging Rope, Fracture, Dynamic Friction, Static
  Friction, Pyramid, Cards, Stack, Stack Ratio, Rod, Joint Grid, Rope, Heavy
  Rope, and Net source-row packets, the Spring and Spring Ratio source packets
  plus open CPU/GPU gates, the
  `avbd-demo3d`
  Ground source-row
  py-demo, benchmark row, and packet, the `avbd-demo3d` Dynamic Friction
  source-row py-demo, benchmark row, and packet, the `avbd-demo3d` Static
  Friction source-row py-demo, benchmark row, and packet, the `avbd-demo3d`
  Pyramid source-row py-demo, benchmark row, and packet, the `avbd-demo3d`
  Rope, Heavy Rope, Spring, and Spring Ratio source-row py-demos, benchmark
  rows, and packets, the `avbd-demo3d` Stack, Stack Ratio, and Soft Body source-row py-demos,
  benchmark rows, and packets, the `avbd-demo3d` Breakable source-row py-demo,
  benchmark row, and packet, and one-DOF
  break/reset checks,
  expand
  articulated facade coverage beyond the new link-link,
  world-link, explicit-anchor, spherical linear-only point-joint entrypoints,
  same-multibody link-link/world-link fixed/spherical save/load rebuilding
  including dartpy same-multibody/world-link fixed/spherical and one-DOF
  design-mode rebuild checks, and
  same-multibody/world-link revolute/prismatic motor save/load rebuilding
  including selected non-cardinal axis-basis persistence and restored
  same-multibody/world-link endpoint-shape assertions, and
  same-multibody/world-link fixed/spherical/revolute/prismatic broken-state
  save/load/reset persistence including explicit-anchor fixed and selected
  non-cardinal one-DOF motor rows with restored effort-limit state, with the
  dartpy same-multibody/world-link fixed/spherical and one-DOF binary
  round-trips, direct break/skip/reset non-cardinal basis checks, fixed and
  spherical reset endpoint-shape assertions, and explicit-anchor one-DOF motor
  reset endpoint/axis-shape assertions now treated as current evidence,
  broaden rigid-contact feature persistence beyond the current
  box/sphere/cylinder/capsule/plane/mesh known/unknown shape-frame feature
  identity tests, endpoint-A/B explicit-shape local-point evidence, actual
  narrow-phase primitive-feature evidence, same-feature sphere/plane replay plus
  sphere/mesh-face, sphere/mesh-edge, mesh-vertex replay, and
  mesh-face/mesh-edge/mesh-vertex small-pose persistence, cylinder/capsule
  cap/side/rim small-pose persistence, live box-box row-order evidence,
  endpoint-order row-identity evidence, spanning-top and multi-top box-pile
  row-persistence evidence, plus
  private rigid tangent-dual projection, or
  implement the first missing row from the new
  [`avbd-demo-corpus.md`](104-vertex-block-descent-solver/avbd-demo-corpus.md)
  matrix, while the plan/dev-task surfaces must also keep the 2D/3D source-demo
  corpus, paper/video scenes, CPU/GPU benchmark packets, and performance
  leadership gates explicit instead of treating the current free-rigid rows as
  AVBD completion.
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
  [`../dev_tasks/variational_integrator_solver/graduation-criteria.md`](../dev_tasks/variational_integrator_solver/graduation-criteria.md)
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

### PLAN-092: Performance Dashboard

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
  The tracked slice now covers the new DART 7 solver families' end-to-end
  `World::step` surfaces (rigid-body sequential-impulse/IPC, VBD + default
  deformable grid, FEM bar, AVBD fixed-joint, motor, and breakable-joint rows)
  alongside the original core step/scaling rows, and
  `scripts/benchmark_display_names.py` rewrites the raw Google Benchmark names
  into readable chart titles (merge `--humanize`) with family-grouped,
  axis-labelled local preview. Add new surfaces by extending both
  `run_performance_dashboard_benchmarks.py` (CPU-only, end-to-end filter) and
  the `SURFACES` map.
- Gate: `pixi run bm-dashboard-preview` renders the dashboard locally from real
  Google Benchmark JSON; each `main` publish updates the hosted per-benchmark
  history.

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
- Next step: The promoted-header allowlist, strict promotion-surface audit,
  package-contract check, installed-package smoke, World-promotion blocker
  inventory, C++ namespace/target transaction, Python import-layout transaction,
  and source-tree move are now in place: `dart::simulation::World`,
  `dartpy.simulation.World`, and `dartpy.World` point at the ECS-backed facade,
  generated stubs no longer publish `dartpy.simulation_experimental`, and the
  classic Python world is quarantined as `dartpy.gui.RenderWorld`. The urgent
  remaining promotion work is hardening and cleanup: finish stale docs, keep
  the public-header/package/import guards green, add negative smokes for retired
  experimental paths and targets, and keep strict-final World-promotion checks
  green with parity evidence sourced from `release-6.*` branch refs.
  The promoted `World` remains double-backed; public scalar precision selectors
  (`sim.World(dtype=...)`, `sim.World[...]`, scalar-specific aliases, or a public
  C++ scalar-template facade) stay deferred until a separate
  scalar-instantiation plan proves the required ownership, binding,
  serialization, collision, differentiability, package, and migration gates.
  The intended path is DART 7 official API promotion, not a DART 8 middle step.
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

### PLAN-050: DART 7 World Binding Transition

- Owner doc:
  [`../onboarding/python-bindings.md#dart-7-world-bindings-and-transition`](../onboarding/python-bindings.md#dart-7-world-bindings-and-transition)
- Status: Complete
- Horizon: Later
- Dimension: Algorithm extensibility
- Next step: Keep this item complete. Current DART 7 promotion work is tracked
  by PLAN-040, PLAN-041, and PLAN-042; the ECS World is promoted to
  `dart::simulation`, `dartpy.simulation`, and `dartpy.World`.
- Gate: The old split is retired: generated stubs no longer
  publish `dartpy.simulation_experimental`, `check-dartpy-import-layout` guards
  the promoted import shape, and any remaining `experimental` references are
  compatibility or transition checks tracked by PLAN-041 rather than PLAN-050.

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
  depth/segmentation outputs). For material/lighting/texture fidelity and for
  promoting debug visuals into the GUI component (per-shape PBR, IBL reflections,
  texture mipmaps, an app `debugProvider`, richer debug primitives), see
  [`../design/renderer_fidelity_and_debug_visuals.md`](../design/renderer_fidelity_and_debug_visuals.md).
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

### PLAN-102: Demos App

- Owner doc: [`../design/demos_app.md`](../design/demos_app.md); user
  instructions in [`../../examples/README.md`](../../examples/README.md).
- Status: Complete
- Horizon: Now
- Dimension: Easy start
- Next step: Implemented — `dart-demos` is now the smaller C++ World demo app
  with `rigid_body`, `deformable_body`, and `vbd_deformable` runtime-switchable
  scenes plus lightweight `Planned World Ports` placeholders for high-value DART
  6 concepts that still need World-native ports (IK, SIMBICON walking,
  operational-space control, robot puppets, mobile manipulation); the retired
  collision sandbox route now points to concrete Python collision-debugging rows
  instead of a C++ placeholder. The
  `dart/gui/detail` `ExampleScene` set is intentionally kept as the renderer's
  internal test fixtures (see the design doc's "examples vs renderer fixtures"
  decision), so PLAN-101's `--scene` smoke gate is unaffected.
- Gate: `dart-demos` launches and switches across the World scenes in one window
  without window recreation; the headless cycle smoke renders every registered
  scene; only CLI/headless support programs stay standalone;
  `pixi run lint` and `check-docs-policy` green.

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
- Scope note: WS1–WS5 first slices plus the PLAN-080-WS4 boxed-LCP contact
  prerequisite are implemented and verified (FD-of-step validated; default build
  untouched and bitwise-identical when off). That path landed via PR #2761
  (merged 2026-05-30, milestone DART 7.0, CHANGELOG included; the
  `feature/differentiable-simulation` branch was deleted post-merge). Remaining
  for that path is hardening/examples/promotion, not new workstreams. Dojo is
  now tracked as a separate planned evaluation
  track under PLAN-110: maximal-coordinate variational hard-contact NCP/IPM with
  implicit gradients, using Dojo.jl as method evidence and a comparison baseline
  rather than a dependency.
- Next step: The WS1–WS5 differentiability surface is merged to `main` (PR #2761);
  the remaining work is the hardening/examples/Dojo-spike follow-ups. Run the
  torch-autograd test (`pip install torch`); add the
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
