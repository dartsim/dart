# Experimental World Scalable Compute — Dev Task

Multiphase plan to add multi-core CPU and (later) GPU support to the experimental
simulation World. Detailed phases live in [`01-plan.md`](01-plan.md). Durable
rationale and the option survey live in
[`../../design/compute_backend_research.md`](../../design/compute_backend_research.md)
and [`../../design/scalable_compute_decisions.md`](../../design/scalable_compute_decisions.md).
Operating priority is owned by `docs/plans/dashboard.md` (PLAN-030).

## Current Status

- [x] Phase 0: Foundations — executor-parity test (already present as
      `RigidBodyStepParallelMatchesSequential`, verified green), EnTT concurrency
      contract documented on `ComputeExecutor` + debug-only hazard assertion in
      `ParallelExecutor`, contact-shaped benchmark proxy added to
      `bm_compute_graph.cpp`, resource identity model decided (per-entity stable
      strings). Model/State interface decision is folded into Phase 2.
- [x] Phase 1: Resource-access metadata (diagnostic + hazard validation) —
      `ComputeAccessMode` + `ComputeResourceAccess` + `findResourceHazards()` +
      DOT surfacing + per-entity wiring on the kinematics stage; 30/30
      `test_compute_graph` cases pass. Contract in
      [`../../plans/030-compute-resource-access/`](../../plans/030-compute-resource-access/).
- [x] Phase 2: Immutable Model + batched SoA State. `RigidBodyStateBatch`
      flat-scalar SoA with single- and multi-world (leading dimension) extract/apply;
      immutable `RigidBodyModelBatch` (inverse mass + inertia) realizing the
      Model/State split; scalar-generic SoA kernels (position, velocity, and the
      angle-axis exponential-map orientation kernel) and the linear and full
      (`integrateRigidBodyStateBatch`, plus a torque overload) batched integrators.
      `BatchedRigidBodyIntegrationStage` wires the SoA path into a live
      `WorldStepStage` at full parity with the per-entity integrator (force, torque,
      orientation), including parent-before-child local-transform write-back for
      frame-coupled rigid bodies.
- [x] Phase 3: Multi-core hardening, SIMD, data locality. Landed: O((N+E) log N)
      topological sort; multi-worker (1/2/4/8) determinism parity with a bitwise gate
      for the map-only integration stage (per-body nodes run concurrently);
      `findResourceHazards()` as the unordered-write ambiguity detector; a cost gate
      (`ParallelExecutor::setInlineThreshold`) that runs sub-threshold graphs inline;
      and an explicit-SIMD orientation kernel (`integrateOrientationsSimd`,
      Eigen-vectorized transcendentals) that the batched integrator dispatches to
      above a body-count threshold, with the scalar-generic kernel as the fallback.
      The linear (position/velocity) kernels are memory-bound and already optimally
      auto-vectorized at -O3, so they keep the scalar-generic path. Also landed: a
      deterministic reduction (`totalKineticEnergy`) with fixed-order chunk partials
      and a fixed-ULP/relative tolerance gate, the reduction shape later parallel
      stages reuse. The `pixi run bm-compute-check` gate now checks the full
      expected `bm_compute_graph` compute/world/rigid-body/contact-shaped/
      contact-island-shaped/Phase 5 CPU-baseline corpus for positive finite timings
      and requires the compute-bound contact-island row to beat sequential by real
      time. The performance dashboard publishes both the serial contact-shaped
      hard-case proxy and the independent contact-island speedup surface, plus the
      Phase 5 CPU-baseline history. The trivial Euler rigid-body rows remain
      overhead-bound and must not be used to choose backends.
- [x] Phase 4: Homogeneous batch (CPU) + rollout. `stepWorldsBatched` (parallel
      batch executor via the injected executor, bit-identical to sequential),
      `rolloutWorldsBatched`, and a pure-SoA control-sequence rollout
      (`rolloutRigidBodyStateBatch`). Heterogeneous batches are deferred to Phase 6
      by design.
- [~] Phase 5: GPU prototype behind a gate with a kill criterion (internal, no
  public API). The MVP CUDA path is now opt-in behind
  `DART_ENABLE_EXPERIMENTAL_CUDA`, a Linux-only Pixi `cuda` environment, a
  private build-tree CUDA wrapper over the force-driven `RigidBodyStateBatch`
  SoA path, a parity unit test, a packet-compatible end-to-end smoke benchmark,
  and a manual/non-required GitHub workflow that writes and validates the Phase 5
  packet artifact. Remaining: run that workflow on a project-owned GPU runner
  and use representative workload results for the CUDA-versus-SYCL decision.
- [ ] Phase 6: Reassess — broaden GPU, auto-scheduling, Pattern B, differentiable
      state (each gated; gated on Phase 5 evidence).

## Goal

Make the experimental World scale on multi-core CPUs now and keep a clean,
evidence-gated path to GPU acceleration, by maturing the existing ECS +
compute-graph + injectable executor substrate rather than building a new
framework or touching the classic World.

## Non-Goals (for early phases)

- No new public concurrency API beyond executor injection; no `entt`, `comps`,
  GPU device, stream, kernel, memory-pool, or solver-registry type in the public
  API.
- No automatic scheduling from resource-access metadata until Phase 6; metadata
  stays diagnostic.
- No public GPU API; Phase 5 is an internal prototype and benchmark report.
- No changes to the classic `dart::simulation::World`.
- No differentiable-simulation commitment; it stays a deferred capability.
- No single-scene contact/constraint GPU work (Pattern B) before Pattern A
  evidence justifies it.

## Key Decisions

- Mature the existing substrate; do not redesign. Executor injection is the only
  public seam, so a future GPU backend slots in without changing `World`.
- Pull the state-representation refactor (immutable Model + batched SoA State)
  forward to Phase 2: it is the half that transfers to the GPU, and today's
  array-of-structs access over `entt::registry` does not.
- Enforce determinism with a parity test and a written EnTT concurrency contract
  first; gate determinism by tolerance (bitwise for map-only stages, fixed-ULP
  for reductions) because float reductions are not bitwise-reproducible.
- Co-locate scalar-generic kernels with the SoA rewrite, instantiate `double`
  only; do not template the current immature `double`-Eigen kernels in isolation.
- Benchmark a contact/constraint-shaped workload proxy before any backend
  decision; trivial Euler physics would mislead the CUDA-versus-SYCL choice.
- Keep Taskflow as the CPU backend; defer CUDA-versus-SYCL to the Phase 5
  benchmark; ship any GPU support as an optional/separate package following the
  sidecar shape in `docs/design/scalable_compute_decisions.md`.

## Immediate Next Steps

(Phases 0-2 and Phase 4 are done, and Phase 3's determinism gate and topological
sort are in; see `RESUME.md`.)

1. Preserve the current default `World::step` solver pipeline
   (`RigidBodyVelocityStage` -> `RigidBodyContactStage` ->
   `MultibodyForwardDynamicsStage` -> `RigidBodyPositionStage` ->
   `KinematicsStage`) from the rigid-body dynamics work. Keep
   `BatchedRigidBodyIntegrationStage` as the explicit unconstrained SoA path;
   its frame-coupled rigid-body parity remains covered by the
   simulation-experimental test label.
2. Keep `pixi run bm-compute-check` green as the checked benchmark corpus for
   `bm_compute_graph`, including the dashboard contact-shaped proxy,
   contact-island speedup surface, and Phase 5 CPU-baseline series. Do not use
   trivial Euler integration to choose backends.
3. Phase 5 GPU prototype — blocked on a GPU runner the project does not have;
   treat runner provisioning and a GPU build/import gate as
   maintainer/infrastructure-owned prerequisites. The optional/separate package
   shape, pre-registered go/no-go threshold, and
   `pixi run bm-phase5-gpu-packet-check --write-template <packet.json>` /
   `--input <packet.json>`, the `bm-phase5-cuda-packet` packet writer, plus
   `pixi run check-compute-backend-boundaries` and
   `pixi run check-no-gpu-runtime-dependencies` evidence gates live in
   `docs/design/scalable_compute_decisions.md`. The packet validator requires
   the build/import and policy-gate evidence booleans to be true for the same
   change. `pixi run check-phase5-cuda-benchmark-contract` keeps optional CUDA
   benchmark files packet-compatible, and the `check-phase5-cuda-workflow` task
   keeps the manual workflow wired to the required gates and artifacts. The
   no-GPU dependency gate applies to default/core manifests; explicitly opt-in
   sidecar Pixi features/environments may carry GPU runtime packages.
4. Phase 5 CUDA evidence: run `pixi run -e cuda test-cuda` on a CUDA host, then
   dispatch `.github/workflows/ci_cuda.yml` on a self-hosted runner labeled
   `cuda` once the project has one. The workflow runs the policy gates, the full
   CPU/GPU benchmark row through `bm-phase5-cuda-full`,
   `bm-phase5-cuda-packet`, and
   `bm-phase5-gpu-packet-check`, then uploads both JSON files.
   `workflow_dispatch` workflows must exist on the default branch before
   maintainers can dispatch them against a candidate ref, so this workflow has
   to land (or an equivalent default-branch workflow must exist) before the
   project-owned CUDA artifact can be produced.
   Keep the workflow non-required until runner stability and benchmark signal
   are proven.

## Relationship To Other Surfaces

- This task is the single working tracker for the multi-core/GPU effort. The
  earlier narrow `compute_resource_access` dev task has been consolidated into
  Phase 1 here.
- The PLAN-030 resource-access milestone keeps its objective-specific contract in
  [`../../plans/030-compute-resource-access/`](../../plans/030-compute-resource-access/)
  (mission + evaluator), which is the gate for Phase 1.
- Durable rationale and the option survey live in
  [`../../design/compute_backend_research.md`](../../design/compute_backend_research.md).
