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
- [~] Phase 2: Immutable Model + batched SoA State. Seed landed:
  `RigidBodyStateBatch` flat-scalar SoA value type with
  `extractRigidBodyState`/`applyRigidBodyState` (single world) and a
  round-trip test. Remaining: a leading world dimension > 1, an immutable
  Model split, and scalar-generic (`template <typename Scalar>`, `double`-only)
  kernels.
- [~] Phase 3: Multi-core hardening, SIMD, data locality. Landed early:
  O((N+E) log N) topological sort and a multi-worker (1/2/4/8) determinism
  parity test. Remaining: tolerance-based determinism gate, SIMD on SoA
  kernels, cost gate, and a Bevy-style ambiguity detector once a second
  write-conflicting stage exists.
- [ ] Phase 4: Homogeneous batch (CPU) + rollout API
- [ ] Phase 5: GPU prototype behind a gate with a kill criterion (internal, no
      public API); CUDA-versus-SYCL decided from the benchmark
- [ ] Phase 6: Reassess — broaden GPU, auto-scheduling, Pattern B, differentiable
      state (each gated)

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
  benchmark; ship any GPU support as an optional/separate package.

## Immediate Next Steps

(Phase 0/1 and the early Phase 3 items above are done; see `RESUME.md`.)

1. Extend `RigidBodyStateBatch` to a leading world dimension > 1 (homogeneous
   replication) and add the immutable Model split.
2. Make the integration and kinematics kernels scalar-generic
   (`template <typename Scalar>`, instantiate `double` only) and have them read
   and write the SoA batch instead of per-entity `registry.get`.
3. Add the tolerance-based determinism gate (bitwise for map-only stages,
   fixed-ULP for reductions) and a SIMD pass on the SoA kernels.
4. Wire `bm-check` baselines for the experimental benchmarks so phase gates cite
   committed numbers.

## Relationship To Other Surfaces

- This task is the single working tracker for the multi-core/GPU effort. The
  earlier narrow `compute_resource_access` dev task has been consolidated into
  Phase 1 here.
- The PLAN-030 resource-access milestone keeps its objective-specific contract in
  [`../../plans/030-compute-resource-access/`](../../plans/030-compute-resource-access/)
  (mission + evaluator), which is the gate for Phase 1.
- Durable rationale and the option survey live in
  [`../../design/compute_backend_research.md`](../../design/compute_backend_research.md).
