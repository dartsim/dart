# Resume: Experimental World Scalable Compute

## Last Session Summary

Thirty-one verified commits on the feature branch, each green
(`pixi run build`, `pixi run build-simulation-experimental-tests`,
`pixi run lint`, the experimental ctest label, and the benchmark). The branch now
carries the full standalone structure-of-arrays (SoA) toolkit and a live World
step driven by it at full parity with the per-entity integrator:

- Phase 0/1: EnTT concurrency contract + debug hazard assert, contact-shaped
  benchmark proxy, resource-access metadata (`findResourceHazards`/DOT/per-entity
  kinematics wiring), and an O((N+E) log N) topological sort.
- Determinism (Phase 3 gate): multi-worker (1/2/4/8) parity against the
  sequential reference, with a bitwise gate for the map-only integration stage
  (per-body nodes run concurrently and must match bit-for-bit, since the stage
  has no cross-body reduction).
- Phase 2 state: `RigidBodyStateBatch` (flat-scalar SoA, leading world
  dimension) + the immutable `RigidBodyModelBatch` (Model/State/Control split),
  with `extract`/`apply` (single- and multi-world) validated by a code-reviewer
  pass (Critical bounds-check + Major hazard-cost findings fixed).
- Phase 2 kernels/integrators: scalar-generic `integratePositionsSemiImplicit`,
  `integrateVelocitiesSemiImplicit`, and `integrateOrientationsSemiImplicit`
  (angle-axis exponential map, exact for constant angular velocity); the linear
  and full (`integrateRigidBodyStateBatch`) batched integrators, including a
  torque overload that adds angular velocity from torque via the world-inertia
  (`R I R^T`) LDLT solve, mirroring the per-entity `integrateAngularVelocity`.
- Phase 4 (partial): CPU batch executor (`stepWorldsBatched`), World rollout
  (`rolloutWorldsBatched`), and a pure-SoA control-sequence rollout
  (`rolloutRigidBodyStateBatch`).
- Phase 2 live wiring (this session): `BatchedRigidBodyIntegrationStage` — an
  additive `WorldStepStage` that drives a live World step through the SoA path
  (extract -> `integrateRigidBodyStateBatch` via the executor -> apply +
  frame-cache update), deferring frame-coupled bodies to
  `RigidBodyIntegrationStage`. A parity test verifies it matches the per-entity
  stage to 1e-10 across a full dynamic step (linear force, torque, orientation)
  for free bodies, plus a frame-coupled fallback test. The orientation scheme is
  unified on the exponential map and `RigidBodyModelBatch` now carries inertia,
  so the SoA stage is a full drop-in replacement for free rigid bodies.

## Current Branch

`feature/experimental-cuda-mvp` — adds the first opt-in CUDA MVP for Phase 5.
CUDA remains off by default and is enabled through the `cuda` Pixi environment
or `DART_ENABLE_EXPERIMENTAL_CUDA=ON`.

## Immediate Next Step

`BatchedRigidBodyIntegrationStage` now matches the per-entity integrator across a
full free-body dynamic step, so the SoA path is a verified drop-in. The natural
next steps, in rough order:

1. Optionally let `World::step` (or the default pipeline) select the batched
   stage, behind the executor seam, once frame-coupled coverage is broadened
   beyond the current per-entity fallback (e.g. parent-before-child ordering of
   the frame-cache loop so the SoA path handles rigid-body parenting directly).
2. Phase 3's deliverables are all implemented: determinism gate (bitwise for
   map-only), cost gate (`ParallelExecutor::setInlineThreshold`), explicit-SIMD
   orientation kernel (`integrateOrientationsSimd`) with scalar fallback, and a
   deterministic reduction (`totalKineticEnergy`) with a fixed-ULP tolerance
   gate. The one open exit criterion -- "parallel/SIMD beats the Phase 0
   baseline" -- is not achievable on today's physics: measured
   `BM_RigidBodyStepParallel` is slower than sequential because trivial Euler
   integration is overhead-bound (the plan's thesis). It waits for a
   compute-bound workload (contact solver) plus committed `bm-check` baselines on
   stable benchmark CI.
3. Phase 4 heterogeneous batches (deferred-by-design to Phase 6 in `01-plan.md`).
4. Phase 5 GPU evidence: on a CUDA host run `pixi run -e cuda test-cuda`, then
   dispatch the manual `CI CUDA` workflow once a self-hosted runner carries the
   `cuda` label. Keep this non-required until runner stability and benchmark
   signal are established; Phase 6 remains gated on those results.

Keep `entt` internal and the public handle API unchanged.

## Context That Would Be Lost

- Build/test gotcha: `pixi run build` builds libraries only, NOT the unit-test
  binaries. Use `pixi run build-simulation-experimental-tests` (target
  `dart_experimental_tests`) before `ctest -L simulation-experimental`, or you
  will run stale test binaries that silently pass.
- The world-space dynamics are frame-independent: only the `localTransform`
  bookkeeping depends on parent frames, which is why the batched stage can
  integrate in flat SoA order and only the frame-cache loop (or the per-entity
  fallback) needs parent-before-child ordering.
- The state-representation refactor (immutable Model + batched SoA State) is the
  true foundation, intentionally pulled forward to Phase 2 before SIMD, batch, or
  GPU work, because today's array-of-structs `entt::registry` access exercises
  the scheduling seam but not the data seam that transfers to the GPU.
- Determinism: a parity test (`RigidBodyStepParallelMatchesSequential`) and a
  debug-only hazard assertion in `ParallelExecutor` guard it. The Phase 3
  determinism gate must stay tolerance-based, not bitwise, because float
  reductions are non-associative.
- `ComputeNode::ExecuteFn` is a host `std::function`; it cannot cross to a device.
  The GPU prototype (Phase 5) needs a separate data-describable execution path;
  only the graph and metadata are shared.
- The current CUDA MVP is deliberately private: the build-tree-only `.cuh`
  avoids installed header leakage, and `DART_ENABLE_EXPERIMENTAL_CUDA` only
  builds a private CUDA target plus smoke tests/benchmarks.
- Do not benchmark or pick a GPU backend on the current Euler-only physics; it is
  embarrassingly parallel and will mislead the CUDA-versus-SYCL decision. The
  Phase 0 contact-shaped proxy (`BM_ContactShaped*` in `bm_compute_graph.cpp`)
  is the hard-case baseline; it shows no parallel speedup, as expected.
- Resource-access metadata (Phase 1) is implemented at per-entity stable-string
  granularity and stays diagnostic. The milestone contract is in
  `docs/plans/030-compute-resource-access/`; the former narrow dev task is
  consolidated into `01-plan.md` Phase 1.
- Keep Taskflow behind the executor seam; keep all backend names out of the
  public API; do not touch the classic `dart::simulation::World`.

## How to Resume

```bash
git status && git log -5 --oneline
```

Then read `01-plan.md` and use `pixi run -e cuda test-cuda` for the focused CUDA
validation path.
