# Resume: Experimental World Scalable Compute

## Last Session Summary

Twenty-one verified commits on the feature branch, each green
(`pixi run build`, `pixi run build-simulation-experimental-tests`,
`pixi run lint`, the experimental ctest label, and the benchmark). The branch now
carries the full standalone structure-of-arrays (SoA) toolkit and its first live
wiring:

- Phase 0/1: EnTT concurrency contract + debug hazard assert, contact-shaped
  benchmark proxy, resource-access metadata (`findResourceHazards`/DOT/per-entity
  kinematics wiring), and an O((N+E) log N) topological sort.
- Determinism: multi-worker (1/2/4/8) parity test against the sequential
  reference.
- Phase 2 state: `RigidBodyStateBatch` (flat-scalar SoA, leading world
  dimension) + the immutable `RigidBodyModelBatch` (Model/State/Control split),
  with `extract`/`apply` (single- and multi-world) validated by a code-reviewer
  pass (Critical bounds-check + Major hazard-cost findings fixed).
- Phase 2 kernels/integrators: scalar-generic `integratePositionsSemiImplicit`,
  `integrateVelocitiesSemiImplicit`, and `integrateOrientationsSemiImplicit`;
  the linear and full (`integrateRigidBodyStateBatch`) batched integrators.
- Phase 4 (partial): CPU batch executor (`stepWorldsBatched`), World rollout
  (`rolloutWorldsBatched`), and a pure-SoA control-sequence rollout
  (`rolloutRigidBodyStateBatch`).
- Phase 2 live wiring (this session): `BatchedRigidBodyIntegrationStage` — an
  additive `WorldStepStage` that drives a live World step through the SoA path
  (extract -> `integrateRigidBodyStateBatch` via the executor -> apply +
  frame-cache update), deferring frame-coupled bodies to
  `RigidBodyIntegrationStage`. Two parity tests verify it matches the per-entity
  stage to 1e-10 for free bodies (no angular velocity / torque) and reproduces
  the per-entity result for the frame-coupled fallback.

## Current Branch

`feature/experimental-world-scalable-compute` — twenty-one commits ahead of
`main`, working tree clean, all gates green. Not pushed; accumulating toward one
larger DART 7 PR.

## Immediate Next Step

The SoA path now drives a live step, but only at parity with the per-entity
integrator in the linear regime. To make `BatchedRigidBodyIntegrationStage` a
full drop-in replacement (and let `World::step` select it), the SoA path must
reach parity for spinning bodies under torque:

1. Carry inertia in `RigidBodyModelBatch` and add an angular-velocity-from-torque
   step (the world-inertia LDLT solve that `integrateAngularVelocity` performs).
2. Reconcile the orientation scheme: the per-entity integrator uses an angle-axis
   update while `integrateOrientationsSemiImplicit` uses a quaternion product;
   pick one (or make the kernel match) so a spinning-body parity test passes.
3. Then optionally let `World::step` choose the batched stage.

After that: Phase 3 explicit SIMD on the hot kernels (currently auto-vectorized
at -O3); Phase 4 heterogeneous batches (deferred-by-design in `01-plan.md`);
Phase 5 GPU prototype (blocked on GPU hardware/CI + the CUDA-vs-SYCL benchmark);
Phase 6 reassess. Keep `entt` internal and the public handle API unchanged.

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

Then read `01-plan.md` and continue with the angular-from-torque parity work
described under "Immediate Next Step."
