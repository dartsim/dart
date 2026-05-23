# Resume: Experimental World Scalable Compute

## Last Session Summary

Eight verified commits on the feature branch (each green: `pixi run build`,
`pixi run lint`, experimental ctest targets, benchmark): Phase 0/1 + plan (EnTT
concurrency contract + debug assert, contact-shaped benchmark proxy,
resource-access metadata with `findResourceHazards`/DOT/per-entity kinematics
wiring); O((N+E) log N) topological sort; multi-worker (1/2/4/8) determinism
parity test; Phase 2 seed (`RigidBodyStateBatch` flat-scalar SoA +
`extractRigidBodyState`/`applyRigidBodyState`); a code-reviewer pass whose
Critical (state-batch bounds check) and Major (hazard reachability cost) findings
were fixed; API-inventory/doc sync; and the leading world dimension
(`extractRigidBodyStateBatch`/`applyRigidBodyStateBatch`, homogeneous multi-world).

Since then: scalar-generic SoA kernels (`integratePositionsSemiImplicit`,
`integrateVelocitiesSemiImplicit`) and a batched linear integrator
(`integrateRigidBodyStateBatchLinear`); a CPU batch executor (`stepWorldsBatched`)
and rollout (`rolloutWorldsBatched`); and the immutable Model batch
(`RigidBodyModelBatch` + `extractRigidBodyModelBatch` + a model-based integrator
overload) realizing the Model/State/Control split.

## Current Branch

`feature/experimental-world-scalable-compute` — sixteen commits ahead of `main`,
working tree clean, all gates green. Not pushed; accumulating toward one larger
DART 7 PR.

## Immediate Next Step

The remaining Phase 2 work is the larger integration (best for a fresh session):
add an angular (orientation-from-angular-velocity) scalar-generic kernel, then
wire the batched SoA integrator into the live `WorldStepStage` pipeline so
`World::step` drives the SoA path instead of per-entity `registry.get` (the
current `integrateRigidBody` does the full force->velocity->position->orientation
update with inertia, so the SoA path must reach parity before it can replace it).
After that: Phase 3 explicit SIMD on the hot kernels; Phase 4 control sequences +
heterogeneous batches; Phase 5 GPU (blocked on GPU hardware/CI + the CUDA-vs-SYCL
benchmark); Phase 6 reassess. Keep `entt` internal and the public handle API
unchanged; see `01-plan.md`.

## Context That Would Be Lost

- The state-representation refactor (immutable Model + batched SoA State) is the
  true foundation and is intentionally pulled forward to Phase 2, before SIMD,
  batch, or GPU work. Today's array-of-structs access over `entt::registry`
  (`world_step_stage.cpp` `integrateRigidBody`) exercises the scheduling seam but
  not the data seam that transfers to the GPU.
- Determinism: a parity test (`RigidBodyStepParallelMatchesSequential`) and a
  debug-only hazard assertion in `ParallelExecutor` now guard it. The Phase 3
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
  `docs/plans/030-compute-resource-access/` (mission + evaluator); the former
  narrow dev task is consolidated into `01-plan.md` Phase 1.
- Known follow-up surfaced by Phase 0: `ComputeGraph::buildTopologicalOrder` is
  O(N^2) and dominates `SequentialExecutor` on large graphs; replace with an
  O(N+E) Kahn's queue in Phase 3.
- Keep Taskflow behind the executor seam; keep all backend names out of the
  public API; do not touch the classic `dart::simulation::World`.

## How to Resume

```bash
git status && git log -3 --oneline
```

Then read `01-plan.md`, confirm Phase 0 is still the next step, and begin with the
executor-parity test described under "Immediate Next Step."
