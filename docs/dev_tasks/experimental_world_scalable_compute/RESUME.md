# Resume: Experimental World Scalable Compute

## Last Session Summary

Phases 0 and 1 are implemented and verified green (build, lint, and the
experimental unit tests all pass). Phase 0: the executor-parity test already
existed (`RigidBodyStepParallelMatchesSequential`); added the EnTT concurrency
contract (documented on `ComputeExecutor`, debug-asserted in `ParallelExecutor`)
and a contact-shaped benchmark proxy. Phase 1: resource-access metadata
(`ComputeAccessMode`, `ComputeResourceAccess`, `ComputeGraph::findResourceHazards`,
DOT surfacing, per-entity kinematics wiring) with 30/30 `test_compute_graph`
cases. Next is Phase 2 (immutable Model + batched SoA State).

## Current Branch

`feature/experimental-world-scalable-compute` — Phase 0/1 changes staged for
commit; verified against `pixi run build`, `pixi run lint`, and the experimental
ctest targets.

## Immediate Next Step

Begin Phase 2: introduce DART-owned state/control owner value types and an
SoA/tensor-friendly State buffer (leading world dimension defaulting to one),
keeping `entt` internal and the public handle API unchanged. Co-locate the
scalar-generic kernel rewrite (`template <typename Scalar>`, instantiate `double`
only) with this change. This is the data seam that transfers to multi-core,
SIMD, and GPU; see `01-plan.md` Phase 2.

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
