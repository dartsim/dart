# Resume: Experimental World Scalable Compute

## Last Session Summary

Four verified commits on the feature branch (each green: `pixi run build`,
`pixi run lint`, experimental ctest targets, benchmark): (1) Phase 0/1 + plan —
EnTT concurrency contract + debug assert, contact-shaped benchmark proxy, and
resource-access metadata (`ComputeAccessMode`, `findResourceHazards`, DOT,
per-entity kinematics wiring); (2) O((N+E) log N) topological sort; (3)
multi-worker (1/2/4/8) determinism parity test; (4) Phase 2 seed —
`RigidBodyStateBatch` flat-scalar SoA with `extractRigidBodyState`/
`applyRigidBodyState` and a round-trip test. A code-reviewer pass on the branch
was requested; fold its findings in next.

## Current Branch

`feature/experimental-world-scalable-compute` — four commits ahead of `main`,
working tree clean, all gates green. Not pushed; accumulating toward one larger
DART 7 PR.

## Immediate Next Step

Continue Phase 2: extend `RigidBodyStateBatch` to a leading world dimension > 1
(homogeneous replication), add the immutable Model split, and make the
integration/kinematics kernels scalar-generic (`template <typename Scalar>`,
instantiate `double` only) so they read/write the SoA batch instead of per-entity
`registry.get`. Keep `entt` internal and the public handle API unchanged. This is
the data seam that transfers to multi-core, SIMD, and GPU; see `01-plan.md`
Phase 2.

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
