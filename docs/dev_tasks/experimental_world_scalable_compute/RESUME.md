# Resume: Experimental World Scalable Compute

## Last Session Summary

PR #2698 merged the Phase 0-4 scalable-compute foundation. The current follow-up
promotes the live SoA path into the default experimental `World::step` pipeline:
`BatchedRigidBodyIntegrationStage` now handles frame-coupled rigid-body
parenting with parent-before-child local-transform write-back instead of falling
back to `RigidBodyIntegrationStage`. The follow-up also wires
`pixi run bm-compute-check` into the performance dashboard workflow and tightens
that checker so all 28 expected `bm_compute_graph` rows must be present with
positive finite timings, including the Phase 5 CPU-baseline smoke row.

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
- Phase 4: CPU batch executor (`stepWorldsBatched`), World rollout
  (`rolloutWorldsBatched`), and a pure-SoA control-sequence rollout
  (`rolloutRigidBodyStateBatch`).
- Phase 2 live wiring: `BatchedRigidBodyIntegrationStage` drives a live World
  step through the SoA path (extract -> `integrateRigidBodyStateBatch` via the
  executor -> apply -> parent-ordered frame-cache update). Parity tests verify it
  matches the per-entity stage to 1e-10 across a full dynamic step (linear force,
  torque, orientation) for free and frame-coupled bodies. The orientation scheme
  is unified on the exponential map and `RigidBodyModelBatch` carries inertia, so
  the SoA stage is the default drop-in path for unconstrained rigid bodies.

## Current Branch

`feature/experimental-world-scalable-compute-cuda-reconcile` — local branch
reconciling the scalable-compute gates with the opt-in CUDA MVP from draft PR
#2710. CUDA remains off by default and is enabled through the `cuda` Pixi
environment or `DART_ENABLE_EXPERIMENTAL_CUDA=ON`; no GitHub mutation has been
made from this branch.

## Immediate Next Step

`BatchedRigidBodyIntegrationStage` now matches the per-entity integrator across a
full free-body dynamic step and frame-coupled rigid-body parenting. The default
experimental `World::step` pipeline selects the batched stage behind the executor
seam. The natural next steps, in rough order:

1. Phase 3's deliverables are all implemented: determinism gate (bitwise for
   map-only), cost gate (`ParallelExecutor::setInlineThreshold`), explicit-SIMD
   orientation kernel (`integrateOrientationsSimd`) with scalar fallback, and a
   deterministic reduction (`totalKineticEnergy`) with a fixed-ULP tolerance
   gate. The one open exit criterion -- "parallel/SIMD beats the Phase 0
   baseline" -- is not achievable on today's physics: measured
   `BM_RigidBodyStepParallel` is slower than sequential because trivial Euler
   integration is overhead-bound (the plan's thesis). It waits for a
   compute-bound workload (contact solver) plus an extension of the checked
   `pixi run bm-compute-check` corpus and dashboard contact-shaped proxy/Phase 5
   CPU-baseline series on stable benchmark CI.
2. Phase 4 heterogeneous batches (deferred-by-design to Phase 6 in `01-plan.md`).
3. Phase 5 local CUDA substrate is reconciled with the gate shape: optional CUDA
   benchmark files must register the packet-compatible
   `BM_Phase5RigidBodyBatchGpu/4096/128/100` row, the packet checker requires
   build/import and policy evidence booleans, and the CUDA MVP remains private
   and opt-in. The Phase 5 exit is still not complete until a project-owned GPU
   runner runs the CUDA build/import path and a measured go/no-go packet passes.
4. Phase 6 reassess remains gated on Phase 5 evidence.

Keep `entt` internal and the public handle API unchanged.

## Context That Would Be Lost

- Build/test gotcha: `pixi run build` builds libraries only, NOT the unit-test
  binaries. Use `pixi run build-simulation-experimental-tests` (target
  `dart_experimental_tests`) before `ctest -L simulation-experimental`, or you
  will run stale test binaries that silently pass.
- The world-space dynamics are frame-independent: only the `localTransform`
  bookkeeping depends on parent frames, which is why the batched stage integrates
  in flat SoA order and then writes rigid-body frame properties
  parent-before-child.
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
  is the hard-case baseline; it is a reproducibility/smoke surface, not backend
  selection evidence. `pixi run bm-compute-check` plus the performance dashboard
  keep the full expected compute benchmark corpus reproducible.
- Current validation evidence:
  - `pixi run test-all` passed after all current code, docs, workflow, checker,
    and Phase 5 GPU packet-template edits. The docs phase still emits the known
    generated-stub `None_` autodoc warnings, but the documentation build passes.
  - `pixi run -e cuda test-cuda` passed on a local CUDA host after reconciling
    draft PR #2710 with this task's gates. It built the opt-in CUDA target, ran
    `test_rigid_body_state_batch_cuda`, and ran the smoke benchmark rows
    `BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10` and
    `BM_Phase5RigidBodyBatchGpu/1024/128/10`.
  - The local manual full Phase 5 row also ran successfully with
    `BM_Phase5RigidBodyBatch(CpuBaseline|Gpu)/4096/128/100`, writing
    `.benchmark_results/phase5_cuda_local_full.json`; median local timings were
    about 3.49 s CPU versus 38.3 ms GPU. This is useful local evidence only and
    does not close Phase 5 without project-owned GPU build/import CI and a
    passing go/no-go packet for the same change.
  - `pixi run test-simulation-experimental` passed after the default batched
    world-step update.
  - `pixi run bm-compute-check --benchmark-min-time 1ms` checked all 28 expected
    rows after the checker tightening.
  - Checker/dashboard Python tests passed.
  - `pixi run pytest
python/tests/unit/test_check_no_gpu_runtime_dependencies.py
python/tests/unit/test_check_compute_backend_boundaries.py
python/tests/unit/test_check_phase5_gpu_packet.py
python/tests/unit/test_check_phase5_cuda_benchmark_contract.py
python/tests/unit/test_check_compute_graph_benchmarks.py
python/tests/unit/test_performance_dashboard_workflow.py
python/tests/unit/test_run_performance_dashboard_benchmarks.py -q` passed
    with 54 tests after narrowing the no-GPU dependency checker to default/core
    manifests while allowing explicitly opt-in sidecar Pixi features, and after
    adding the Phase 5 CUDA benchmark-contract checker and packet evidence
    booleans.
  - Checker input mode passed against
    `.benchmark_results/compute_graph_check.json`.
  - `pixi run bm-phase5-gpu-packet-check --write-template <packet.json>` is the
    checked template path for future manual Phase 5 GPU evidence packets. The
    template now includes false-by-default booleans for GPU build/import,
    backend-boundary, no-GPU default/core dependency, and Phase 5 benchmark
    contract evidence; `--input` requires them to be true.
  - `pixi run python scripts/run_performance_dashboard_benchmarks.py --dry-run`
    listed the contact-shaped dashboard command and Phase 5 CPU-baseline
    command.
  - `pixi run pytest python/tests/unit/test_check_phase5_gpu_packet.py -q`
    passed for the executable Phase 5 go/no-go packet validator.
  - `pixi run pytest python/tests/unit/test_check_compute_backend_boundaries.py -q`
    passed for the executable Phase 5 backend-boundary checker.
  - `pixi run check-compute-backend-boundaries` passed for the executable
    Phase 5 backend-leakage API-boundary review.
  - `pixi run check-no-gpu-runtime-dependencies` passed for the executable
    no-GPU default/core package manifest gate; opt-in Pixi GPU sidecar features
    are intentionally allowed.
  - A direct parse of PR #2710's `pixi.toml` with the updated
    `check_no_gpu_runtime_dependencies.py` passed, proving its opt-in CUDA
    feature shape is compatible with the default/core no-GPU dependency gate.
  - `pixi run check-phase5-cuda-benchmark-contract` passed for the optional CUDA
    benchmark contract. Its regression tests intentionally reject PR #2710's
    current `BM_CudaRigidBodyStateBatchLinear/<bodies>` smoke-row shape as
    insufficient for the Phase 5 go/no-go packet.
  - A direct parse of PR #2710's
    `tests/benchmark/simulation/experimental/bm_cuda_rigid_body_state_batch.cpp`
    with `check_phase5_cuda_benchmark_contract.py` reported the expected two
    violations: missing `BM_Phase5RigidBodyBatchGpu` and missing the
    `4096/128/100` manual go/no-go workload.
  - `pixi run docs-build` passed after tightening Doxygen `internal` symbol
    exclusions, and `pixi run lint` then stayed green against the generated docs
    tree.
  - `pixi run lint` passed.
  - `pixi run check-lint` passed after adding the Phase 5 CUDA benchmark
    contract task, packet evidence booleans, and extending
    simulation-experimental formatting checks to `.cu`/`.cuh` files.
  - `pixi run check-api-boundaries` passed after the checker was tightened to
    require Doxygen `internal` symbol exclusions.
  - `git diff --check` passed.
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

Then read `01-plan.md`. The remaining work is not another Euler-integration
cleanup: keep `pixi run bm-compute-check`, the dashboard contact-shaped proxy
surface, and the Phase 5 CPU-baseline surface green; wait for a compute-bound
contact/constraint workload before closing the Phase 3 speedup exit; and
provision a project GPU runner plus build/import CI before starting Phase 5.
The GPU runner/CI prerequisites are owned by project maintainers/infrastructure;
the package shape, pre-registered go/no-go threshold, and
`pixi run bm-phase5-gpu-packet-check --write-template <packet.json>` /
`--input <packet.json>` plus `pixi run check-compute-backend-boundaries` and
`pixi run check-no-gpu-runtime-dependencies` evidence gates are now durable in
`docs/design/scalable_compute_decisions.md`. If continuing through draft PR
#2710, first reconcile its opt-in CUDA benchmark/test names and Pixi feature
shape with these gates, especially
`pixi run check-phase5-cuda-benchmark-contract`, rather than loosening the Phase
5 exit criteria.
For the local CUDA reconciliation path, use `pixi run -e cuda test-cuda` on a
CUDA host after the default checks pass. The runner/CI prerequisite remains
maintainer/infrastructure-owned; local CUDA evidence is useful but does not close
Phase 5 without the build/import gate and packet evidence.
