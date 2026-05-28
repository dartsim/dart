# Resume: Experimental World Scalable Compute

## Last Session Summary

PR #2698 merged the Phase 0-4 scalable-compute foundation, and the current
branch has merged the latest `origin/main` rigid-body dynamics work. The default
experimental `World::step` pipeline now preserves the contact/multibody solver
ordering (`RigidBodyVelocityStage`, `RigidBodyContactStage`,
`MultibodyForwardDynamicsStage`, `RigidBodyPositionStage`, `KinematicsStage`).
`BatchedRigidBodyIntegrationStage` remains the explicit unconstrained SoA path:
it handles frame-coupled rigid-body parenting with parent-before-child
local-transform write-back instead of falling back to `RigidBodyIntegrationStage`.
The follow-up also wires `pixi run bm-compute-check` into the performance
dashboard workflow and tightens that checker so all expected `bm_compute_graph`
rows must be present with positive finite timings, including the serial
contact-shaped hard case, the compute-bound contact-island speedup surface, and
the Phase 5 CPU-baseline smoke row.

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
- Phase 3 speedup surface: `BM_ContactIslandShaped*` models independent
  contact/constraint islands that are internally Gauss-Seidel ordered but
  parallel across islands. `pixi run bm-compute-check` requires those rows and
  enforces that `BM_ContactIslandShaped/16/512/64` beats sequential by real time.
  The existing `BM_ContactShaped*` serial chain stays as the hard-case
  low-parallelism proxy.
- Phase 2 live wiring: `BatchedRigidBodyIntegrationStage` drives an explicit
  unconstrained World step through the SoA path (extract ->
  `integrateRigidBodyStateBatch` via the executor -> apply -> parent-ordered
  frame-cache update). Parity tests verify it matches the per-entity stage to
  1e-10 across a full dynamic step (linear force, torque, orientation) for free
  and frame-coupled bodies. The orientation scheme is unified on the exponential
  map and `RigidBodyModelBatch` carries inertia, so this stage remains the
  benchmark/prototype seam for SoA/SIMD/device work even though default
  `World::step` now uses the contact-aware split solver pipeline.
- Phase 5 CUDA evidence is now packet-oriented: the CUDA benchmark emits the
  full-workload CPU/GPU final-state error counter, `bm-phase5-cuda-full` writes
  the full-row benchmark JSON, `bm-phase5-cuda-packet` converts the benchmark
  JSON plus explicit evidence flags into the validator's packet shape, and the
  manual CUDA workflow validates/uploads the benchmark JSON and packet artifact.
  `check-phase5-cuda-workflow` keeps that workflow wired to the policy gates,
  full-row task, packet checker, and artifact upload paths.

## Current Branch

`main`. All three scalable-compute PRs are merged: **#2698** (Phases 0-4
foundation), **#2710** (opt-in CUDA smoke path, merged 2026-05-25), and
**#2712** (experimental scalable compute CUDA gates, merged 2026-05-27). There is
no longer an open PR for this task; the Phase 5 packet gates, full-batch
benchmark contract, and the extended `ci_cuda.yml` workflow all live on the
default branch. CUDA remains off by default and is enabled through the `cuda`
Pixi environment or `DART_ENABLE_EXPERIMENTAL_CUDA=ON`. The `CI CUDA / CUDA
Build` job runs CUDA-on configure + `build-cuda` on standard `ubuntu-latest`
(no GPU) on every push to `main`/`release-*` and on path-filtered PRs, and is
green on `main` HEAD. GPU CI is build/import only: the project does not maintain
a self-hosted GPU runner, so the Phase 5 go/no-go runtime packet is measured
manually on a CUDA host and recorded in
`docs/design/scalable_compute_decisions.md` (the recorded GO is 109.6x at
4096/128/100). `check-phase5-cuda-workflow` guards that `ci_cuda.yml` keeps the
build/import gate and never reintroduces a self-hosted GPU runner.

## Immediate Next Step

`BatchedRigidBodyIntegrationStage` now matches the per-entity integrator across a
full free-body dynamic step and frame-coupled rigid-body parenting. The default
experimental `World::step` pipeline selects the merged contact/multibody solver
pipeline; keep the batched stage as the explicit unconstrained SoA path behind
the executor seam. PR #2712 is merged, so all in-repo Phase 0-5 deliverables now
live on `main`. Per the project decision to not maintain a self-hosted GPU runner
(too tedious to maintain), Phase 5 is closed on manual CUDA-host evidence rather
than a self-hosted CI runner: the build/import gate runs on GitHub-hosted
`ubuntu-latest`, and the go/no-go packet (109.6x, error 1.78e-15) is recorded in
`docs/design/scalable_compute_decisions.md`. To refresh it on any CUDA host, run
`pixi run -e cuda test-cuda`, `pixi run -e cuda bm-phase5-cuda-full`,
`pixi run bm-phase5-cuda-packet ...`, then
`pixi run bm-phase5-gpu-packet-check --input <packet.json>`.
The remaining work, in rough order:

1. Phase 3's deliverables are implemented: determinism gate (bitwise for
   map-only), cost gate (`ParallelExecutor::setInlineThreshold`), explicit-SIMD
   orientation kernel (`integrateOrientationsSimd`) with scalar fallback,
   deterministic reduction (`totalKineticEnergy`) with a fixed-ULP tolerance
   gate, and a checked compute-bound contact-island speedup surface. Measured
   `BM_RigidBodyStepParallel` remains slower than sequential because trivial
   Euler integration is overhead-bound (the plan's thesis), so backend decisions
   must use the contact/constraint-shaped surfaces instead.
2. Phase 4 heterogeneous batches (deferred-by-design to Phase 6 in `01-plan.md`).
3. Phase 5 is closed (GO). The CUDA MVP stays private and opt-in; optional CUDA
   benchmark files must register the packet-compatible
   `BM_Phase5RigidBodyBatchGpu/4096/128/100` row, and the packet checker requires
   build/import and policy evidence booleans. GPU CI is build/import only on
   `ubuntu-latest`; there is no self-hosted runtime job. The go/no-go decision
   rests on the manual CUDA-host packet recorded in the decisions doc and is
   reproducible on any CUDA host.
4. Phase 6 reassess is unblocked by the Phase 5 GO but unstarted; each item
   (broaden GPU, auto-scheduling, Pattern B, differentiable state) needs its own
   design note and gate.

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
- Do not benchmark or pick a GPU backend on the unconstrained Euler rigid-body
  rows alone; they are embarrassingly parallel and will mislead the
  CUDA-versus-SYCL decision. The Phase 0 contact-shaped proxy
  (`BM_ContactShaped*` in `bm_compute_graph.cpp`) is the hard-case baseline; it
  is a reproducibility/smoke surface, not backend selection evidence. The Phase
  3 speedup evidence is the contact-island proxy (`BM_ContactIslandShaped*`),
  where independent islands give the parallel executor compute-bound work.
  `pixi run bm-compute-check` plus the performance dashboard keep the full
  expected compute benchmark corpus reproducible.
- Current validation evidence:
  - 2026-05-28 full re-verification on `main` (`7f88f2a2908`, all three PRs
    merged): the four policy gates `check-compute-backend-boundaries`,
    `check-no-gpu-runtime-dependencies`, `check-phase5-cuda-benchmark-contract`,
    and `check-phase5-cuda-workflow` all passed; 66 Python checker unit tests
    passed; `bm-phase5-gpu-packet-check --write-template` writes a false-default
    template and `--input` correctly rejects it (numeric-error guard). The
    `dart_experimental_tests` build (Release) succeeded and
    `ctest -L simulation-experimental` was 21/21 green in 0.36 s.
    `bm-compute-check --benchmark-min-time 1ms` checked all 34 rows and enforced
    the contact-island speedup gate (`BM_ContactIslandShaped/16/512/64`
    parallel/sequential = 0.198, well under the 0.95 bound; 4-island 0.430,
    8-island 0.293). The trivial Euler `BM_RigidBodyStep` rows stayed
    overhead-bound (parallel/sequential > 1), confirming why they must not pick
    backends. A local CUDA-host spike (RTX 5000 Ada, nvcc 12.4) ran
    `pixi run -e cuda test-cuda` (parity test 1/1) and `bm-phase5-cuda-full`,
    then generated and validated a packet that was accepted:
    `worldCount=4096 bodyCount=128 stepCount=100`, CPU baseline median 4129.16 ms
    vs GPU full-workload median 37.67 ms (speedup 109.6x, >= the 1.25x gate),
    `maxFinalStateAbsError=1.78e-15` within the Phase 2 tolerance contract. A
    `gh` check confirmed `CI CUDA` is green on `main` HEAD. Following the project
    decision to not maintain a self-hosted GPU runner, this manual CUDA-host
    packet is the recorded Phase 5 go/no-go evidence (GO); it is reproducible on
    any CUDA host and is recorded in
    `docs/design/scalable_compute_decisions.md`.
  - After the latest `origin/main` merge, `pixi run lint` passed and
    `pixi run docs-build` passed with the known generated-stub `None_` autodoc
    warnings.
  - `pixi run test-all` passed after merging current `origin/main` and
    reconciling the rigid-body dynamics default pipeline with the scalable-
    compute branch. The docs phase still emits the known generated-stub `None_`
    autodoc warnings, but the documentation build passes.
  - `pixi run -e cuda test-cuda` passed on a local CUDA host after reconciling
    the merged PR #2710 CUDA path with this task's gates. It built the opt-in
    CUDA target, ran `test_rigid_body_state_batch_cuda`, and ran the smoke
    benchmark rows
    `BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10` and
    `BM_Phase5RigidBodyBatchGpu/1024/128/10`.
  - The local manual full Phase 5 row also ran successfully on 2026-05-26 after
    the latest `origin/main` merge with
    `BM_Phase5RigidBodyBatch(CpuBaseline|Gpu)/4096/128/100`, writing
    `.benchmark_results/phase5_cuda_ci_full.json`; the generated packet passed
    with `worldCount=4096`, `bodyCount=128`, `stepCount=100`, `speedup=94.871`,
    and `maxFinalStateAbsError=1.78e-15`. This is useful local evidence only
    and does not close Phase 5 without project-owned GPU build/import CI and a
    passing go/no-go packet for the same change.
  - `pixi run test-simulation-experimental` passed after reconciling the
    scalable-compute branch with the merged contact/multibody default pipeline.
  - `pixi run bm-compute-check --benchmark-min-time 1ms` checked all 34 expected
    rows after adding the contact-island speedup surface. Local ratios included
    `BM_ContactIslandShaped/4/512/64 = 0.465`,
    `BM_ContactIslandShaped/8/512/64 = 0.173`, and
    `BM_ContactIslandShaped/16/512/64 = 0.510` parallel/sequential by real time;
    the checker now requires the largest row to stay below 0.95.
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
    listed the contact-shaped dashboard command, contact-island dashboard
    command, and Phase 5 CPU-baseline command.
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
  - `pixi run check-phase5-cuda-workflow` passed; after the self-hosted job
    removal it now guards that `ci_cuda.yml` keeps the `cuda-build` build/import
    gate on a GitHub-hosted runner and never reintroduces a self-hosted GPU
    runner.
  - A read-only GitHub runner check on 2026-05-26 saw online self-hosted Linux
    runners labeled `self-hosted`, `Linux`, `X64`, `dartsim`, and `docker`, but
    no runner labeled `cuda`; the project-owned GPU runner prerequisite remains
    open.
  - PR #2710 installed `.github/workflows/ci_cuda.yml` on the default branch;
    PR #2712 extends it with the Phase 5 packet gates, full-row task, packet
    checker, and artifact upload path.
  - A direct parse of PR #2710's original
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
surface, the contact-island speedup surface, and the Phase 5 CPU-baseline surface
green. Phase 5 is closed (GO). GPU CI is build/import only: `CI CUDA / CUDA
Build` compiles the CUDA targets on a GitHub-hosted `ubuntu-latest` runner and is
green on `main`. The project does not maintain a self-hosted GPU runner, so the
go/no-go runtime packet is measured manually on a CUDA host; the pre-registered
threshold, recorded GO result, and the
`pixi run bm-phase5-gpu-packet-check --write-template <packet.json>` /
`--input <packet.json>`, `bm-phase5-cuda-packet`,
`pixi run check-compute-backend-boundaries`, and
`pixi run check-no-gpu-runtime-dependencies` evidence gates are durable in
`docs/design/scalable_compute_decisions.md`. `check-phase5-cuda-workflow` now
guards that `ci_cuda.yml` keeps the `cuda-build` gate and never reintroduces a
self-hosted GPU runner. If changing the CUDA path, keep its opt-in
benchmark/test names and Pixi feature shape reconciled with these gates,
especially `check-phase5-cuda-benchmark-contract`.
To reproduce/refresh the go/no-go on any CUDA host, run `pixi run -e cuda
test-cuda`, `pixi run -e cuda bm-phase5-cuda-full`,
`pixi run bm-phase5-cuda-packet ...`, then
`pixi run bm-phase5-gpu-packet-check --input <packet.json>`.
