# PLAN-030: Compute Scalability Roadmap

- Operating state: `PLAN-030` in `docs/plans/dashboard.md`

## Outcome

DART has a concrete compute roadmap for multi-core CPU, SIMD, and candidate GPU
workloads before public GPU APIs become commitments.

## Scope

This plan covers how DART should decide and sequence scalable-compute work:

- multi-core CPU algorithm structure;
- SIMD-friendly data paths;
- benchmark-backed GPU candidate workloads;
- CUDA package constraints;
- SYCL-versus-CUDA decision criteria;
- public API boundaries for accelerated algorithms.

Out of scope for this plan:

- Committing to a GPU backend before workload evidence exists.
- Rewriting DART around one compute backend.
- Treating backend availability as success without research workload evidence.

## Current Evidence

- `dart/simd/` provides explicit SIMD types and geometry helpers.
- `tests/benchmark/simd/` provides SIMD benchmark coverage.
- `docs/design/hierarchical_allocator.md` documents allocator direction for
  cache-friendly simulation.
- `docs/onboarding/build-system.md` documents build options and dependency
  structure.
- `.github/workflows/ci_simd.yml` validates SIMD-oriented CI coverage.

## Workload Candidates

Rank candidate workloads before choosing backend architecture:

| Workload                                   | Why it matters                                       | Early evidence needed                               |
| ------------------------------------------ | ---------------------------------------------------- | --------------------------------------------------- |
| Batched collision and distance queries     | Common in planning, RL, and dataset generation       | Shapes, batch sizes, baseline CPU timing            |
| Batched dynamics                           | Important for ensembles, optimization, and learning  | Model sizes, independence assumptions, state layout |
| Large model ensembles                      | Research workloads often run many worlds or rollouts | Memory layout, scheduler behavior, reproducibility  |
| Differentiable or gradient-heavy workloads | Relevant to optimization and learning                | API expectations, precision requirements            |
| Contact/constraint solving at scale        | Often dominates hard simulations                     | Solver structure, sparsity, data transfer costs     |

Current ranking:

| Rank | Workload                                              | Why now                                                                                                                                           | CPU/SIMD prerequisites                                                                                                                                                  | CUDA/SYCL decision evidence needed                                                                                                                                         |
| ---: | ----------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
|    1 | Contact/constraint solving at scale                   | It connects directly to existing LCP/contact solver contracts, constraint grouping, solver benchmarks, and PLAN-020's first algorithm family.     | Stable solver data contract, deterministic fixed-seed scenarios, contact-group independence evidence, scalar CPU baseline, and Jacobi/blocked-Jacobi comparison points. | Transfer cost for sparse/contact data, whether parallel iterations converge acceptably, and whether kernel structure favors CUDA-specific tuning or portable SYCL kernels. |
|    2 | Batched collision and distance queries                | Planning, RL, and dataset generation commonly issue many independent queries, and `dart/simd/geometry` already targets batch geometry operations. | Representative shape mixes, batch-size ranges, scalar collision baseline, SIMD transform/query microbenchmarks, and stable high-level query API boundaries.             | Device-side shape representation cost, host/device transfer amortization, and whether package paths can ship required backend dependencies.                                |
|    3 | Batched dynamics or large model ensembles             | Ensembles and rollouts are high-value research workloads, but determinism and memory layout are harder than independent queries.                  | World or skeleton independence criteria, scheduler overhead benchmark, allocator/memory-budget evidence, and single-thread correctness preservation.                    | Memory layout and synchronization costs, reproducibility across devices, and whether acceleration can stay behind world/solver options rather than public device types.    |
|    4 | SIMD-heavy mesh, point-cloud, and transform utilities | Existing SIMD module and benchmarks make this a lower-risk proving ground, but it may be less decisive for full simulation throughput.            | Scalar/SIMD benchmark pairs, Eigen interoperability checks, architecture fallback coverage, and realistic data sizes.                                                   | Mostly CPU/SIMD first; GPU evidence is only useful if downstream workloads show bulk transfer amortization.                                                                |
|    5 | Differentiable or gradient-heavy workloads            | Research value is high, but API expectations and precision requirements are not yet grounded in DART-specific evidence.                           | User/research case collection, baseline optimization tasks, precision policy, and interaction with algorithm extension contracts.                                       | Whether gradients require a separate AD stack, custom kernels, or integration with external differentiable-simulation tooling.                                             |

The ranking favors workloads with existing tests and benchmark surfaces before
workloads that need new public API commitments. GPU backend selection remains
blocked until at least one top-ranked workload has benchmark evidence that
includes transfer/setup costs and package impact.

## Decision Framework

### Multi-Core CPU First

Before public GPU commitments:

- identify parallel units that do not break deterministic behavior;
- benchmark thread scheduling overhead;
- preserve single-threaded correctness tests;
- avoid exposing thread pool internals as public API.

### SIMD

SIMD work should:

- keep scalar fallbacks;
- be benchmarked against realistic data sizes;
- avoid forcing users into architecture-specific APIs;
- preserve Eigen interoperability where it matters.

### GPU: CUDA Versus SYCL

Choose by evidence, not preference.

| Criterion            | CUDA kernels                                     | SYCL portability                          |
| -------------------- | ------------------------------------------------ | ----------------------------------------- |
| Performance ceiling  | Likely stronger for NVIDIA-heavy research setups | Depends on compiler/runtime maturity      |
| Packaging complexity | Higher for CUDA toolchains and wheels            | Higher for portability/runtime variance   |
| Platform reach       | Narrower, but common in robotics/ML labs         | Broader in principle                      |
| Maintenance cost     | Focused backend, fewer portability targets       | More backend variance to test             |
| API risk             | Easier to overfit public API to CUDA concepts    | Risk of abstracting before workload facts |

The first GPU milestone should be an internal prototype and benchmark report,
not a public API.

## Workstreams

### 1. Workload Ranking

Collect candidate workloads from examples, benchmarks, issue/PR evidence, and
research use cases. Rank by:

- user value;
- expected performance bottleneck;
- data movement cost;
- testability;
- package/CI feasibility;
- API impact.

### 2. Benchmark Gate Design

Define benchmark gates before implementation:

- baseline CPU implementation;
- scalar versus SIMD comparison where relevant;
- multi-core CPU scaling curve;
- GPU prototype comparison only when transfer and setup costs are included.

Initial gates:

| Workload                                  | Baseline gate                                                                                                                            | Scaling gate                                                                                                                                        | Package/CI gate                                                                                                                    | GPU decision gate                                                                                      |
| ----------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| Contact/constraint solving at scale       | Run the LCP comparison benchmark for standard, boxed, and friction-index scenarios; record solver status and fixed-seed reproducibility. | Compare serial baselines against Jacobi, blocked-Jacobi, red-black Gauss-Seidel, or independent constraint-group candidates before adding new APIs. | Confirm the benchmark runs through `pixi run bm lcp_compare` on the default environment and does not require optional GPU tooling. | Include data-transfer/setup costs and sparse/contact data layout in any CUDA or SYCL prototype report. |
| Batched collision and distance queries    | Build scalar batch scenarios with representative shape mixes and query counts.                                                           | Compare scalar transforms/queries against `dart::simd` geometry benchmarks and any multi-core batching prototype.                                   | Keep optional backend dependencies behind existing collision build toggles; do not require GPU dependencies for default packages.  | Show transfer amortization for realistic batch sizes before exposing device-backed query APIs.         |
| Batched dynamics or large model ensembles | Record single-thread world/skeleton stepping throughput and deterministic replay behavior.                                               | Measure scheduler overhead and independent-world scaling before shared-state parallelism.                                                           | Keep CI smoke coverage CPU-only until a backend is selected; package fallback behavior must be explicit.                           | Prove reproducibility and memory-layout viability before exposing CUDA or SYCL concepts publicly.      |

These gates are intentionally benchmark-first. A backend prototype is not
evidence for public API readiness unless it beats the relevant scalar or
CPU/SIMD baseline after setup, transfer, package, and CI costs are included.

Local baseline evidence:

- Command: `pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`
- Result: passed on the default Release build and built `BM_LCP_COMPARE` first.
- Baseline signal: `Dantzig/Smoke` reported `contract_ok=1`, `iterations=1`,
  `bound_violation=0`, and residual/complementarity counters.
- Timing caveat: CPU scaling was enabled during the local run, so timing values
  are useful only as a command smoke signal, not as portable performance data.

Candidate comparison points for contact/constraint solving:

| Candidate                                      | Compare against                                              | Evidence to collect                                                                                                                                              | API boundary rule                                                                                                        |
| ---------------------------------------------- | ------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| Jacobi-family iterative sweep                  | `Dantzig/Smoke` for correctness and PGS/BGS for throughput   | Fixed-seed boxed and friction-index problems, iteration count to reach tolerance, bound/complementarity counters, and deterministic repeated runs.               | Expose only solver/options selection; keep thread scheduling and row storage internal.                                   |
| Blocked Jacobi over contact-sized blocks       | Scalar `BlockedJacobiSolver` and `BgsSolver` baselines       | Per-contact block sizes, convergence versus block size, cache behavior, and whether contact coupling preserves the v0 `LcpProblem` contract.                     | Do not expose block storage or contact cache layout as public API.                                                       |
| Red-black Gauss-Seidel or graph coloring       | `RedBlackGaussSeidelSolver`, PGS, and BGS                    | Coloring cost, deterministic color assignment, per-iteration wall time, and final validation counters for the same generated scenarios.                          | Keep graph/color data as benchmark/internal detail until a high-level solver policy needs it.                            |
| Independent constrained-group scheduling       | Current serial constrained-group solve in `ConstraintSolver` | Number and size of independent groups, scheduling overhead, determinism under fixed input order, and end-to-end world-step impact beyond solver microbenchmarks. | Select through solver/world policy; do not expose task scheduler internals.                                              |
| SIMD residual and bounds-validation kernels    | Scalar validation in LCP tests and comparison benchmark      | Residual/complementarity validation time, Eigen interoperability, scalar fallback parity, and realistic vector lengths from contact workloads.                   | Keep SIMD types behind validation/solver implementation; public callers should continue to see `LcpResult` counters.     |
| CUDA or SYCL prototype for sparse/contact data | Best CPU/SIMD candidate after setup and transfer costs       | Host/device transfer cost, sparse/contact data layout, kernel occupancy or portability data, package footprint, and CPU fallback behavior on unsupported hosts.  | Prototype internally first; no public device, stream, allocator, or backend-specific type until benchmark evidence wins. |

The first comparison should extend the LCP benchmark harness rather than the
public API. A useful benchmark report should include the smoke command above,
a larger fixed-seed scenario, at least one parallel-friendly candidate, and the
same correctness counters used by the v0 LCP solver contract.

Existing harness evidence:

- `tests/benchmark/lcpsolver/bm_lcp_compare.cpp` already includes fixed-seed
  standard, boxed active-bound, friction-index, and scaled LCP generators.
- Larger friction-index scenarios are registered for Dantzig, PGS, Jacobi,
  BlockedJacobi, RedBlackGaussSeidel, BGS, NNCG, Newton-family, and other
  solvers with contact counts up to 64.
- Local comparison command:
  `pixi run bm lcp_compare -- '--benchmark_filter=BM_LcpCompare_(Dantzig|Jacobi)_FrictionIndex/16'`
- Local result: Dantzig and Jacobi friction-index comparisons both reported
  `contract_ok=1` and `bound_violation=0`; Dantzig completed in one iteration,
  while Jacobi used 13 iterations on the local Release build.
- Timing caveat: CPU scaling was enabled, so these timings are evidence that
  the benchmark path runs and reports comparable counters, not portable
  performance data.

Parallel prototype evidence:

- `BM_LCP_COMPARE` now includes benchmark-only serial and threaded validation
  counter paths for fixed-seed friction-index problems. The threaded candidate
  splits row residual, complementarity, and bounds counters across worker
  threads without changing solver APIs or exposing scheduler details.
- Local command:
  `pixi run bm lcp_compare -- '--benchmark_filter=BM_LcpValidation_.*FrictionIndex/16' --benchmark_min_time=0.05s`
- Local result: serial and threaded validation both reported `contract_ok=1`,
  `bound_violation=0`, and matching residual/complementarity counters on the
  local Release build.
- Decision signal: the threaded candidate was slower at contact count 16 because
  it creates and joins workers per benchmark iteration. Any production path
  should test worker reuse, larger contact counts, or SIMD kernels before
  proposing a public accelerator option.
- Package/CI impact: this benchmark uses only the default C++ standard library
  and existing benchmark target; it does not add CUDA, SYCL, OpenMP, TBB, or
  package-manager requirements.

First-milestone decision:

- PLAN-030 is complete for the first compute milestone with
  contact/constraint solving as the top-ranked workload.
- Current evidence supports continuing with benchmark-only implementation tasks,
  not a public GPU or accelerator API.
- The next implementation task should be scoped separately around worker reuse,
  larger contact counts, SIMD residual kernels, or solver-level parallelism, and
  it should beat scalar baselines after setup and scheduling costs are counted.

### 3. API Boundary Design

Acceleration should normally be selected through high-level algorithm options
or backend policies. Avoid exposing raw kernel, stream, memory pool, or device
types unless there is a clear long-term public contract.

### 4. Packaging And CI Constraints

Before public GPU support:

- decide which package paths can ship it;
- define fallback behavior when GPU support is unavailable;
- define CI coverage for build, import, and at least one smoke benchmark;
- record unsupported platforms explicitly.

## Acceptance Criteria

Active implementation is justified when:

- candidate workloads are ranked;
- benchmark gates are named;
- CPU/SIMD prerequisites are listed;
- CUDA/SYCL decision criteria are tied to workload evidence.

This plan is complete for the first compute milestone when:

- one workload has scalar, SIMD or multi-core, and optional GPU prototype
  benchmark evidence;
- API boundaries avoid backend leakage;
- package and CI implications are documented;
- the roadmap can decide whether public GPU commitments are justified.

## Revision Triggers

Revise this plan when:

- benchmark results change workload priority;
- package manager constraints change;
- CI platform coverage changes;
- a research direction requires a new accelerated workload;
- API-boundary review rejects a proposed backend exposure;
- CUDA or SYCL ecosystem constraints materially change.
