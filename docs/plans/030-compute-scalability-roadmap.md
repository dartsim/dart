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
- `docs/design/scalable_compute_decisions.md` owns durable CPU, SIMD, and GPU
  decision rules.
- `docs/design/hierarchical_allocator.md` documents allocator direction for
  cache-friendly simulation.
- `docs/onboarding/build-system.md` documents build options and dependency
  structure.
- `.github/workflows/ci_simd.yml` validates SIMD-oriented CI coverage.

## Open Gaps

- Rank candidate accelerated workloads from examples, benchmarks, issue/PR
  evidence, and research use cases.
- Name benchmark gates for the first workload.
- List CPU/SIMD prerequisites before any GPU prototype.
- Tie CUDA/SYCL decision evidence to concrete workload and package constraints.

## Workload Candidates

Rank candidate workloads before choosing backend architecture:

| Workload                                   | Why it matters                                       | Early evidence needed                               |
| ------------------------------------------ | ---------------------------------------------------- | --------------------------------------------------- |
| Batched collision and distance queries     | Common in planning, RL, and dataset generation       | Shapes, batch sizes, baseline CPU timing            |
| Batched dynamics                           | Important for ensembles, optimization, and learning  | Model sizes, independence assumptions, state layout |
| Large model ensembles                      | Research workloads often run many worlds or rollouts | Memory layout, scheduler behavior, reproducibility  |
| Differentiable or gradient-heavy workloads | Relevant to optimization and learning                | API expectations, precision requirements            |
| Contact/constraint solving at scale        | Often dominates hard simulations                     | Solver structure, sparsity, data transfer costs     |

Durable CPU, SIMD, GPU, and CUDA/SYCL decision rules live in
`docs/design/scalable_compute_decisions.md`.

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

### 3. API Boundary Design

Acceleration should normally be selected through high-level algorithm options
or backend policies. Avoid exposing raw kernel, stream, memory pool, or device
types unless there is a clear long-term public contract.

Use `docs/design/scalable_compute_decisions.md` for backend-leakage constraints.

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
