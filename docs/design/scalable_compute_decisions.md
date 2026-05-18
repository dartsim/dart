# Scalable Compute Decisions

## Status

Proposal. This document owns durable CPU, SIMD, and GPU decision rationale.
Current sequencing for scalable-compute roadmap work lives in
`docs/plans/dashboard.md` under PLAN-030.

## Purpose

DART needs a compute roadmap for multi-core CPU, SIMD, and possible GPU
workloads before any public accelerator API becomes a commitment. This design
keeps backend decisions tied to research workloads, benchmark evidence,
packaging impact, and API boundaries.

## Workload-First Decision Model

Choose acceleration work from measured workload needs, not backend availability.
For each candidate workload, collect:

- user value and representative research use cases;
- expected bottleneck and baseline CPU timing;
- data movement and setup cost;
- determinism requirements;
- package and CI feasibility;
- public API impact.

The first GPU milestone should be an internal prototype and benchmark report,
not a public API.

## Multi-Core CPU Constraints

Multi-core CPU work should:

- identify parallel units that do not break deterministic behavior;
- preserve single-threaded correctness tests;
- include scheduling overhead in benchmark results;
- avoid exposing thread pools, task graphs, or worker ownership as public API
  unless they become an intentional long-term contract.

## SIMD Constraints

SIMD work should:

- keep scalar fallbacks;
- benchmark realistic data sizes;
- preserve Eigen interoperability where it matters;
- avoid forcing users into architecture-specific APIs;
- keep ISA-specific code behind internal dispatch or implementation boundaries.

## GPU Prototype Gate

Before public GPU support:

- include transfer and setup costs in benchmarks;
- define fallback behavior when GPU support is unavailable;
- document package paths that can and cannot ship GPU support;
- add CI coverage for build/import and at least one smoke benchmark;
- review API boundaries for backend leakage.

## CUDA Versus SYCL Criteria

Choose CUDA or SYCL from workload evidence:

| Criterion            | CUDA kernels                                     | SYCL portability                          |
| -------------------- | ------------------------------------------------ | ----------------------------------------- |
| Performance ceiling  | Likely stronger for NVIDIA-heavy research setups | Depends on compiler/runtime maturity      |
| Packaging complexity | Higher for CUDA toolchains and wheels            | Higher for portability/runtime variance   |
| Platform reach       | Narrower, but common in robotics/ML labs         | Broader in principle                      |
| Maintenance cost     | Focused backend, fewer portability targets       | More backend variance to test             |
| API risk             | Easier to overfit public API to CUDA concepts    | Risk of abstracting before workload facts |

No public API should expose raw kernel, stream, memory pool, or device concepts
until DART has a clear long-term contract for them.

## Related Designs

`docs/design/hierarchical_allocator.md` covers world-level memory ownership and
allocator direction. It is a dependency for some scalable-compute work, not a
replacement for workload ranking or backend decisions.
