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

## Experimental Compute Graph Boundary

The experimental compute graph is the current CPU-first substrate for scalable
simulation work. It belongs under `dart::simulation::experimental`, keeps
classic `dart::simulation::World` untouched, and uses a backend-neutral
`compute::ComputeExecutor` boundary. Sequential execution is the reference path;
Taskflow is the first parallel backend but should not appear in stable public
APIs.

The first milestone includes graph nodes and explicit dependencies, static
parallel-level inspection, opt-in execution profiles, domain/acceleration
metadata, DOT visualization, graph-backed experimental kinematics and
rigid-body stages, and `compute::WorldStepPipeline` for composing multiple
experimental solver stages. The graph core should stay domain-neutral so
articulated-body, deformable-body, fluid, control, sensor, rendering-prep,
SIMD/data-locality, and GPU-candidate work can share the same inspection and
profiling surface.

## Resource Access Metadata

The next compute milestone is descriptive resource access metadata for graph
nodes. It should declare whether a node reads, writes, mutates, reduces, or uses
scratch resources, and it should improve validation, DOT output, profiling
context, and future graph-shaping decisions.

Keep the first resource-access PR conservative:

- explicit graph edges remain the correctness source of truth;
- resource access metadata starts as diagnostics and validation input;
- obvious read/write and write/write hazards should be detected before
  dependency inference is attempted;
- reductions need explicit declarations instead of being treated as ordinary
  multi-writer access;
- no GPU residency, stream, memory-transfer, solver registry, or rendering API
  should be introduced as part of resource access metadata.

## Freshness And Cache Strategy

Scalable compute needs a freshness model that keeps public APIs simple while
avoiding recursive per-object cache bookkeeping in hot paths. The simulation
API should guarantee fresh common reads and explicit stage synchronization, but
the implementation can choose the cheapest internal strategy for each workload:

- generation or epoch counters for cheap "already fresh" checks;
- stage-local dirty sets when only a subset of frames, shapes, or sensors need
  refresh;
- dependency graphs for ordered kinematics, collision, sensor, and rendering
  prep updates;
- batched cache refresh for SIMD, multi-core CPU, or future accelerator paths;
- diagnostic freshness metadata for profiling, not public dirty flags.

Performance comparisons should measure the full workload: state mutation,
freshness bookkeeping, kinematics refresh, query update, solver work, and
readout. A kinematics-only pipeline should be compared against the full physics
pipeline for the same scene before claiming a user-visible performance gain.

## Multi-Core CPU Constraints

Multi-core CPU work should:

- identify parallel units that do not break deterministic behavior;
- batch small work units when per-node scheduling overhead dominates;
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

## Backend Candidate Scope

Backend technology is an implementation decision until a separate public
backend API is intentionally designed. Candidate implementation paths include
multi-core CPU schedulers, SIMD dispatch, CUDA, Metal, Vulkan compute,
ROCm/HIP, LLVM/JIT code generation, and future runtimes, but user APIs should
name DART concepts, algorithms, solver policies, and workload capabilities.

Adding or replacing a backend should preserve the public simulation API unless
the active major-release compatibility policy explicitly allows a breaking
change. Backend names may appear in build flags, diagnostics, profiling output,
benchmark reports, and developer docs; they should not become required public
type names, solver names, or namespace names without a promotion design and
migration plan.

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

`docs/design/compute_backend_research.md` owns the evidence survey behind these
decisions: how comparable engines scale, a comparison of multi-core and GPU
backend libraries with packaging realities, and a DART workload-candidate
ranking. Use it for the workload ranking and backend evidence that this
framework applies.

`docs/design/hierarchical_allocator.md` covers world-level memory ownership and
allocator direction. It is a dependency for some scalable-compute work, not a
replacement for workload ranking or backend decisions.
