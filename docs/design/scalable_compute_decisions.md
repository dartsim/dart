# Scalable Compute Decisions

## Status

Proposal. This document owns durable CPU, SIMD, and GPU decision rationale.
Current sequencing for scalable-compute roadmap work lives in
`docs/plans/dashboard.md` under PLAN-030.

## Purpose

DART needs a compute roadmap for multi-core CPU, SIMD, and GPU
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

The historical first GPU prototype was internal. Current
[PLAN-040 M1](../plans/040-dart7-release-hardening.md) requires every named rigid
example on CPU and CUDA Float64, including contact and restart. Installation
remains optional. PLAN-030 owns runtime/kernel qualification before broader
implementation; a prototype speedup cannot substitute for this coverage.

## Simulation Compute Graph Boundary

The simulation compute graph is the current CPU-first substrate for scalable
simulation work. It belongs under `dart::simulation` and uses a backend-neutral
`compute::ComputeExecutor` boundary. Sequential execution is the reference path;
Taskflow is the first parallel backend but should not appear in stable public
APIs.

Implemented early compute slices include graph nodes and explicit dependencies, static
parallel-level inspection, opt-in execution profiles, domain/acceleration
metadata, DOT visualization, graph-backed kinematics and
rigid-body stages, and `compute::WorldStepPipeline` for composing multiple
solver stages. The graph core should stay domain-neutral so
articulated-body, deformable-body, fluid, control, sensor, rendering-prep,
SIMD/data-locality, and GPU-candidate work can share the same inspection and
profiling surface.

## Resource Access Metadata

Resource-access metadata already declares reads, writes, mutations, reductions
and scratch and supports diagnostics. It does not infer dependencies or prove
alias-safe access. Retain the following contract while qualifying it in M1:

- explicit graph edges remain the correctness source of truth;
- resource access metadata starts as diagnostics and validation input;
- obvious read/write and write/write hazards should be detected before
  dependency inference is attempted;
- reductions need explicit declarations instead of being treated as ordinary
  multi-writer access;
- no GPU residency, stream, memory-transfer, solver registry, or rendering API
  should be introduced as part of resource access metadata.

## Semantic Graph, Executable Plan And Runtime Adapter

Keep three layers separate:

1. **DART semantic graph:** physical/program order, input/output resources,
   solver-iteration boundaries and coarse computation groups. Explicit edges
   are authoritative in M1. Domains do not dictate one task per entity.
2. **Executable plan:** bind node/group work to state-owned buffers and kernel
   implementations; choose serial/parallel/device lowering, scratch, completion
   dependencies and optional capture. Validate before running.
3. **Runtime adapter:** Taskflow, another CPU pool or a device graph submits
   already-defined work. It cannot invent physics dependencies or turn host
   closures into device kernels.

Each resource needs identity including model/state/lane, range and alias
information. Infer RAW, WAR and WAW edges only against declared program order;
unknown aliasing serializes or rejects. Read/read may overlap; reductions need
owned partials and an explicit merge tree. `Reduce` metadata alone is not a
race-freedom or reproducibility guarantee. Debug hazard diagnostics must be
complemented by release-mode validation and adversarial dependency tests.

Group coarse stages/islands and small compatible kernels to amortize overhead.
A group preserves its dependency and observable error boundaries. A solver's
Gauss-Seidel order cannot silently become Jacobi, and a fixed-iteration method
cannot gain a different convergence policy because tasks were rescheduled.
M1 may execute whole solver loops in one node or ordered stream; M2 measures
coarsening, graph updates, conditional device nodes and critical-path/locality
heuristics. Scheduling is constrained optimization measured on workloads, not
a promise of globally optimal job ordering.

Immutable topology may be shared. Mutable graph execution records, bindings,
profilers, continuation and scratch belong to each in-flight state. A runtime
pool may be shared within a documented total worker budget, including range
workers and nested solver work. Test interleaving, concurrent invocations,
nesting, teardown and profiling on/off; instrumentation must preserve dispatch.

A device node completes when the work and dependent writes complete, not when
its host launch returns. Use correct event dependencies or synchronous
completion; asynchronous overlap is not an M1 requirement. Error/cancellation
must stop dependent work, drain outstanding device jobs before buffers are
released and leave a documented state. Invalid device contexts require explicit
reconstruction. Do not promise rollback merely from catching a host exception.

Active contact counts are data within prepared capacity. Graph caches need
explicit invalidation for topology, selected variant/configuration, buffer
identity, capacity, device/context and restore. Snapshot files contain semantic
state and versioned configuration, not pointers, streams, captured graphs or
thread-pool state; reconstruct those before the prepared step boundary.

WP-030.1–030.4 select a runtime/kernel combination using the
[research shortlist](compute_backend_research.md). Hard gates are correctness,
deterministic supported reductions, lifetime/isolation, error draining,
first-post-bake no allocation including submission, bounded workers, package
isolation and supported toolchains. Rank adoption candidates only after they qualify;
diagnostic/baseline timings may include clearly labeled unqualified candidates. WP-122.8 verifies the complete selected M1 pipeline afterward.

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

The checked CPU scalability surface lives in `pixi run bm-compute-check`.
`BM_ContactShaped*` is the serial, low-parallelism contact/constraint-shaped
hard case. `BM_ContactIslandShaped*` is the compute-bound speedup surface: each
island is internally sequentially coupled, but independent islands write
disjoint state and can run concurrently. The checker requires the largest
contact-island row to beat sequential by real time so Phase 3 evidence does not
come from trivial Euler-only rigid-body rows.

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
- review API boundaries for backend leakage with
  `pixi run check-compute-backend-boundaries`.

The Phase 5 prototype is allowed to start only after these prerequisites are
true:

- the CUDA targets compile on the GitHub-hosted fork-PR fallback, and trusted
  same-repository PRs, protected branch pushes, and manual dispatches run the
  CUDA runtime smoke path on the `ubuntu-latest-gpu` runner;
- a sidecar package/component plan exists that does not add GPU runtime
  dependencies to the default C++ package, default Pixi environment, or official
  `dartpy` wheel;
- the prototype can run an identical-semantics CPU fallback in the same test and
  benchmark packet;
- the benchmark packet measures full workload time, including host/device
  transfer, setup, kernel execution, and readback needed to compare final state.

The first go/no-go benchmark is pre-registered as homogeneous-batch rigid-body
integration with `worldCount = 4096`, `bodyCount = 128`, and `stepCount = 100`
over the same `RigidBodyStateBatch` and controls on CPU and GPU. If runner memory
cannot hold that case, use the largest power-of-two `worldCount` that fits, but
do not claim a go decision below `worldCount = 1024`. The GPU path passes only if
its median full-workload time is at least 1.25x faster than the CPU batch median
and the CPU/GPU final states match the Phase 2 tolerance contract. Otherwise that historical prototype failed its gate. This threshold remains
the Phase 5 packet contract. [PLAN-040](../plans/040-dart7-release-hardening.md)
owns current milestone backend requirements; this historical batch threshold
does not define acceptance for other workloads.

The checked CPU smoke row is
`BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10`; the full
`BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100` row is reserved for the manual
Phase 5 go/no-go packet produced on a CUDA host, so routine CI does not spend
every run on the full GPU comparison workload.
If an optional CUDA benchmark file is present, `pixi run
check-phase5-cuda-benchmark-contract` requires it to register the matching
`BM_Phase5RigidBodyBatchGpu/4096/128/100` row so measured GPU benchmark JSON can
feed the packet checker without ad hoc renaming.

Manual Phase 5 go/no-go reports should start from
`pixi run bm-phase5-gpu-packet-check --write-template <packet.json>` and must
pass `pixi run bm-phase5-gpu-packet-check --input <packet.json>` after measured
values are filled in. The packet combines Google Benchmark median rows for the
full CPU fallback and matching GPU prototype workload with metadata proving the
packet includes transfer, setup, kernel/compute, and readback time, plus the
final-state CPU/GPU absolute error. The packet also carries boolean evidence
that the GPU build/import gate, backend-boundary check, no-GPU default/core
dependency check, and Phase 5 benchmark-contract check all passed for the same
change. The checker enforces the `worldCount >= 1024`, `bodyCount = 128`,
`stepCount = 100`, 1.25x median speedup, final-state tolerance, and evidence
rules above.
When a CUDA benchmark JSON file is available, the packet can be generated and
validated without hand-copying rows:

```bash
pixi run -e cuda bm-phase5-cuda-full
pixi run bm-phase5-cuda-packet \
  --benchmark-json .benchmark_results/phase5_cuda_ci_full.json \
  --output .benchmark_results/phase5_cuda_packet.json \
  --includes-transfer-setup-compute-readback \
  --gpu-build-import-gate-passed \
  --compute-backend-boundaries-passed \
  --no-gpu-runtime-dependencies-passed \
  --phase5-benchmark-contract-passed
pixi run bm-phase5-gpu-packet-check \
  --input .benchmark_results/phase5_cuda_packet.json
```

The same go/no-go packet must include `pixi run check-compute-backend-boundaries`
evidence so backend-specific concepts stay out of public C++ headers and the
default dartpy simulation bindings. CI runs the CUDA smoke path on trusted GPU
events, while fork PRs stay on the hosted compile fallback. The measured
go/no-go packet is still generated by a maintainer on a CUDA host with the
commands above and recorded under "Recorded Phase 5 Go/No-Go" below.
`pixi run check-phase5-cuda-workflow` guards the trusted-event GPU gate and
fork-PR fallback.

### Recorded Phase 5 Go/No-Go

Measured on a CUDA host (NVIDIA RTX 5000 Ada Generation, nvcc 12.4) on
2026-05-28 against `main` with the commands above:

- Pre-registered workload `worldCount=4096 bodyCount=128 stepCount=100`.
- CPU batch median 4129.16 ms vs GPU full-workload median 37.67 ms (transfer +
  setup + compute + readback) -> 109.6x median speedup, comfortably past the
  1.25x gate.
- CPU/GPU final-state max absolute error 1.78e-15, within the Phase 2 tolerance
  contract; the CUDA parity unit test passes.
- `pixi run bm-phase5-gpu-packet-check --input
.benchmark_results/phase5_cuda_packet.json` accepted the packet.

Decision: **GO** on the kill criterion. The GPU prototype clears the speedup and
parity bar, so the Phase 6 GPU track is not cut. CUDA is the implemented backend
(Taskflow GPU tasking is CUDA-only); the CUDA-versus-SYCL choice for any broader
backend stays a Phase 6 decision behind the internal executor interface, made
only if and when Phase 6 GPU stage coverage is pursued. This decision rests on
single-host historical measurements. Current CI runs CUDA runtime tests for
trusted events and compile-only coverage for fork PRs; neither automatically
refreshes this historical performance packet.

## Cross-Cutting Invariants

These held for every phase of the scalable-compute work and remain
binding for follow-on (Phase 6) work:

- Synchronous `World::step()` stays deterministic and unchanged in semantics;
  sequential execution and independent physical oracles are the reference.
  Optional DART 6 differential evidence belongs on `release-6.*` branches.
- Executor injection remains a DART-owned abstract execution seam; concrete
  runtime implementations stay private. No `entt`, `comps`, thread-pool, GPU device, stream, kernel,
  memory-pool, or solver-registry type enters the public API.
- Every phase exit cites a checked-in benchmark baseline through `bm`,
  `bm-check`, or `bm-compute-check` — not a vague "benchmark green."
- When dartpy gains a parallel or batched step, it releases the GIL around the
  step and forbids Python callbacks inside compute nodes.

## GPU Packaging Shape

GPU support must be an optional sidecar, not a dependency of the core install:

- Core C++ packages and official `dartpy` wheels keep CPU-only
  simulation support and must import/run without a GPU runtime present.
  `pixi run check-no-gpu-runtime-dependencies` guards the default Pixi and
  dartpy wheel manifests against accidental CUDA/SYCL/ROCm/HIP runtime
  dependencies.
- Default Pixi dependencies must stay GPU-runtime-free; explicitly opt-in Pixi
  features/environments may carry CUDA/SYCL/etc. packages when they are not part
  of the default install surface.
- GPU code may ship as a separate conda component, a separate PyPI package or
  CUDA-index wheel, or a source-only developer option while the packaging story
  is immature.
- Backend-specific libraries, compiler toolchains, and device runtimes must not
  be linked into the default `dart` target or default `dartpy` wheel.
- A CPU fallback with identical semantics is required in the core package before
  any GPU sidecar is useful.
- Build flags, package names, CI labels, diagnostics, and benchmark reports may
  name CUDA, SYCL, or another backend. Third-party runtime/framework types and implementation namespaces stay
  private; DART-owned algorithm/variant names remain public policy concepts. A small DART-owned `cpu`/`cuda` preference and resolved-device report
  are accepted public design under PLAN-041/042. Existing identifier checkers
  still apply until WP-040.2 lands their bounded fixture migration.
- Any sidecar package must have its own build/import smoke CI (the nvcc compile
  needs no GPU, so a GitHub-hosted runner suffices) and a no-GPU import test
  proving the core package remains usable without it.

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

`docs/design/shared_cuda_device_substrate.md` (PLAN-031) owns the intra-codebase
decision of which GPU device-runtime code is shared across experimental CUDA
solvers versus kept solver-local, so new GPU solvers reuse common blocks instead
of reinventing the runtime probe, error mapping, launch config, and device
buffers. It operates within the packaging shape, prototype gate, and
backend-neutrality rules this document owns.
