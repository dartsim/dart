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

- the CUDA targets build and import in CI on a GitHub-hosted runner (the nvcc
  compile needs no GPU), and the GPU go/no-go runtime evidence is produced
  manually on a CUDA host (the project does not maintain a self-hosted GPU
  runner, so GPU CI is build/import only);
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
and the CPU/GPU final states match the Phase 2 tolerance contract. Otherwise the
GPU track is parked or cut until a more representative compute-bound workload
exists.

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
default dartpy experimental bindings. CI does not run the GPU comparison: the
`CI CUDA / CUDA Build` job only compiles and imports the CUDA targets on a
GitHub-hosted `ubuntu-latest` runner as the build/import gate, and
`pixi run check-phase5-cuda-workflow` guards that the workflow keeps that gate
wired and never reintroduces a self-hosted GPU runner. The measured go/no-go
packet is generated by a maintainer on any CUDA host with the commands above and
recorded under "Recorded Phase 5 Go/No-Go" below.

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
single-host evidence by design: the project does not maintain GPU CI, so the
packet is reproducible on any CUDA host rather than gated on a project-owned
runner.

## Cross-Cutting Invariants

These held for every phase of the experimental scalable-compute work and remain
binding for follow-on (Phase 6) work:

- Synchronous `World::step()` stays deterministic and unchanged in semantics;
  sequential execution is the reference path. The classic
  `dart::simulation::World` is not modified.
- Executor injection through the abstract `ComputeExecutor` is the only public
  concurrency seam. No `entt`, `comps`, thread-pool, GPU device, stream, kernel,
  memory-pool, or solver-registry type enters the public API.
- Every phase exit cites a checked-in benchmark baseline through `bm`,
  `bm-check`, or `bm-compute-check` — not a vague "benchmark green."
- When dartpy gains a parallel or batched step, it releases the GIL around the
  step and forbids Python callbacks inside compute nodes.

## GPU Packaging Shape

GPU support must be an optional sidecar, not a dependency of the core install:

- Core C++ packages and official `dartpy` wheels keep CPU-only experimental
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
  name CUDA, SYCL, or another backend. Public C++/Python API types, namespaces,
  solver names, and required user configuration must not.
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
