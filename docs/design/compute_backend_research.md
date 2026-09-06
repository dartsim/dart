# Compute Backend Research

## Status And Decision Boundary

Research refreshed **2026-09-04** from upstream documentation/releases and
DART parent revision `afe6c7d`. This is decision input, not a claim that the
candidates were benchmarked inside DART. [PLAN-030](../plans/030-compute-scalability-roadmap.md)
owns executable comparison packets; [compute decisions](scalable_compute_decisions.md)
owns the semantic graph and acceptance contract. [PLAN-040](../plans/040-dart7-release-hardening.md)
owns the milestone backend, precision and example requirements.

There is no single “best parallel library” independent of workload and layer.
A task scheduler, device-kernel portability library and device graph facility
solve different problems. Compare combinations without committing the public
API to any of them. Vendor speedup and scheduling-cost figures are
workload-specific reports, not DART results or bounds on achievable performance.

## Current DART Baseline

- `ComputeExecutor` has sequential and Taskflow-backed implementations;
  [parallel_executor.cpp](../../dart/simulation/compute/parallel_executor.cpp)
  caches lowering but also owns a separate range-worker pool. Total worker
  budget, nested/concurrent use and profiling behavior require qualification.
- DART's [Pixi lock](../../pixi.lock) resolves Taskflow **4.1.0**; the
  [manifest](../../pixi.toml) allows `>=4,<5`; the CMake fallback is **4.0.0**.
  The project host standard is C++23; the private CUDA target currently uses
  CUDA C++17. A library requiring device C++20 therefore needs a real build
  prototype, even though the host already supports C++23.
- Resource-access metadata and explicit graph edges exist. Metadata validation
  does not infer edges, resolve aliases or implement deterministic reductions.
- CPU SI physics remains the default World path. Resident CUDA rigid
  integration and separate solver kernels exist, but host Taskflow dispatch
  does not turn those CPU stages into device kernels.
- `dart/simd/` supplies scalar and architecture-specific primitives. Reuse
  measured primitives; their presence does not prove every solver vectorizes.
- Trusted CUDA CI runs runtime tests; fork PRs use a hosted compile fallback.
  The historical Phase 5 integration packet is preserved in the decision doc,
  with its original scope and host. It is not a contact-simulation benchmark.

## CPU Runtime Shortlist

Versions below are survey pins, not dependency changes. Every promoted runtime
must pass the same DART hard gates before timing can decide between them.

| Candidate and upstream evidence                                                     | Verified characteristics                                                                                                                                                                                                                                             | DART assessment and experiment                                                                                                                                                                                                                                                                                                                              |
| ----------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Taskflow 4.1.0](https://github.com/taskflow/taskflow/releases/tag/v4.1.0)          | MIT, header-only, C++20; task/dependency graphs and cooperative execution facilities.                                                                                                                                                                                | Incumbent: lowest migration cost and existing graph adapter. Measure submission allocation, nesting, cancellation and total workers. Existing constant-allocation-floor tests do not pass a zero-allocation requirement.                                                                                                                                    |
| [Dispenso 1.6.2](https://github.com/facebookincubator/dispenso/releases/tag/v1.6.2) | MIT, C++14, compiled library; thread pool, task sets, parallel loops and graph executors. [Graph executor contract](https://github.com/facebookincubator/dispenso/blob/v1.6.2/dispenso/graph_executor.h) disallows concurrent invocation of the same graph executor. | Serious challenger for shared-pool and repeated-graph workloads. Give each in-flight state its own graph execution record; test allocations and exception/cancellation handling rather than assuming equivalence to Taskflow.                                                                                                                               |
| [oneTBB 2023.1.0](https://github.com/uxlfoundation/oneTBB/releases/tag/v2023.1.0)   | Apache-2.0, compiled runtime; parallel algorithms, task arenas and flow graphs.                                                                                                                                                                                      | Serious challenger for composable CPU execution. Measure arena isolation and packaging costs; avoid a library changing process-wide `global_control` limits for unrelated applications.                                                                                                                                                                     |
| [enkiTS 1.12](https://github.com/dougbinks/enkiTS/releases/tag/v1.12)               | Zlib license, C++11, task sets/dependencies; upstream advertises allocation-free scheduling.                                                                                                                                                                         | Allocation-focused fallback if the primary shortlist fails. Verify the claim for DART's repeated DAG, pinned/nested tasks and error paths. A DART wrapper would own exception capture and cooperative cancellation.                                                                                                                                         |
| [OpenMP 6.0](https://www.openmp.org/specifications/)                                | Compiler/runtime standard for loops, tasks, dependencies and offload; implementation coverage varies.                                                                                                                                                                | Screen with bounded examples, not a default new dependency. It can use explicit deterministic reduction trees; reproducibility is not automatic. Check runtime coexistence, toolchain/wheel support and task allocations. [Clang's support table](https://clang.llvm.org/docs/OpenMPSupport.html) distinguishes implemented features from work in progress. |

Taskflow's [execution](https://taskflow.github.io/taskflow/ExecuteTaskflow.html),
[exception](https://taskflow.github.io/taskflow/ExceptionHandling.html) and
[cancellation](https://taskflow.github.io/taskflow/RequestCancellation.html)
contracts need adapter-specific tests. In particular, cancellation with
semaphore-dependent tasks has documented restrictions. Do not assume a
cancellation request drains device work or makes captured buffers safe to free.

**Recommendation:** retain Taskflow provisionally; compare it with Dispenso
and oneTBB on identical DART workloads. Use enkiTS as the allocation fallback.
An incumbent wins a performance tie only after passing every hard gate. If no
runtime qualifies, retain qualified serial execution and block parallel promotion;
do not write a general-purpose scheduler to evade a failed comparison.

## Kernel And Device Execution Options

| Candidate and upstream evidence                                                                       | Verified role                                                                                                                                                                                                         | DART recommendation                                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Native C++/CUDA                                                                                       | Direct host/device kernels with explicit memory and event ownership; already used by DART.                                                                                                                            | Required reference implementation for the portability experiment; narrow dependency surface, but duplicated kernel implementations need parity tests.                                                                 |
| [Kokkos](https://kokkos.org/kokkos-core-wiki/get-started/quick-start.html)                            | Execution/memory spaces, parallel dispatch and views; current major version requires C++20. [Unmanaged views](https://kokkos.org/kokkos-core-wiki/API/core/view/memoryTraits.html) can wrap externally owned buffers. | Prototype Serial/CUDA over DART-owned buffers against native kernels; test layout, build cost, FP64, completion, allocation and packaged imports. Do not replace DART model/state ownership with framework ownership. |
| [Kokkos Graph](https://kokkos.org/kokkos-core-wiki/ProgrammingGuide/Graph.html)                       | `Kokkos::Graph` exposes graph construction and submission; topology is fixed after instantiation, with backend graph lowering where supported.                                                                        | Compare repeated fixed-shape submission. It is not an automatic scheduler for arbitrary physics or dynamic contact topology.                                                                                          |
| [RAJA resources](https://raja.readthedocs.io/en/develop/sphinx/user_guide/feature/resource.html)      | Loop policies and host/device resources/events.                                                                                                                                                                       | Screen if kernel portability is valuable but Kokkos ownership/toolchain cost is unsuitable. It does not by itself replace a general task-graph runtime.                                                               |
| [SYCL / AdaptiveCpp](https://adaptivecpp.github.io/AdaptiveCpp/architecture/)                         | C++ heterogeneous kernel compilation and runtime scheduling over supported backends; substantial compiler/runtime integration.                                                                                        | Portability screen after the native/Kokkos experiment. Pin compiler/runtime/device combinations and match floating-point flags. Broader vendor reach is a hypothesis until tested on target hardware.                 |
| [NVIDIA stdexec](https://github.com/NVIDIA/stdexec)                                                   | Experimental C++20 sender/receiver implementation; CPU facilities and a separate GPU toolchain path requiring supported `nvc++`, not ordinary NVCC.                                                                   | Track composable completion/cancellation design. Do not infer standard-library shipping dates or adopt it as a foundation without a supported-toolchain prototype.                                                    |
| [Taskflow GPU tasking](https://taskflow.github.io/taskflow/GPUTasking.html)                           | CUDA graph wrappers such as `tf::cudaGraph` / `tf::cudaGraphExec`.                                                                                                                                                    | Possible lowering adapter; existing CPU closures still need actual CUDA kernels. Compare launch/capture overhead rather than selecting from API symmetry.                                                             |
| [CUDA Graphs](https://docs.nvidia.com/cuda/cuda-programming-guide/04-special-topics/cuda-graphs.html) | Graph capture/replay and constrained conditional nodes, including device-side control flow.                                                                                                                           | Ordered streams provide a simple correctness reference. Use graph replay only where repeated workloads justify it; conditional nodes do not capture arbitrary host control flow.                                      |

**Recommendation:** WP-030.3 compares native CPU/CUDA with Kokkos Serial/CUDA
on the same owned storage and numerical contract. CPU scheduling and device
kernel portability may choose different libraries. PLAN-040 owns backend and
precision admission; this comparison does not establish production support for
other GPU vendors or expose framework types through the facade.

## Broader Scheduling References

[StarPU](https://starpu.gitlabpages.inria.fr/features.html) demonstrates task
submission from data dependencies, residency and performance models;
[HPX](https://github.com/TheHPXProject/hpx) adds asynchronous/distributed runtime
facilities; [Legion](https://legion.stanford.edu/overview/) uses logical regions
and privileges for data-centric execution. These are useful references for
resource identity, movement and scheduling. Their broader runtime, packaging
and programming models require a demonstrated need before adopting distributed
execution or an optimizing compiler framework.

The transferable idea is a computation node/group graph with explicit program
order, owned resource ranges and measurable costs. Access modes alone cannot
choose the intended direction of conflicting operations. A graph determines
legal schedules, not a unique optimum; launch overhead, memory locality,
critical paths, contention, solver update semantics and transfers determine
whether a legal schedule is useful.

## Workloads And Comparison Method

Keep two independent axes in evidence:

- **A: independent worlds.** Homogeneous batches expose regular parallel work
  and useful throughput. Include interleaved states and separate controls;
  full World copies or replicated model arrays are not proof of shared-model
  efficiency.
- **B: interacting bodies in one world.** Collision/contact islands are
  irregular and coupled. Single-body examples can isolate physical correctness;
  interacting many-body scenes expose collision and scheduling costs. These
  answer different questions from independent-world throughput. The owning
  plans determine workload sizes and sequence; none implies a universal GPU
  speedup or a mandatory batch-first architecture.

Pre-register a benchmark matrix containing empty/tiny DAGs, coarse independent
nodes, chains, forks/joins, reductions, contact-shaped and independent-island
work, repeated submissions, interleaved worlds, nested calls and representative simulation
scenes. Fix graph shape, physics, accuracy, precision, compiler flags, threads
and hardware. Include first use, prepared execution and end-to-end workflows;
record p50/p95/p99 latency, throughput, allocations, actual total workers,
transfers, setup/capture cost, memory, build/install size and failures. Reuse
`bm-compute-check` and existing packets but distinguish synthetic proxies from
physics. A faster result at lower convergence accuracy is not a runtime win.

Run sanitizer/race and forced-error tests separately from timing. Pin versions
and source revisions, retain raw packets, and write a short decision including
rejected options, hard-gate failures, limitations and revisit triggers. The
packet contract lives in PLAN-030; this survey does not close those packets.

## LCP Solver CUDA Expansion Criteria

`dart/simulation/compute/cuda/lcp_jacobi_batch_cuda.*` implements direct
Jacobi/red-black-GS/PGS packet kernels. Other LCP methods benchmarked in a
CUDA-enabled build can still execute entirely on CPU. A broader direct-device
claim needs native problem-form coverage, update-order/reduction contracts,
CPU/device accuracy evidence, actual-device identity and complete costs.
Promoting a CUDA LCP packet is not evidence that World collision/contact runs
on CUDA. Preserve this distinction in every benchmark and capability row.

## Packaging And Maintenance

Keep the default C++/dartpy install CPU-only with no GPU runtime dependency;
qualify an optional CUDA component and a simple DART-owned device preference.
Test core import without the component/device and explicit unavailable-device
errors. Installation policy is separate from runtime evidence; PLAN-040 owns
the required backend matrix.

Do not require a distinct wheel for every CUDA minor release by assumption:
[NVIDIA minor-version compatibility](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html)
has driver/toolkit and feature constraints. Record the actual toolkit, minimum
driver, GPU architectures, runtime distribution/license obligations and binary
compatibility matrix. A successful source build does not prove wheel viability.
The final CPU/runtime and kernel choices remain **undecided pending prototypes**.
