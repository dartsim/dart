# Compute Backend Research

## Status

Proposal. This document owns the durable evidence survey and DART
workload-candidate ranking behind the scalable-compute roadmap. It complements
[`scalable_compute_decisions.md`](scalable_compute_decisions.md), which owns the
workload-first decision rules and the CUDA-versus-SYCL criteria. Operating state
(priority, status, next action) stays in `docs/plans/dashboard.md` under
PLAN-030; this file does not repeat those fields.

## Purpose

DART's north star names multi-core CPU as a first-class direction, treats SIMD
as already part of the codebase, and requires an explicit, evidence-backed GPU
roadmap with a CUDA-versus-SYCL decision framework before any public accelerator
API becomes a commitment (`docs/ai/north-star.md`). The decision framework
already exists. The gap this document closes is the missing _evidence_: how
comparable engines actually scale, which C++ backend libraries are viable for a
package-distributed research library, and which DART workloads are the real
acceleration candidates.

This is a research and decision-input document, not an API commitment and not a
change to the active PLAN-030 milestone.

## Method And Evidence Basis

- Repository inspection of DART's current compute substrate (the simulation
  compute graph, the SIMD module, native-collision sequencing).
- A 2025-2026 landscape review of comparable physics engines and their compute
  backends.
- A 2025-2026 comparison of C++ multi-core and GPU library options, including
  conda-forge and PyPI packaging realities.
- Deeper threads on ECS automatic parallel scheduling (Unity DOTS, flecs, Bevy),
  the Newton/MuJoCo-Warp architecture, and differentiability versus scalar-
  templating versus `std::execution` timelines.
- A three-perspective review of the verdict and plan (architecture, plan
  critique, and risk analysis) whose corrections are reflected in the verdict and
  the phase order.

Source URLs are listed under [Sources](#sources). Vendor throughput claims
(for example Genesis "43M FPS" or "80x faster") are best-case batched marketing
figures and are treated as upper bounds, not validated benchmarks.

## DART's Current Compute Substrate

New scalable-compute work builds on what already exists; the survey below should
not motivate parallel-from-scratch designs.

- The simulation compute graph under `dart::simulation::compute`
  is the CPU-first substrate. `ComputeExecutor` is the backend-neutral boundary
  with a sequential reference path and a Taskflow-backed `ParallelExecutor`
  (`TaskflowExecutor` is a compatibility alias).
- `ComputeStageMetadata` already carries a `ComputeStageDomain` (Kinematics,
  RigidBody, Collision, Constraint, Sensor, Rendering, and more) and a
  `ComputeStageAcceleration` mask that already reserves `TaskParallel`,
  `DataParallel`, `Simd`, `DataLocality`, and `Gpu`. Acceleration intent is
  therefore already expressible as graph metadata.
- The SIMD module (`dart/simd/`) is mature: SSE4.2, AVX, AVX2, AVX512, NEON, and
  SVE backends with a scalar fallback and Eigen interop.
- DART 6 parity evidence stays on `release-6.*` branches, and Taskflow stays
  behind the executor boundary.
- The native-collision performance plan already sequences single-core, then
  scenario-scale, then multi-core CPU, then optional GPU work for collision data
  structures (`docs/plans/035-native-collision-dashboard.md`).

## Two Acceleration Patterns

The single most important framing from the landscape review is that "parallel
physics" splits into two patterns with different engineering, risk, and payoff.

- **Pattern A — batch many parallel environments.** One small scene replicated
  thousands of times and stepped in lockstep. Serves reinforcement learning,
  sim-to-real, and large-scale optimization users. Requires near-identical
  per-environment shapes, state exposed as device tensors, and ideally the
  learning loop on the same device. This is the model behind MuJoCo MJX, Brax,
  Isaac Lab, Genesis, MuJoCo Warp, and TinyDiffSim.
- **Pattern B — accelerate one large scene.** Many bodies and contacts in a
  single world. Serves the classic DART user: a single robot or a cluttered,
  contact-rich scene. Much harder to accelerate because contact graphs are
  irregular and dynamically sparse. PhysX 5's GPU rigid-body solver is the main
  proof it is tractable; most CPU robotics engines stay multi-threaded here.

Pattern A is the higher-ROI, lower-risk first GPU target because it can reuse
the existing solver replicated across a batch. Pattern B is a multi-year solver
rewrite. DART should treat them as separate roadmap tracks.

## Landscape Evidence

| Engine                  | Backend                                      | Pattern               | Differentiable            | Packaging             | Lesson for DART                                                                                                                                                  |
| ----------------------- | -------------------------------------------- | --------------------- | ------------------------- | --------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| MuJoCo MJX              | JAX/XLA (NVIDIA/AMD/Apple/TPU)               | A                     | Yes                       | pip (CUDA via JAX)    | A vectorizing IR buys batch parallelism and autodiff together, but static shapes punish irregular contact.                                                       |
| Google Brax             | JAX/XLA                                      | A                     | Yes                       | pip (CUDA via JAX)    | Keeping the learning loop on-device — not raw solver speed — is the dominant speedup.                                                                            |
| MuJoCo Warp             | NVIDIA Warp (CUDA)                           | A                     | Not yet                   | pip, CUDA-bound       | XLA portability versus hand-tuned CUDA throughput is a real fork; DeepMind keeps both.                                                                           |
| NVIDIA Warp (framework) | Python kernels JIT to C++/CUDA, CPU fallback | substrate             | Yes (reverse-mode)        | pip (`warp-lang`)     | Zero-copy device state and designed-in differentiability are the lessons to take; DART can realize them in its C++ core rather than a Python-first kernel layer. |
| Isaac Gym / Sim / Lab   | PhysX 5 GPU, tensor API                      | A (and B)             | No                        | heavy (Omniverse)     | The GPU tensor state API (no host-device copy) is the most-copied idea; the install weight is the cautionary tale.                                               |
| PhysX 5 standalone      | C++ CPU+GPU, now BSD-3 open                  | A and B               | No                        | C++ SDK               | The mature open GPU rigid-body solver others build on; proves Pattern B is possible.                                                                             |
| Genesis                 | Taichi (Python to GPU)                       | A                     | Partial                   | pip (`genesis-world`) | Multi-solver breadth behind one API is a differentiator; the speed comes from SoA plus GPU, which is reproducible from a C++ core.                               |
| Taichi / DiffTaichi     | `@ti.kernel` JIT to CUDA/Vulkan/CPU          | substrate             | Yes (source-transform AD) | pip (`taichi`)        | Source-to-source AD preserves parallelism better than tape-based array autodiff.                                                                                 |
| Drake                   | C++ CPU, context-per-thread pool             | B (batch via threads) | Yes (`AutoDiffXd` scalar) | pip + apt             | Template-on-scalar gives cheap autodiff; context-per-thread is the safe CPU batch pattern.                                                                       |
| RaiSim                  | CPU multibody, multi-threaded                | B                     | No                        | licensed              | Single-scene contact speed is achievable on CPU; no public GPU batch pipeline.                                                                                   |
| Bullet / PyBullet       | Bullet 3 has OpenCL GPU, unexposed           | B                     | No                        | pip                   | An unexposed GPU backend delivers no user value; exposure and packaging are the hard part.                                                                       |
| TinyDiffSim             | header-only C++/CUDA, scalar-templated       | A                     | Yes (by construction)     | pip (`pytinydiffsim`) | Templating the whole pipeline on an AD scalar type yields both autodiff and GPU batch.                                                                           |

Cross-cutting lessons:

1. The two patterns need different engineering; batch-RL (Pattern A) is the
   higher-ROI, lower-risk first GPU target.
2. The dominant speedup lever is the GPU tensor state API with no host-device
   copy, not raw kernel speed.
3. For a C++20 core, template-on-scalar-type is the lowest-friction route to
   differentiability and also helps SIMD.
4. Context-per-thread plus a thread pool is the safe near-term multi-core win
   that needs no solver rewrite.
5. Adopt the architectural lessons of the Python-first and array-language
   engines — structure-of-arrays state, zero-copy device views, batch-first
   execution, and designing differentiability in early — inside DART's existing
   C++ core and SIMD stack. A single-source C++ device path (CUDA or SYCL behind
   the executor seam) keeps GPU code in the same language and data layout as the
   CPU path; a Python-embedded kernel framework or array language would invert a
   C++ engine's dependency direction and is better treated as a reference than a
   foundation.
6. The field expects pip wheels with bundled CUDA; the heavy Omniverse install
   is the counter-example to avoid.

## Convergent Reference: Newton

Newton (contributed to the Linux Foundation in 2025 by Disney Research, Google
DeepMind, and NVIDIA, built on NVIDIA Warp) is the closest architectural
reference for where the DART 7 World is already heading. Its shape:

- an immutable `Model` (topology, parameters) built once, plus mutable `State`,
  `Control`, and `Contacts`, all in structure-of-arrays layout with a leading
  world dimension (`nworld`);
- a kernel-per-stage pipeline (kinematics → collision → constraint → solve →
  integrate);
- multiple solvers (MuJoCo-Warp, XPBD, VBD, Featherstone, semi-implicit) behind
  one interface, all operating on the same `Model`/`State`;
- a zero-copy tensor view of device state, so the learning loop never leaves the
  device;
- differentiability via Warp's autodiff tape.

The DART 7 World is structurally convergent with this: EnTT components
are the SoA columns, public handles are thin views, the compute graph is the
kernel-per-stage pipeline, and `ComputeExecutor` is the backend seam where a
device backend slots in without changing `World`. The two moves that complete the
alignment are an explicit immutable-Model / batched-State split with an `nworld`
leading dimension, and treating the device state buffer as the single source of
truth behind the handle facade. The classic object-oriented `Skeleton`/`BodyNode`
core is the opposite of SoA; porting it would be a rewrite, which is the concrete
reason this roadmap is scoped to the DART 7 World.

## ECS Auto-Scheduling Evidence

Automatic parallel scheduling from per-node component access is proven in
production ECS engines (Unity DOTS Job System, flecs, Bevy), and they converge on
exactly the Read/Write/ReadWrite/Reduce/Scratch model the resource-access
milestone proposes: read/read runs in parallel; same-resource write/write or
read/write must be ordered. It is not speculative. The must-have guardrails,
which this roadmap adopts, are:

- access declarations must be complete and honest; an undeclared read or write
  silently races, so debug-mode access tracking should trap violations;
- access metadata is under-constrained on its own — keep DART's explicit edges as
  the correctness source of truth and add an ambiguity detector that flags
  unordered write conflicts;
- defer structural (entity/component creation/removal) changes to command buffers
  merged at minimal sync points; never mutate layout mid-parallel-phase;
- make reductions a first-class, explicitly deterministic primitive (per-thread
  partials, fixed merge order); arbitrary parallel reduction is non-deterministic;
- gate parallelism by cost so tiny nodes run inline; preserve a deterministic
  single-threaded fallback.

## DART Workload-Candidate Ranking

DART's compute-heavy paths and their parallelism candidacy. ABA refers to the
articulated-body algorithm in `Skeleton::computeForwardDynamics()`; constraint
groups are the independent `constrained_group` units fed to the boxed-LCP solver.

| Workload                              | Multi-core CPU           | SIMD                     | GPU pattern                      | Notes                                                                            |
| ------------------------------------- | ------------------------ | ------------------------ | -------------------------------- | -------------------------------------------------------------------------------- |
| Batch of independent worlds/envs      | High (task per world)    | High                     | A (highest ROI)                  | RL/optimization throughput; reuses existing solver.                              |
| Across-skeleton dynamics in one scene | High (task per skeleton) | Medium (spatial algebra) | B-lite                           | Crowds and swarms; many independent ABA sweeps.                                  |
| Intra-skeleton ABA sweep              | Low (only branchy trees) | Medium                   | Hard                             | Recursive tree dependency; tall chains stay serial.                              |
| Collision narrow phase                | High (per pair)          | High (batch SAT/clip)    | B                                | Already a native-collision performance target.                                   |
| Collision broad phase                 | Medium-High              | Low-Medium               | B                                | Keep deterministic ordering when parallelizing.                                  |
| Constraint / contact LCP solve        | Medium (across groups)   | Low-Medium               | B (batched/colored PGS, hardest) | The classic contact-rich bottleneck; intra-group solve is coupled and iterative. |
| Kinematics / frame-cache refresh      | High                     | High                     | Either                           | Tied to the freshness model in `scalable_compute_decisions.md`.                  |
| Sensors / rendering-prep              | High (per sensor)        | Medium                   | Either                           | Perception-heavy workloads.                                                      |

Ranking takeaways:

- The highest-value, lowest-risk acceleration targets need no solver rewrite:
  batch-of-worlds, across-skeleton dynamics, and collision narrow phase are all
  task-parallel today and map directly onto the existing compute-graph executor.
- The constraint/contact LCP solve is the hardest and the most valuable for the
  classic single-scene DART user. Independent constraint groups parallelize, but
  the intra-group iterative solve is contact-coupled; this is where Pattern B GPU
  work would eventually pay off and where it is riskiest.
- SIMD has the broadest reach across kinematics, spatial algebra, and batch
  narrow phase, and it is already partially in place.

## LCP Solver CUDA Expansion Criteria

DART's current direct LCP CUDA execution is intentionally narrow:
`dart/simulation/compute/cuda/lcp_jacobi_batch_cuda.*` owns fixed-iteration
projected Jacobi, red-black Gauss-Seidel, and PGS kernels over dense standard,
boxed, and friction-index packets, plus grouped host-side wrappers for
variable-size world-contact batches on the Jacobi/PGS paths. Those rows are
direct CUDA LCP execution only for the Jacobi, red-black GS, and PGS packet
paths. CUDA-enabled `BM_LCP_COMPARE` rows for Dantzig, Lemke, Baraff, Newton,
MPRGP, Interior Point, ADMM, SAP, and other manifest solvers are CPU solver rows
emitted from a CUDA-enabled build tree; they prove build and runtime
compatibility, not device execution of those solver algorithms.

An all-solver or broader-direct-CUDA LCP claim needs a separate design slice
before implementation. The minimum acceptance bar is:

- a backend-neutral execution contract that records solver identity, problem
  form, fixed iteration or convergence policy, and whether the row executed
  device kernels;
- CPU-vs-device parity tests for every promoted solver/problem-form pair,
  including standard, boxed, and friction-index cases where the solver natively
  supports them;
- deterministic reduction and update-order rules for any solver that is not
  already row- or problem-parallel by construction;
- benchmark counters that distinguish CUDA build-state rows from direct device
  LCP execution rows;
- default no-GPU build, packaging, and public API boundary checks staying clean;
- `pixi run -e cuda test-cuda` for the focused smoke path and
  `pixi run -e cuda test-all` on Linux CUDA hosts before claiming runtime
  coverage.

This keeps broader LCP CUDA work under Pattern B: useful, high-risk
single-scene contact acceleration. It should not be inferred from the current
Jacobi/PGS/red-black GS batch kernels, and it should not expose backend, stream,
kernel, or device-resource types through DART's public LCP API.

## Multi-Core CPU Options

| Option                 | Maturity / requirement                                          | Packaging                                                     | Verdict for DART                                                                                                                        |
| ---------------------- | --------------------------------------------------------------- | ------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| Taskflow (current)     | Header-only, C++17, active (3.10, 2025)                         | Trivial (header / conda-forge)                                | Keep as the primary backend behind `ComputeExecutor`; no API leakage when internal.                                                     |
| oneTBB                 | Very mature, work-stealing, NUMA-aware                          | conda-forge + wheels, adds runtime shared lib                 | Solid but redundant alongside Taskflow; adopt only for a concrete need.                                                                 |
| OpenMP                 | Ubiquitous, pragma-based                                        | Needs compiler OpenMP runtime; complicates clang/macOS wheels | Reduction order is not fixed by the spec, so reductions are not bitwise-reproducible without costly `ordered`; least boundary-friendly. |
| std::execution (P2300) | Accepted for C++26; `stdexec`/`nvexec` reference impls churning | Not yet shipped by standard libraries                         | The long-term CPU+GPU convergence target under one model; not shippable in a binary library today.                                      |
| Raw std::thread pool   | Zero deps, full control                                         | Trivial                                                       | Reimplements what Taskflow provides; only a minimal fallback.                                                                           |

Recommendation: keep Taskflow as the primary multi-core backend behind the
executor boundary. Watch `std::execution` as the eventual unifier of CPU and GPU
scheduling, and re-evaluate once standard-library vendors ship it. Avoid adding
TBB or OpenMP unless a measured need appears; OpenMP also carries the
reduction-determinism caveat that conflicts with DART's determinism goals.

## GPU Backend Options

| Option                     | Reach                               | Packaging burden                      | Notes                                                                                                                                           |
| -------------------------- | ----------------------------------- | ------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| CUDA kernels               | NVIDIA only                         | Heaviest (per-CUDA-minor binaries)    | Best performance and ecosystem; conda-forge ships per-component CUDA 12+ with a `cuda-version` metapackage; PyPI needs separate wheels/indexes. |
| SYCL — AdaptiveCpp / DPC++ | Cross-vendor (NVIDIA/AMD/Intel/CPU) | Extra LLVM-based toolchain            | "Write once, run anywhere"; AdaptiveCpp's single-pass generic compiler shrinks the build matrix. Strongest cross-vendor bet.                    |
| NVIDIA Warp                | NVIDIA GPU + CPU fallback           | pip (`warp-lang`)                     | Purpose-built for differentiable robotics; Python-first, so it fits dartpy more than the C++ core and would invert the dependency.              |
| Kokkos                     | CUDA/HIP/SYCL/OpenMP/threads        | conda-forge, but per-backend binaries | Abstracts source, not distribution; vendor-neutral kernels from one source.                                                                     |
| HIP / ROCm                 | AMD (hipify to NVIDIA)              | Heavier, Linux-centric                | Secondary target at best.                                                                                                                       |
| Apple Metal                | Apple Silicon only                  | Separate codegen                      | Niche; only path to Apple-GPU compute.                                                                                                          |
| Vulkan compute             | Broadest (incl. mobile, MoltenVK)   | Light runtime, low-level SPIR-V       | High effort, weak HPC tooling; low payoff for numerical robotics.                                                                               |
| LLVM / JIT codegen         | Any                                 | Large LLVM dependency                 | Essentially what Warp/AdaptiveCpp do internally; not justified unless building a bespoke kernel language.                                       |

The CUDA-versus-SYCL choice hinges on one question: is cross-vendor GPU support a
goal? CUDA maximizes performance and ecosystem but locks DART to NVIDIA and the
worst packaging matrix; AdaptiveCpp buys portability and a smaller build matrix
at the cost of a heavier toolchain. Abstraction layers (SYCL, Kokkos, Warp) pay
off only when multi-vendor is a real requirement, and they abstract source, not
distribution — per-backend binaries are still shipped. The full criteria table
lives in [`scalable_compute_decisions.md`](scalable_compute_decisions.md); this
research adds the evidence that, among single-source C++ options, AdaptiveCpp is
the strongest cross-vendor "write-once" path while CUDA remains the strongest
NVIDIA-only path. Both keep device kernels in C++ alongside the existing core,
which is the property that matters for a C++ engine; Python-first kernel
frameworks would invert that dependency.

## Packaging And CI Constraints

GPU binaries do not ship cleanly today on either channel:

- conda-forge dropped the CUDA 11.8 default in May 2025 and splits CUDA 12+ into
  per-component packages coordinated by a `cuda-version` metapackage; this is
  manageable but not free.
- PyPI still requires separate wheels or indexes per CUDA minor (the PyTorch and
  CuPy model). The "Wheel Variants" proposals (PEP 817, PEP 825) aim to automate
  selection but are still draft as of early 2026.

Consequence for DART: GPU support must be an optional, separate component or
package, never a hard dependency of the core wheel or conda package. The core
must keep a working CPU fallback with identical semantics, matching the existing
GPU prototype-gate rules and sidecar package shape in
`scalable_compute_decisions.md`.

## Verdict

This verdict is the durable decision behind the roadmap. Phases 0-5 are complete
and merged; operating priority and the Phase 6 backlog now live in
`docs/plans/dashboard.md` (PLAN-030), and the GPU prototype gate, the recorded
go/no-go, and the CUDA-versus-SYCL criteria live in
`scalable_compute_decisions.md` (the multiphase dev-task folder has been
retired). Scope is the DART 7 World only;
shared backend-neutral utilities (for example `dart/simd/`) may live elsewhere.

Direction (confirmed by a three-perspective review — architecture, plan critique,
and risk analysis):

- **Mature the existing substrate; do not redesign.** The EnTT-ECS World, the
  compute graph, and the injectable `ComputeExecutor` are the right, field-
  convergent foundation. Executor injection is the only public concurrency seam;
  every backend name (Taskflow, CUDA, SYCL, GPU, streams) stays out of the
  public API.
- **Batch (Pattern A) before single-scene GPU (Pattern B).** This matches the
  documented execution-shape ladder (single world → kinematics-only →
  homogeneous batch → rollout). Pattern B (GPU contact/LCP for one large scene)
  is a much-later, higher-risk track.
- **CPU batch is a real GPU prerequisite, not throwaway.** It validates the
  structure-of-arrays layout and throughput at near-zero GPU risk; the GPU path
  becomes a backend swap rather than a rewrite.

Decisions resolved from the repository's own design docs and the evidence above,
so they need no external input:

- **Differentiability stays deferred** (a capability-matrix axis in
  `simulation_cpp_api.md`), not a near-term API. Keep the door open
  cheaply by writing the rewritten dynamics scalar-generic and instantiating only
  `double` — but co-locate that with the state rewrite, not before it.
- **CUDA-versus-SYCL is decided at the Phase 5 benchmark gate**, not now. The GPU
  path stays C++-native (single-source CUDA or SYCL kernels behind the executor
  seam, reusing the CPU data layout); because Taskflow's GPU tasking is CUDA-only,
  the backend stays behind an internal interface so the choice is reversible.
- **`std::execution` is not a near-term dependency** — no standard library ships
  it before roughly 2028, so Taskflow remains the CPU backend; revisit only when
  vendors ship it.
- **Resource-access metadata stays diagnostic first**, layered over explicit
  edges, with hazard validation; automatic scheduling is a later, gated step.

Three corrections the review surfaced, now built into the phase order:

- The **state-representation axis** (immutable Model + batched SoA State) is co-
  equal with execution and is pulled forward, because today's array-of-structs
  access over `entt::registry` exercises the scheduling seam but not the data
  seam that transfers to the GPU.
- **Determinism is enforced first** with an executor-parity test and a written
  EnTT concurrency contract, and its gate is tolerance-based (bitwise for map-
  only stages, fixed-ULP for reductions) because float reductions are not
  bitwise-reproducible.
- **Backends are not chosen on trivial physics.** A contact/constraint-shaped
  benchmark proxy is a prerequisite, because semi-implicit Euler over free bodies
  is embarrassingly parallel and would mislead the backend decision.

## Open Questions And Decision Triggers

These remain genuinely open and should trigger a re-decision when answered:

- Does a profiled, representative (contact-shaped) workload show the GPU full-
  workload time, including transfer, beating the CPU batch by the pre-registered
  threshold? This is the Phase 5 go/no-go for any Pattern B GPU work.
- Is cross-vendor GPU support required, or is NVIDIA-first acceptable? This is the
  deciding input for CUDA versus AdaptiveCpp at the Phase 5 gate.
- Do real downstream users need heterogeneous batches (domain randomization
  beyond state), which would change what the Model/State split must capture?
- Does differentiability get promoted from a deferred capability to a committed
  one? If so, the scalar-generic kernels are instantiated for an autodiff scalar.
- When do standard-library vendors ship `std::execution`? That is the trigger to
  re-evaluate the CPU backend.

## Related Designs

- `docs/plans/dashboard.md` (PLAN-030) owns operating priority and the Phase 6
  backlog now that the multiphase plan's Phases 0-5 are complete and its
  dev-task folder has been retired.
- [`scalable_compute_decisions.md`](scalable_compute_decisions.md) owns the
  workload-first decision rules, multi-core and SIMD constraints, the GPU
  prototype gate, and the CUDA-versus-SYCL criteria.
- [`hierarchical_allocator.md`](hierarchical_allocator.md) covers world-level
  memory ownership, a dependency for batched and accelerator paths.
- `docs/plans/dashboard.md` (PLAN-030) owns scalable-compute operating state and
  the resource-access milestone gate.
- `docs/plans/035-native-collision-dashboard.md` owns the collision-specific
  single-core, multi-core, and GPU sequencing.
- `docs/ai/north-star.md` owns the mission requirement for an evidence-backed
  CPU/GPU roadmap.

## Sources

Physics-engine landscape:

- MuJoCo MJX — <https://mujoco.readthedocs.io/en/stable/mjx.html>
- MuJoCo Warp — <https://mujoco.readthedocs.io/en/stable/mjwarp/index.html>,
  <https://github.com/google-deepmind/mujoco_warp>
- Differentiable simulators overview — <https://simulately.wiki/docs/domain/differentiable/>
- Isaac Lab (2025) — <https://arxiv.org/html/2511.04831v1>; Isaac Gym —
  <https://developer.nvidia.com/blog/introducing-isaac-gym-rl-for-robotics/>
- Brax — <https://arxiv.org/pdf/2106.13281>, <https://pypi.org/project/brax/>
- Genesis — <https://genesis-embodied-ai.github.io/>,
  <https://genesis-world.readthedocs.io/>
- NVIDIA Warp — <https://github.com/NVIDIA/warp>, <https://nvidia.github.io/warp/>
- PhysX 5 open-source (2025) — <https://developer.nvidia.com/blog/open-source-simulation-expands-with-nvidia-physx-5-release/>
- Drake — <https://drake.mit.edu/>, batch eval —
  <https://github.com/RobotLocomotion/drake/issues/16339>
- RaiSim — <https://raisim.com/>; SimBenchmark —
  <https://leggedrobotics.github.io/SimBenchmark/>
- PyBullet GPU status — <https://github.com/bulletphysics/bullet3/issues/3105>
- Taichi / DiffTaichi — <https://github.com/taichi-dev/taichi>,
  <https://arxiv.org/pdf/1910.00935>
- TinyDiffSim — <https://github.com/erwincoumans/tiny-differentiable-simulator>

C++ backend and packaging:

- Taskflow — <https://github.com/taskflow/taskflow>,
  <https://taskflow.github.io/taskflow/release-3-10-0.html>
- oneTBB feedstock — <https://github.com/conda-forge/tbb-feedstock>
- OpenMP reproducibility — <https://arxiv.org/html/2408.05148v1>
- std::execution — <https://github.com/NVIDIA/stdexec>,
  <https://en.cppreference.com/cpp/experimental/execution>
- conda-forge CUDA 11.8 drop — <https://conda-forge.org/news/2025/05/29/cuda-118/>;
  packaging GPUs — <https://pypackaging-native.github.io/key-issues/gpus/>
- AdaptiveCpp — <https://github.com/AdaptiveCpp/AdaptiveCpp>,
  <https://adaptivecpp.github.io/hipsycl/sscp/compiler/generic-sscp/>
- Kokkos — <https://github.com/kokkos/kokkos>
- AMD ROCm — <https://rocm.docs.amd.com/>
- NVIDIA Wheel Variants — <https://developer.nvidia.com/blog/streamline-cuda-accelerated-python-install-and-packaging-workflows-with-wheel-variants/>

Deeper review threads:

- Newton — <https://developer.nvidia.com/blog/announcing-newton-an-open-source-physics-engine-for-robotics-simulation/>,
  <https://github.com/newton-physics/newton>
- ECS auto-scheduling — Bevy
  <https://bevy-cheatbook.github.io/programming/schedules.html>, flecs
  <https://www.flecs.dev/flecs/md_docs_2Systems.html>, Unity Entities
  <https://docs.unity3d.com/Packages/com.unity.entities@1.0/manual/systems-scheduling-jobs.html>
- Scalar-templating — Drake default scalars
  <https://drake.mit.edu/doxygen_cxx/group__default__scalars.html>; Eigen
  AutoDiffScalar packet-SIMD caveat
  <https://libeigen.gitlab.io/eigen/docs-nightly/unsupported/classEigen_1_1AutoDiffScalar.html>
- std::execution status — <https://github.com/bemanproject/execution>,
  <https://gcc.gnu.org/projects/cxx-status.html>
