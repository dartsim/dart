# PLAN-030: Compute Scalability Roadmap

- Operating state: `PLAN-030` in `docs/plans/dashboard.md`

## Outcome

DART has a computation-graph-backed simulation pipeline in the experimental
World that can run on multi-core CPU first, expose swappable research pipeline
stages, and later admit SIMD or GPU kernels without committing public GPU APIs
too early.

## Scope

This plan covers how DART should decide and sequence scalable-compute work:

- computation-graph structure for the overall simulation step;
- multi-core CPU algorithm structure;
- executor backend selection, starting with a Taskflow evaluation;
- swappable pipeline-stage contracts for research algorithms;
- SIMD-friendly data paths;
- benchmark-backed GPU candidate workloads;
- CUDA package constraints;
- SYCL-versus-CUDA decision criteria;
- public API boundaries for accelerated algorithms.

Out of scope for this plan:

- Committing to a GPU backend before workload evidence exists.
- Rewriting DART around one compute backend.
- Changing the stable classic `dart::simulation::World::step()` contract during
  DART 7.
- Exposing Taskflow, thread pools, streams, or device objects as stable public
  API before the experimental contracts prove useful.
- Treating backend availability as success without research workload evidence.

## Current Evidence

- `dart/simd/` provides explicit SIMD types and geometry helpers.
- `tests/benchmark/simd/` provides SIMD benchmark coverage.
- `docs/design/hierarchical_allocator.md` documents allocator direction for
  cache-friendly simulation.
- `docs/onboarding/build-system.md` documents build options and dependency
  structure.
- `.github/workflows/ci_simd.yml` validates SIMD-oriented CI coverage.
- Current `dart::simulation::World::step()` is a serial classic-world pipeline:
  forward dynamics per mobile Skeleton, global constraint solve, position
  integration, time/frame advance, then sensors. Its constructor still creates a
  `BoxedLcpConstraintSolver` for gz-physics compatibility.
- Current `dart::simulation::experimental::World` is ECS-backed and opt-in, but
  on `main` it only has design/simulation mode, kinematic cache updates,
  serialization, and entity wrappers. Its CMake component already lists
  Taskflow as an experimental dependency.
- Remote branch `origin/feature/multi_core` contains a classic-World
  `compute_graph` module, Taskflow executor, `WorldStepGraph`, tests, and
  benchmarks. Its own progress doc reports that per-skeleton graph overhead
  dominates small workloads and batching is the key to benefit for 32+
  Skeletons.
- Remote branch `origin/feature/sim_exp` contains a broader experimental-World
  prototype with `simulation/experimental/compute`, Taskflow and sequential
  executors, experimental dynamics, benchmarks, and classic-adapter parity
  scaffolding, but that work is not on `main`.
- Remote branch `origin/constraint_solver` shows earlier constraint-group
  parallelization work that should inform the constraint-island stage.
- The first implementation milestone adds the graph core, sequential and
  Taskflow executors, a graph-built experimental World kinematics stage exposed
  through the current `World::step()` slice, graph-backed rigid-body
  linear-force integration, a swappable `WorldStepStage` contract, a non-owning
  `WorldStepPipeline` for composing multiple experimental solver stages, opt-in
  execution profiles for per-node runtime load and observed parallelism, DOT
  graph visualization, focused tests, and the first batching benchmark
  workloads.
- `docs/dev_tasks/compute_resource_access/` tracks the next implementation
  milestone: descriptive resource read/write metadata for compute nodes,
  validation, and visualization before dependency inference or contact-stage
  scheduling.

## Workload Candidates

Rank candidate workloads before choosing backend architecture:

| Workload                                   | Why it matters                                       | Early evidence needed                               |
| ------------------------------------------ | ---------------------------------------------------- | --------------------------------------------------- |
| Whole simulation pipeline graph            | Coordinates all other CPU/SIMD/GPU work              | Stage contracts, parity tests, scheduling overhead  |
| Batched collision and distance queries     | Common in planning, RL, and dataset generation       | Shapes, batch sizes, baseline CPU timing            |
| Batched dynamics                           | Important for ensembles, optimization, and learning  | Model sizes, independence assumptions, state layout |
| Large model ensembles                      | Research workloads often run many worlds or rollouts | Memory layout, scheduler behavior, reproducibility  |
| Differentiable or gradient-heavy workloads | Relevant to optimization and learning                | API expectations, precision requirements            |
| Contact/constraint solving at scale        | Often dominates hard simulations                     | Solver structure, sparsity, data transfer costs     |

## Decision Framework

### Multi-Core CPU First

Before public GPU commitments:

- use `dart::simulation::experimental::World` as the graph owner and keep the
  classic World as the compatibility baseline and parity oracle;
- identify parallel units that do not break deterministic behavior: independent
  worlds or rollouts, independent multibodies before contact, collision pair
  batches, and constraint islands;
- benchmark thread scheduling overhead;
- batch small work units when per-node scheduling overhead dominates;
- preserve single-threaded correctness tests;
- avoid exposing thread pool internals as public API.

### Computation Graph Pipeline

The north-star simulation step is a typed DAG built from experimental ECS state:

```text
dirty-state/version scan
  -> kinematics/cache update
  -> force, actuator, and controller evaluation
  -> unconstrained rigid-body integration batches
  -> unconstrained dynamics batches
  -> broadphase/narrowphase collision batches
  -> constraint island build
  -> constraint solve per independent island
  -> impulse application and integration
  -> sensors, recording, time, and frame advance
```

Each node or stage should declare:

- read, write, and cache component sets;
- broad compute domain, such as rigid-body, articulated-body, deformable-body,
  fluid, collision, constraint, control, sensor, rendering, or custom work;
- acceleration opportunities, such as task parallelism, data parallelism, SIMD,
  data locality, or GPU candidacy;
- deterministic ordering and reduction rules;
- scratch-memory requirements through DART allocators;
- invalidation/version inputs for graph rebuilds;
- profiling counters and benchmark labels;
- the sequential reference operation it must match.

The graph interface should stay backend-neutral. A sequential executor is the
reference and debugging path. Taskflow is the first practical parallel executor
because it maps directly to DAG scheduling and is already present in
experimental dependency metadata. Alternatives remain open: TBB is a candidate
if work stealing or ecosystem support proves stronger, OpenMP is too coarse for
the full DAG but can still be useful inside kernels, and a custom scheduler
should wait until benchmarks show DART-specific needs that Taskflow cannot
cover.

The swappable research surface belongs at stage boundaries, not inside scheduler
internals. `PLAN-020` should supply algorithm contracts for stages such as
constraint solving, collision/distance queries, integrators, and dynamics
kernels. `PLAN-030` owns how those stages are scheduled, batched, benchmarked,
and eventually mapped to SIMD or GPU backends.

The graph should also remain suitable for non-physics consumers that need the
same dependency and profiling model, especially long-term rendering-prep work.
Rendering should be represented as a compute domain or downstream consumer until
there is workload evidence for a real rendering pipeline API.

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

### 1. Branch Evidence Reconciliation

Harvest the useful parts of the remote WIP branches before starting new code:

- keep `origin/feature/multi_core` benchmark and batching lessons;
- prefer the experimental namespace direction from `origin/feature/sim_exp`;
- do not port classic `WorldConfig` graph toggles into stable DART 7 APIs;
- record which tests and benchmarks should be revived in a new dev task.

### 2. Experimental Graph Core

Land the smallest experimental graph substrate:

- `ComputeGraph` and deterministic topological ordering;
- sequential executor as the reference;
- Taskflow executor behind an experimental backend interface;
- graph validation and cycle diagnostics;
- tests that do not require full physics.

### 3. Simulation Stage Graph

Build the first real pipeline in `dart::simulation::experimental::World`:

- kinematics and cache invalidation;
- unconstrained dynamics for independent multibodies or rigid bodies;
- integration and time/frame advance;
- parity checks against classic World where the model can be represented in
  both APIs.

### 4. Resource Access Metadata

Add data-access declarations before contact or GPU scheduling:

- read, write, read-write, reduction, and scratch access modes;
- simple resource identifiers first, with typed ECS/component IDs deferred
  until the string model proves insufficient;
- conservative validation for obvious hazards;
- DOT/resource visualization for graph reviews;
- no replacement of explicit dependencies in the first metadata milestone.

This layer should make compile-time and runtime graph optimization possible
later, but the first PR should treat it as diagnostic metadata and validation
input.

### 5. Collision And Constraint Islands

Add contact stages only after the unconstrained path is measurable:

- collision pair batching;
- constraint island construction;
- per-island solve scheduling when islands are independent;
- deterministic ordering for contacts, islands, impulses, and reductions.

### 6. Swappable Research Stages

Define stage-level algorithm selection with `PLAN-020`:

- stage contracts are stable enough for experiments;
- algorithms can be registered or selected without exposing scheduler internals;
- benchmark harnesses compare an experimental algorithm against DART baselines.

The first concrete contract is `compute::WorldStepStage`, currently exercised by
the default `compute::RigidBodyIntegrationStage`, `compute::KinematicsStage`, a
multi-stage `compute::WorldStepPipeline`, and custom test stages covering
articulated-body, deformable-body, fluid, and rendering-prep domains. Future
collision and constraint stages should use the same pattern rather than exposing
executor internals through `World`.

### 7. SIMD And GPU Candidates

Only after CPU graph evidence exists:

- choose SIMD-friendly data layouts for hot stages;
- prototype GPU nodes internally for one ranked workload;
- include transfer, setup, and package costs in every GPU comparison.

### 8. Workload Ranking

Collect candidate workloads from examples, benchmarks, issue/PR evidence, and
research use cases. Rank by:

- user value;
- expected performance bottleneck;
- data movement cost;
- testability;
- package/CI feasibility;
- API impact;
- whether the workload exercises stage-level modularity, not just raw backend
  throughput.

### 9. Benchmark Gate Design

Define benchmark gates before implementation:

- baseline CPU implementation;
- sequential graph versus direct sequential baseline;
- Taskflow graph versus sequential graph at 1, 2, 4, 8, and hardware-thread
  worker counts where feasible;
- batched versus per-entity node granularity;
- scalar versus SIMD comparison where relevant;
- multi-core CPU scaling curve;
- GPU prototype comparison only when transfer and setup costs are included.

### 10. API Boundary Design

Acceleration should normally be selected through high-level algorithm options
or backend policies. Avoid exposing raw kernel, stream, memory pool, or device
types unless there is a clear long-term public contract. The first public
boundary should remain experimental and should describe pipeline stages and
policies, not Taskflow implementation objects.

The user-facing API should stay smaller than the internal/developer extension
surface. Default stepping should remain simple, while advanced experiments use
experimental-only hooks such as `WorldStepPipeline`, `ComputeStageMetadata`,
execution profiles, and DOT export. Do not introduce a stable solver registry,
GPU runtime object, SIMD storage contract, or rendering backend until benchmark
and workload evidence justifies it.

### 11. Packaging And CI Constraints

Before public GPU support:

- decide which package paths can ship it;
- define fallback behavior when GPU support is unavailable;
- define CI coverage for build, import, and at least one smoke benchmark;
- record unsupported platforms explicitly.

For the first CPU graph milestone, there is no new package path:
`dart-simulation-experimental` owns the headers and sources, Taskflow remains an
experimental component dependency, and CI coverage is the existing
`simulation-experimental` unit-test label plus the standalone
`bm_compute_graph` benchmark target.

## Acceptance Criteria

Active implementation is justified when:

- the next dev task explicitly targets `dart::simulation::experimental::World`,
  not the stable classic World;
- useful branch evidence is harvested from `origin/feature/multi_core`,
  `origin/feature/sim_exp`, and `origin/constraint_solver`;
- the first graph milestone names its stage contracts, sequential parity gate,
  Taskflow-vs-alternatives decision, and benchmark workload;
- CPU/SIMD prerequisites are listed, while CUDA/SYCL criteria remain tied to
  future workload evidence.

This plan is complete for the first compute milestone when:

- experimental World has a graph-built step path with sequential and Taskflow
  executors;
- the first stage after the kinematics-only milestone performs rigid-body
  linear-force integration through the same executor boundary;
- classic World behavior and ABI-sensitive compatibility remain unchanged;
- parity tests compare the graph path with the sequential reference for at
  least one non-trivial model;
- one workload has sequential, multi-core, and batching benchmark evidence;
- at least one stage can swap an algorithm behind a documented experimental
  contract;
- the graph and world pipeline can represent multiple solver domains, including
  articulated-body, deformable-body, fluid, and rendering-prep stages, without
  hard-coding rigid-body-only assumptions into the graph core;
- stage and node metadata can describe parallelism, SIMD/data-locality, and GPU
  opportunities without exposing backend-specific public API;
- profiled execution can report static DAG levels, per-node compute load,
  worker assignment, critical path time, and observed parallelism for debugging
  and future runtime graph-shaping heuristics;
- graph visualization can be exported from the same graph/profile data used by
  tests and benchmarks;
- the next active milestone declares resource read/write metadata for
  validation and visualization before using it for dependency inference or
  scheduling;
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
- gz-physics or other downstream compatibility evidence changes the classic
  World boundary;
- CUDA or SYCL ecosystem constraints materially change.
