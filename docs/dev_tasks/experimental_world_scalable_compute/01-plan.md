# Experimental World Scalable Compute — Multiphase Plan

This is the detailed phase plan. `README.md` owns status and the short
checklist; this file owns per-phase deliverables, exit criteria, and rationale.
Durable decision rationale and the option survey live in
`docs/design/compute_backend_research.md` and
`docs/design/scalable_compute_decisions.md`. Operating priority lives in
`docs/plans/dashboard.md` (PLAN-030).

## Scope

All implementation targets `dart::simulation::experimental` (the EnTT-ECS
`World`, the `compute` graph/executor substrate, and `comps`). Shared, reusable
code may land outside the folder (for example `dart/simd/` SIMD kernels or a
small `dart/common/` utility) when it is genuinely backend-neutral and useful to
other modules. The classic `dart::simulation::World` stays untouched.

## Verdict This Plan Executes

Mature the existing ECS + compute-graph + injectable `ComputeExecutor` substrate
along the documented execution-shape ladder (single world → kinematics-only →
homogeneous batch → rollout), rather than building a new parallel framework or
touching the classic World. Keep Taskflow behind the executor seam, keep every
backend name out of the public API, treat CPU batch as a real prerequisite for
GPU Pattern A, and defer the CUDA-versus-SYCL choice to a benchmarked prototype.

Three corrections came out of multi-agent review and are baked into the phase
order below:

1. The **state-representation axis** (immutable Model + batched structure-of-
   arrays State) is co-equal with the execution axis, not a late deferral. The
   current array-of-structs access over `entt::registry` exercises the scheduling
   seam but not the data seam that actually transfers to the GPU, so the SoA
   decision is pulled forward (Phase 2).
2. **Determinism must be enforced, not assumed.** There is no parity test today
   and parallel nodes mutate the shared registry. A parity test plus a written
   EnTT concurrency contract come first (Phase 0), and the determinism gate is
   tolerance-based, because floating-point non-associativity makes bitwise
   `parallel == sequential` impossible for reductions.
3. **Do not benchmark or pick a backend on today's trivial physics.** Semi-
   implicit Euler over free bodies is embarrassingly parallel and will mislead a
   backend decision once articulated-body dynamics and an iterative contact
   solver land. A contact/constraint-shaped benchmark proxy is a Phase 0
   prerequisite, not a Phase 5 deliverable.

## Current Baseline

- `World` owns one `entt::registry`; public handles are thin views; `step()`
  takes an injectable `compute::ComputeExecutor`.
- `compute` has `ComputeGraph` (DAG + explicit edges), `SequentialExecutor`
  (reference) and `ParallelExecutor` (Taskflow), execution profiling, DOT, and
  stage metadata with domain + acceleration flags (a `Gpu` flag is reserved).
- Physics is early: kinematics (frame-tree world transforms) and semi-implicit
  Euler rigid-body integration only. No ABA, collision, or constraint solver in
  the experimental World yet.
- `ComputeNode::ExecuteFn` is a host `std::function<void()>`; it cannot cross to
  a device. The shared substrate is the graph and metadata, not the closures.
- No determinism parity test and no experimental benchmark corpus exist yet;
  resource-access metadata is not implemented (`ComputeStageMetadata` has only
  domain + acceleration; `ComputeGraph::validate()` checks cycles/ownership).

## Cross-Cutting Invariants

These hold for every phase:

- Synchronous `World::step()` stays deterministic and unchanged in semantics;
  sequential execution is the reference path.
- Executor injection through the abstract `ComputeExecutor` is the only public
  concurrency seam. No `entt`, `comps`, thread-pool, GPU device, stream, kernel,
  memory-pool, or solver-registry type enters the public API.
- The classic `dart::simulation::World` is not modified.
- Every phase exit cites a checked-in benchmark baseline through the `bm` /
  `bm-check` tasks, not a vague "benchmark green."
- When dartpy gains a parallel or batched step, it releases the GIL around the
  step and forbids Python callbacks inside compute nodes.

## Phases

### Phase 0 — Foundations and invariants

Cheap, high-leverage groundwork that unblocks every later gate.

Deliverables:

- An executor-parity test asserting `ParallelExecutor` and `SequentialExecutor`
  produce the same world state after N steps on a multi-body scene, and that the
  two `RigidBodyIntegrationStage` topologies (per-entity DAG versus batched)
  agree on the same scene.
- A written EnTT concurrency contract (no structural changes during execute; two
  concurrent nodes never write the same component instance; explicit edges
  encode every cross-node data dependency) plus debug assertions that enforce it.
- A benchmark corpus and harness for the experimental World, including at least
  one contact/constraint-shaped synthetic workload proxy (sequential coupling,
  irregular memory access), wired to committed `bm-check` baselines.
- A recorded decision on the resource identity model for access metadata
  (component-type versus per-entity versus per-archetype) and a thin
  Model/State interface shape, even though the implementation lands in Phase 2.

Exit criteria:

- Parity test green; the two integration topologies are proven to agree.
- Concurrency contract documented and debug-asserted.
- Benchmark corpus and baselines committed; the contact-shaped proxy runs.
- Resource-model and Model/State interface decisions recorded in this folder.

### Phase 1 — Resource-access metadata (diagnostic)

The PLAN-030 milestone, now correctly sequenced after the resource model is
defined. The objective-specific contract and evaluator for this milestone are
`docs/plans/030-compute-resource-access/mission.md` and `evaluator.md`; this
section is the implementation tracking that previously lived in the separate
`compute_resource_access` dev task (now consolidated here).

Access model:

- Access modes are `Read` (reads without mutating), `Write` (overwrites without
  depending on the old value), `ReadWrite`/mutation (reads and mutates in place),
  `Reduce` (multiple nodes contribute to a deterministic reduction), and
  `Scratch` (node-local temporary, no inter-node meaning).
- Resource identifiers start as stable strings (for example `comps::Transform`,
  `comps::Velocity`, `comps::Force`, `comps::FrameCache`); typed component IDs
  follow only if the string model proves too weak. This is the resource identity
  decision recorded in Phase 0.

Dependency model (hybrid, conservative first):

- Explicit graph edges remain the correctness source of truth; access metadata
  validates and visualizes data hazards but does not infer dependencies yet.
- Inference is deferred because not every dependency is a data hazard: solver
  iteration order, deterministic reductions, frame-hierarchy order, and external
  side effects can require an explicit edge even when accessed resources look
  compatible.

Deliverables:

- The access-mode enum and a small resource-access value type attached to compute
  nodes; stage builders pass useful defaults for kinematics and rigid-body
  integration.
- Conservative validation: read/read sharing is allowed; writes to disjoint
  resources are allowed; same-resource write/read, read/write, write/write, and
  read-write need an explicit edge or a declared reduction; scratch creates no
  hazard; reductions are allowed only when the reduction is explicitly declared.
- DOT output surfaces access (optional for large graphs); a debug access-tracking
  check traps undeclared reads/writes (write-back honesty); profiles can correlate
  hot nodes with hot resources.

Exit criteria:

- The evaluator at `docs/plans/030-compute-resource-access/evaluator.md` passes:
  validator flags same-resource hazards without an edge or a declared reduction,
  access-honesty debug check passes, sharing/conflict/reduction/scratch cases have
  focused tests.
- Graph/world unit tests and the compute-graph benchmark stay green.
- Metadata stays diagnostic; no automatic scheduling yet; classic World
  untouched.

### Phase 2 — State representation: immutable Model + batched SoA State

The data seam that actually transfers to multi-core, SIMD, and GPU. Single-world
first (batch dimension defaults to one).

Deliverables:

- DART-owned state/control owner value types and an SoA/tensor-friendly State
  buffer with a leading world dimension (n = 1 initially). Public handles stay
  thin views; `entt` stays internal.
- Co-located scalar-generic dynamics: introduce `template <typename Scalar>` on
  the rewritten kernels but instantiate only `double`. This rides along with the
  SoA rewrite (same kernels) instead of being a separate SIMD-phase change, and
  keeps the autodiff and batch-SIMD doors open without committing to them.

Exit criteria:

- Single-world step over SoA State matches prior registry-based results within a
  stated tolerance; no public API change.
- The `double` path stays Eigen-vectorizable; only `double` is instantiated.
- API-boundary check shows no new backend or storage type leakage.

### Phase 3 — Multi-core hardening, SIMD, and data locality

Now that the SoA kernels exist, make parallel execution fast and provably
deterministic.

Deliverables:

- Tolerance-based determinism gate: bitwise-identical for map-only
  (embarrassingly parallel) stages; fixed-ULP/relative tolerance for reductions,
  with reductions implemented as per-thread partials merged in a fixed order.
- A cost gate so sub-threshold nodes run inline; coarsen node granularity and
  avoid per-step Taskflow rebuild where profiling shows scheduling overhead
  dominates.
- Done (early): `ComputeGraph::buildTopologicalOrder` was O(N^2) (it rescanned all
  nodes for each emitted node, with O(N) index lookups per edge). Phase 0
  benchmarking found this dominated `SequentialExecutor` on large graphs
  (hundreds of ms for ~16k nodes), which would distort sequential-versus-parallel
  comparisons. Replaced with Kahn's algorithm using a min-heap on node index
  (O((N+E) log N)) that preserves the construction-order tie-break.
- SIMD (via `dart/simd`) and cache-friendly iteration on the SoA integration and
  kinematics kernels, with scalar fallback preserved.
- A Bevy-style ambiguity detector for unordered write conflicts, added once a
  second write-conflicting stage exists (defer until there is real surface).

Exit criteria:

- Determinism tests pass for workers = 1..N within the stated tolerance.
- SIMD-versus-scalar and parallel-versus-sequential benchmarks beat the Phase 0
  baseline with scheduling overhead reported; no `bm-check` regression.

### Phase 4 — Homogeneous batch (CPU) and rollout

Pattern A on the CPU: validates the batched layout and throughput with zero GPU
risk.

Deliverables:

- Batched State with n > 1; a CPU batch executor that runs N homogeneous worlds
  via Taskflow. This proves layout and throughput only; heterogeneous topology
  (domain randomization beyond state) is deferred to Phase 6.
- A rollout API separate from live stepping (model + initial state + control
  sequence + output buffers + step count), deterministic, with no device or
  stream types.

Exit criteria:

- Batched step matches N sequential single-world steps within tolerance; rollout
  matches a loop of steps.
- Throughput scaling benchmark versus baseline; no `entt` or GPU type in the
  public API.

### Phase 5 — GPU prototype behind a gate with a kill criterion

Internal prototype and benchmark report only. No public GPU API.

Deliverables:

- A data-describable GPU execution backend behind `ComputeExecutor` (the graph
  and metadata are shared; the host `ExecuteFn` closures are not, so the GPU path
  is a separate execution model, not a reuse of the node closures).
- Mirror batched-State SoA buffers to device; implement one stage (rigid-body
  integration over a homogeneous batch) as a kernel; measure the full workload
  (transfer + setup + compute) against the CPU batch on the representative
  corpus; provide an identical-semantics CPU fallback; add build/import CI and
  one smoke benchmark.
- The MVP implementation is intentionally narrower than the final Phase 5
  backend: it adds a private build-tree CUDA wrapper for the linear
  `RigidBodyStateBatch` SoA integration path, a parity test, a smoke benchmark,
  the `pixi run -e cuda test-cuda` command, and a manual `ci_cuda.yml` workflow.
  It does not expose a public `CudaExecutor`, CUDA stream/device types, or any
  CUDA names in installed headers.
- Decide CUDA versus SYCL/AdaptiveCpp from the measured results, applying the
  criteria in `scalable_compute_decisions.md`. Because Taskflow's GPU tasking is
  CUDA-only, the backend stays behind an internal interface so the choice is
  reversible.

Kill criterion:

- If the GPU full-workload time including transfer does not beat the CPU batch by
  a pre-registered threshold at batch size B on the representative workload, the
  Phase 6 GPU track is cut or parked. A gate without a failing branch is a
  rabbit hole.

Exit criteria:

- Benchmark report with an explicit go/no-go; build/import gate green; API-
  boundary review shows no backend leakage.

External dependency: GPU CI requires a GPU runner labeled for CUDA. Treat runner
provisioning and the optional/separate GPU package as named prerequisites with
an owner, not as hidden requirements of ordinary PR CI.

### Phase 6 — Reassess (gated by Phase 5 evidence)

Only if Phase 5 passes its kill criterion and workload evidence justifies it:

- Broaden GPU stage coverage.
- Promote auto-scheduling from resource-access metadata behind an intentional,
  verified scheduler contract (with the ECS guardrails: honest declarations,
  deferred structural changes, deterministic reductions, cost gate).
- Heterogeneous batch and single-scene contact/constraint GPU work (Pattern B).
- Differentiable state types, if differentiability is promoted from a deferred
  capability to a committed one.

Each item needs its own design note and gate.

## Dependencies Between Phases

- Phase 1 consumes the Phase 0 resource-model decision.
- Phases 3-5 consume the Phase 2 SoA State; if the Phase 2 layout is wrong,
  later phases inherit it, so Phase 2 is a blocking handoff.
- Phase 5 reuses the Phase 4 batched State and the representative corpus from
  Phase 0; it does not reuse the host node closures.

## Biggest Risk and Cheapest De-risk

The biggest derailment risk is choosing a GPU backend on perf evidence from
trivial, solver-free physics that will not generalize once articulated-body
dynamics and contact solving land. The cheapest de-risk is entirely in Phase 0:
the determinism parity test, the EnTT concurrency contract, and one contact-
shaped benchmark proxy. Those three are days of work, unblock every later gate,
and prevent an expensive, hard-to-reverse backend mistake.
