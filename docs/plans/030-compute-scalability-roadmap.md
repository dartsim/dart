# PLAN-030: Compute Scalability Roadmap

Operating state (priority, status, horizon, north-star dimension, next step, and
gate) lives in [`dashboard.md`](dashboard.md) under `PLAN-030`. The durable
rationale owner for scalable-compute decisions is
[`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md);
this numbered plan owns compute work packets, the retained historical backlog
and progress log rather than duplicating design rationale, per
[`AGENTS.md`](AGENTS.md) and [`../design/AGENTS.md`](../design/AGENTS.md).

## M1 Compute Qualification

[PLAN-040](040-dart7-release-hardening.md) makes compute selection an early
foundation decision. Existing Phase 5 evidence qualifies its recorded
unconstrained workload only. It does not establish full rigid World CUDA
execution, a winning runtime, or zero-allocation graph submission.
[Backend research](../design/compute_backend_research.md) owns candidate
evidence; [compute decisions](../design/scalable_compute_decisions.md) owns
the semantic-graph, executable-plan, adapter, and completion contracts.

These are future implementation packets, not work completed by the planning
PR. Follow [orchestration](../ai/orchestration.md), including recording actual
model/mode, independent author/reviewer ownership, and architecture impact.
No parallel implementation ownership is implied by the research delegation.

### WP-030.1 Benchmark Validity And Runtime Inventory

- Objective/value: measure equivalent execution before choosing a library.
- Scope: executor/pipeline profiling, graph/range dispatch, benchmark fixtures,
  and allocation coverage; reproduce two-pool accounting, missing profiling
  `parallelFor` forwarding, partial caching, and submission-allocation findings.
- Architecture impact: execution order, resource/graph ownership, worker budget, completion,
  precision and allocation; update the compute decisions and assessment.
- Non-goals: runtime adoption or M2 performance promotion.
- Assumptions/open decisions: freeze compiler/library versions, build and
  floating-point flags, hardware, workload sizes, repetitions, and noise policy.
- Acceptance evidence: profiled/unprofiled work and physics agree; measure
  total workers/callers, cold setup, first post-bake allocations, warm
  p50/p95/p99, transfers, memory, idle CPU, and observer overhead separately.
  Include narrow/wide/uneven/nested graphs and existing contact-island work
  with meaningful parallelism. The one-body example alone cannot rank pools.
- Gates: `pixi run lint`, `pixi run test-simulation-quick`, focused compute
  tests, and `pixi run bm-compute-check`; extend existing benchmark tasks when
  the accepted fixture contract requires it.
- Dependencies: accepted WP-040.1.

### WP-030.2 CPU Runtime Comparison

- Objective/value: choose a runtime from DART evidence, not upstream rankings.
- Scope: Taskflow, Dispenso, and oneTBB adapters for the same DART graph/bulk
  contract, a serial reference, and a reproducible benchmark packet.
- Architecture impact: execution order, resource/graph ownership, worker budget, completion,
  precision and allocation; update the compute decisions and assessment.
- Non-goals: multiple default runtimes, GPU kernel portability, physics
  changes, or process-wide concurrency controls from one DART instance.
- Assumptions/open decisions: Taskflow is an incumbent candidate. Screen
  enkiTS/OpenMP; prototype enkiTS if the primary candidates fail allocation or
  latency requirements. Pin versions, licences, platforms, and package routes.
- Acceptance evidence: report single-world latency and concurrent-world
  throughput separately. Hard gates cover correctness, fixed-order reductions,
  isolation, nested execution, total worker budget, exceptions/cancellation,
  draining/reset after failure, and no unexpected scheduling allocations on
  the first post-bake step. Share pools but separate per-state execution
  records when graph instances cannot execute concurrently. Record build,
  integration, code-size, and maintenance costs. Prefer the incumbent on an
  inconclusive tie only if all hard gates pass; otherwise block promotion of
  that parallel path rather than weakening the gates or writing a new runtime.
- Gates: WP-030.1 benchmarks/tests, relevant allocation gates, and
  `pixi run test-all` before production adapter promotion.
- Dependencies: accepted WP-030.1.

### WP-030.3 Kernel Portability And Device Graph Prototype

- Objective/value: test portability benefits before freezing broad kernel work.
- Scope: the same bounded Float64 M1 kernel/data slice using native CPU/CUDA
  and Kokkos Serial/CUDA, DART-owned buffers, streams, and graph replay.
- Architecture impact: execution order, resource/graph ownership, worker budget, completion,
  precision and allocation; update the compute decisions and assessment.
- Non-goals: full M1 physics, another GPU vendor, or runtime-wide conversion.
- Assumptions/open decisions: provisional prototype buffers precede WP-080.1's
  complete model/state design. Screen RAJA, SYCL/AdaptiveCpp, stdexec, StarPU,
  HPX, and Legion for their distinct roles; do not adopt every screened tool.
- Acceptance evidence: identical inputs/formulas pass independent and backend
  oracles; measure persistence/copies, cold compilation, graph creation/replay,
  warm latency, completion, allocation, and duplication. Test views over DART
  allocations and teardown. Record host/device language modes and package,
  driver, architecture, and compiler requirements. A GPU graph wrapper does
  not implement CPU physics on a device.
- Gates: `pixi run test-simulation-quick`, `pixi run -e cuda test-cuda`,
  accepted prototype benchmarks, and CPU-only package isolation checks.
- Dependencies: accepted WP-030.1; no dependency on unfinished WP-080.1.

### WP-030.4 Compute Decision And Graph Contract

- Objective/value: accept one small, replaceable compute design from evidence.
- Scope: design owners, selected adapters, graph/executable lifecycle,
  resource identity/alias checks, and positive/negative conformance tests.
- Architecture impact: execution order, resource/graph ownership, worker budget, completion,
  precision and allocation; update the compute decisions and assessment.
- Non-goals: globally optimal scheduling, multi-GPU/distributed execution,
  unrestricted inference, or changing a solver's numerical update order.
- Assumptions/open decisions: select CPU/runtime and kernel approaches from
  WP-030.2/030.3; retain a serial implementation for small workloads.
- Acceptance evidence: DART-owned semantic/executable/runtime layers; explicit
  physics order; group joins, shared worker budget, independent state/scratch,
  deterministic reductions, and device completion/error semantics. Invalidate
  cached plans on relevant topology/configuration, buffer, capacity, and
  snapshot-binding changes. The bounded inference prototype respects program
  order and RAW/WAR/WAW hazards over states/lanes and aliased ranges; unknown
  access serializes or fails validation. Explicit edges remain the production
  authority until inference is verified. Loop grouping must not turn
  Gauss-Seidel into Jacobi. Record rejected alternatives/revisit triggers.
- Gates: `pixi run lint`, focused graph/lifetime/error/allocation tests,
  `pixi run check-compute-backend-boundaries`, package isolation checks, and
  accepted benchmarks; CUDA gates for selected device adapters.
- Dependencies: accepted WP-030.2, WP-030.3, and WP-040.2.

### WP-030.5 Complete M1 CUDA Rigid Pipeline

- Objective/value: run every M1 rigid example fully on the selected device.
- Scope: resident rigid model/state/control/contact/scratch, sphere/box-plane
  detection, force/torque/rotation, SI friction/restitution, integration, and
  World completion/reporting.
- Architecture impact: execution order, resource/graph ownership, worker budget, completion,
  precision and allocation; update the compute decisions and assessment.
- Non-goals: new shapes/domains, hidden CPU physics fallback, a single-body
  GPU speedup requirement, or asynchronous public World stepping.
- Assumptions/open decisions: Float64 and the accepted M1 variant; active
  contact counts can vary inside fixed topology. An ordered stream is valid
  when graph replay provides no measured benefit.
- Acceptance evidence: all RB rows pass device-to-oracle and CPU/GPU checks;
  record actual stage/backend identity. Dependencies wait for device writes,
  not host launch. Count explicit output/diagnostic/snapshot transfers and
  reject hidden bulk round trips. Propagate async errors; drain before
  reclamation and distinguish recoverable state from invalid device contexts.
  No unexpected host/device allocations after bake, including submission.
- Gates: `pixi run -e cuda test-cuda`, `pixi run -e cuda test-all`, accepted
  RB/compute/allocation tests, and CPU-only package-isolation gates.
- Dependencies: accepted WP-030.4 and WP-080.3.

## Work After M1

M2 adds dynamic contact-island groups, graph reuse across interleaved Worlds,
coarsening, critical-path/locality heuristics, and measured overlap under the
same budget. Compare padded work, bounded graph updates, and device
conditionals on real workloads. A DAG supplies legal order; task, launch,
transfer, residency, and contention costs determine whether scheduling helps.
Device conditional nodes permit loops but not arbitrary host-flow capture.

## Retained Phase 6 Backlog

Historical backlog, now sequenced by PLAN-040. Each admitted item needs its
own design note and gate before work starts: broaden GPU stage coverage
beyond the single rigid-body integration stage; promote auto-scheduling from
resource-access metadata behind a verified scheduler contract (honest
declarations, deferred structural changes, deterministic reductions, cost
gate); heterogeneous batches and single-scene contact/constraint GPU work
(Pattern B; M1 now requires the bounded single-body path, while M2 measures
many-body scaling independently of Pattern A), including any PD-IPC
GPU contact path tracked under
[`081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md`](081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md);
and differentiable state types if differentiability is promoted from a
deferred to a committed capability. Rationale for each lives in
[`../design/compute_backend_research.md`](../design/compute_backend_research.md).

## Progress log

Relocated from the dashboard on 2026-07-03; newest first.

Phases 0-5 are complete and merged to `main` (PRs #2698, #2710,
#2712); the dev-task folder has been retired, so PLAN-030 plus
[`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md)
are now the durable trackers. The default DART 7 `World::step` path
preserves the rigid-body contact/multibody solver pipeline, while the batched
SoA rigid-body stage remains an explicit unconstrained path and
benchmark/prototype seam. Phase 5 is closed with a GO: `CI CUDA / CUDA Build`
compiles the CUDA targets for fork PRs on the hosted fallback and runs CUDA
tests on the trusted `ubuntu-latest-gpu` runner for same-repository PRs,
protected branch pushes, and manual dispatches. The go/no-go runtime packet is
still a measured benchmark packet from a CUDA host. The recorded GO
(2026-05-28, RTX 5000 Ada): speedup 109.6x at 4096/128/100 with final-state
error 1.78e-15, packet accepted (see the owner doc's "Recorded Phase 5
Go/No-Go"). That historical prototype kept CUDA private and non-required. The accepted
M1 contract now requires complete CPU/CUDA example evidence while keeping
CUDA optional to install and runtime types private. The sidecar package shape,
go/no-go threshold, `bm-phase5-gpu-packet-check` /
`check-compute-backend-boundaries` / `check-no-gpu-runtime-dependencies`
evidence gates, and the `check-phase5-cuda-benchmark-contract` row contract
are recorded in the owner doc. To refresh the packet on any CUDA host, run
`bm-phase5-cuda-full` then `bm-phase5-cuda-packet`;
`check-phase5-cuda-workflow` guards that `ci_cuda.yml` keeps fork PRs on the
hosted compile fallback and restricts GPU-runtime steps to trusted events.
Phase 3's speedup surface is the checked contact-island benchmark, not the
trivial Euler rigid-body rows.
