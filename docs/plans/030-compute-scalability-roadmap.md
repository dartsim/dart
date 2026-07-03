# PLAN-030: Compute Scalability Roadmap

Operating state (priority, status, horizon, north-star dimension, next step, and
gate) lives in [`dashboard.md`](dashboard.md) under `PLAN-030`. The durable
rationale owner for scalable-compute decisions is
[`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md);
this numbered plan file exists only to hold PLAN-030 roadmap state (the Phase 6
backlog and progress log) that must not live in the design doc, per
[`AGENTS.md`](AGENTS.md) and [`../design/AGENTS.md`](../design/AGENTS.md).

## Phase 6 backlog

(unblocked by the Phase 5 GO, unstarted; each item needs its
own design note and gate before work starts): broaden GPU stage coverage
beyond the single rigid-body integration stage; promote auto-scheduling from
resource-access metadata behind a verified scheduler contract (honest
declarations, deferred structural changes, deterministic reductions, cost
gate); heterogeneous batches and single-scene contact/constraint GPU work
(Pattern B, only after Pattern A evidence justifies it), including any PD-IPC
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
Go/No-Go"). Keep CUDA private and non-required. The sidecar package shape,
go/no-go threshold, `bm-phase5-gpu-packet-check` /
`check-compute-backend-boundaries` / `check-no-gpu-runtime-dependencies`
evidence gates, and the `check-phase5-cuda-benchmark-contract` row contract
are recorded in the owner doc. To refresh the packet on any CUDA host, run
`bm-phase5-cuda-full` then `bm-phase5-cuda-packet`;
`check-phase5-cuda-workflow` guards that `ci_cuda.yml` keeps fork PRs on the
hosted compile fallback and restricts GPU-runtime steps to trusted events.
Phase 3's speedup surface is the checked contact-island benchmark, not the
trivial Euler rigid-body rows.
