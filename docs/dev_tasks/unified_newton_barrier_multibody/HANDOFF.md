# Unified Newton-Barrier Handoff

## Historical Stop Directive

On 2026-06-11 the maintainer instructed the agent to stop implementation and
focus only on hand-off, with no further verification. This document captures
the current work so a fresh Claude/Codex session can resume without
reconstructing branch history or local context. No lint, build, test,
benchmark, hosted-CI polling, push, or review-triggering action should be
inferred from this hand-off step after that directive.

## Active Branch And PR

- Branch: `simx/plan083-gpu-contact-candidate-packet`
- PR: #2978, `Advance unified Newton-barrier runtime and parity evidence`
- Base: `main`
- Latest pushed commit before the local point-edge barrier-Hessian slice:
  `6746b63973d Record point-point barrier Hessian packet evidence`
- Local commit not yet pushed:
  `48dcfb515cf Add point-edge barrier Hessian packet parity`
- Review rule: keep all remaining PLAN-083 follow-up work on this branch and
  PR. Do not open or revive per-packet, per-phase, or per-slice PRs.
- Consolidation context: former stack PRs #2979-#2983 were folded into #2978.
  Earlier runtime/corpus follow-ups #2970, #2971, #2974, and #2976 are already
  merged into `main`.

## Latest Continuation State

The point-edge barrier-Hessian slice is complete as a local commit ahead of
`origin/simx/plan083-gpu-contact-candidate-packet`. It has not been pushed.
The follow-on point-triangle barrier-Hessian slice has been completed and
validated locally on the same branch. Depending on where a fresh session starts,
it may appear as this checkpoint commit or as local dirty WIP; inspect
`git status -sb` and `git log --oneline` before editing. It has not been pushed
unless `origin/simx/plan083-gpu-contact-candidate-packet` has moved past
`6746b63973d`.

The current slice adds private `PointTriangleBarrierHessianResult` plumbing, a
CUDA wrapper, kernel/launcher scaffolding, focused CPU/CUDA unit parity
coverage for non-degenerate Hessian fixtures, benchmark rows, packet
writer/tests, and durable sidecar updates. The refreshed packet measured
`point_triangle_barrier_hessian.max_result_abs_error =
2.5887393237583516e-12`,
`point_triangle_barrier_hessian.speedup = 1.701903811937476`, and top-level
`speedup = 0.20658008740799394` with `meets_speedup_gate = false`.

Validation for this checkpoint passed `pixi run lint`, `pixi run build`,
`pixi run test-unit` (161/161), `pixi run -e cuda build-cuda Release`,
focused `test_barrier_friction_kernel_cuda` CTest,
`pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
(4 passed), `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`,
the PLAN-083 GPU parity/completion-audit checker pair, and the focused
packet/audit pytest trio (13 passed).

Before any future commit or push, inspect `git status -sb` and the local diff
first. Before any future push to #2978, merge the latest `origin/main` into
this published branch, run the required gates for the accumulated changes, and
push only with explicit maintainer approval. Keep the work on this same branch;
do not open a new PLAN-083 PR.

## Current Local Packet Slices

The current local packet work extends the private barrier/friction CUDA packet
with point-triangle, point-point, and point-edge primitive barrier-Hessian
evaluators on the same #2978 branch. It builds on the already-pushed
point-triangle primitive barrier-gradient plus point-triangle, edge-edge,
point-edge, and point-point tangent-stencil packet work.

Changed implementation and coverage surfaces in this hand-off package:

- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh` adds private
  point-triangle, point-point, and point-edge barrier-Hessian result/input
  packet contracts.
- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cpp` validates
  those inputs, manages device buffers, and exposes
  `evaluatePointTriangleBarrierHessiansCuda`,
  `evaluatePointPointBarrierHessiansCuda`, and
  `evaluatePointEdgeBarrierHessiansCuda`.
- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cu` adds the CUDA
  kernels and launchers for point-triangle barrier values, gradients, and
  compact 12x12 Hessians; point-point barrier values, gradients, and compact
  6x6 Hessians; and point-edge barrier values, gradients, and compact 9x9
  Hessians.
- `tests/unit/simulation/cuda/test_barrier_friction_kernel_cuda.cpp` adds
  CPU/CUDA parity coverage for the point-triangle, point-point, and point-edge
  barrier-Hessian paths, plus non-finite input rejection.
- `tests/benchmark/simulation/bm_plan083_gpu_barrier_friction.cpp`,
  `scripts/write_plan083_gpu_barrier_friction_packet.py`, and
  `tests/test_plan083_gpu_barrier_friction_packet.py` add benchmark packet and
  writer/test coverage for the new primitive barrier-Hessian rows.

The current code has local-output parity evidence for the private
point-triangle primitive barrier gradient, all four primitive-family tangent
stencils, and now the first three primitive-family barrier Hessian rows. The
overall barrier/friction row remains `in-progress` because broader Hessian
assembly, PSD coupling, runtime contact rows, and the top-level/runtime speedup
gate remain future evidence. Durable plan sidecars now record the
point-triangle, point-point, and point-edge barrier-Hessian packets as
in-progress evidence, not completion.

## Resolved Point-Edge WIP

The previous uncommitted point-edge barrier-Hessian prototype is now
compile-ready and validated locally. It adds `PointEdgeBarrierInput`,
`PointEdgeBarrierHessianResult`, `evaluatePointEdgeBarrierHessiansCuda`, the
CUDA launcher/kernel, parity unit coverage, benchmark rows, packet
writer/tests, and durable sidecar updates. The refreshed packet measured
`point_edge_barrier_hessian.max_result_abs_error =
7.716494110354688e-12`,
`point_edge_barrier_hessian.speedup = 1.278816738709317`, and top-level
`speedup = 0.36913439757406363` with `meets_speedup_gate = false`.

Validation for this checkpoint passed `pixi run lint`, `pixi run build`,
`pixi run test-unit` (161/161), `pixi run -e cuda build-cuda Release`,
focused `test_barrier_friction_kernel_cuda` CTest,
`pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
(4 passed), and `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`.

## Last Evidence Gathered Before Stop

The following evidence had already been gathered before the maintainer's
critical no-more-verification stop request:

- `pixi run lint`
- `pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
  (4 passed)
- `pixi run -e cuda build-cuda Release`
- focused `test_barrier_friction_kernel_cuda` CTest
- `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- focused packet/audit Python tests (13 passed)
- `git diff --check`

The latest measured point-edge/point-point tangent-stencil packet values from
that evidence were:

- `point_edge_tangent_stencil.max_result_abs_error =
1.1268763699945339e-14`
- `point_edge_tangent_stencil.speedup = 1.623508438779482`
- `point_point_tangent_stencil.max_result_abs_error =
3.3306690738754696e-16`
- `point_point_tangent_stencil.speedup = 0.9786534468698057`
- top-level `max_result_abs_error = 3.982848877516439e-14`
- top-level `speedup = 0.9375876271274418`
- top-level `meets_speedup_gate = false`

Additional point-point barrier-Hessian evidence had also been gathered before
the maintainer's critical stop request:

- `pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
  (4 passed)
- `pixi run -e cuda build-cuda Release`
- focused `test_barrier_friction_kernel_cuda` CTest
- `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`

The point-point barrier-Hessian packet measured:

- `point_point_barrier_hessian.sample_count = 65536`
- `point_point_barrier_hessian.active_barrier_count = 59578`
- `point_point_barrier_hessian.max_result_abs_error =
2.4868995751603507e-14`
- `point_point_barrier_hessian.speedup = 2.2036790873726364`
- `point_point_barrier_hessian.meets_speedup_gate = true`
- top-level `max_result_abs_error = 3.982848877516439e-14`
- top-level `speedup = 0.18351053106151646`
- top-level `meets_speedup_gate = false`

No additional validation was run during that stop-only handoff. After
continuation resumed, a fresh packet run measured
`point_point_barrier_hessian.speedup = 1.1162432610528892`,
`point_point_barrier_hessian.meets_speedup_gate = false`, top-level
`speedup = 0.4709699971084976`, and top-level `meets_speedup_gate = false`.
The subsequent point-edge barrier-Hessian continuation passed the packet
pytest, CUDA build, focused `test_barrier_friction_kernel_cuda` CTest, and
barrier/friction benchmark packet. It measured
`point_edge_barrier_hessian.max_result_abs_error =
7.716494110354688e-12`,
`point_edge_barrier_hessian.speedup = 1.278816738709317`, and top-level
`speedup = 0.36913439757406363` with `meets_speedup_gate = false`.

## Resume Guidance

1. Resume only from `simx/plan083-gpu-contact-candidate-packet` and PR #2978.
2. Inspect local status before editing, committing, or pushing. The point-edge
   barrier-Hessian checkpoint is locally committed ahead of origin, while the
   point-triangle barrier-Hessian checkpoint may be either the next local commit
   or dirty WIP depending on where the previous session stopped.
3. Check hosted CI and new review comments before editing. Do not reply to bot
   comments.
4. Continue on the same PR with the remaining runtime/parity gaps: broader
   Hessian assembly, PSD coupling, runtime contact rows, and packet speedup
   gates.
5. Keep plan/dev-task text honest: packet rows may move from `planned` to
   `in-progress` only with corresponding runtime or packet evidence, and the
   dev-task folder should not be retired until the remaining in-progress work
   is either completed or explicitly relocated by a maintainer.
