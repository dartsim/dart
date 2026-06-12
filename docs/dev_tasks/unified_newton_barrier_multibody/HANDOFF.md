# Unified Newton-Barrier Handoff

## Historical Stop Directive

On 2026-06-11 the maintainer instructed the agent to stop implementation and
focus only on hand-off, with no further verification. This document captures
the current work so a fresh Claude/Codex session can resume without
reconstructing branch history or local context. No lint, build, test,
benchmark, hosted-CI polling, or review-triggering action should be inferred
from this hand-off step after that directive.

## Active Branch And PR

- Branch: `simx/plan083-gpu-contact-candidate-packet`
- PR: #2978, `Advance unified Newton-barrier runtime and parity evidence`
- Base: `main`
- Latest pushed commit before the point-edge barrier-Hessian slice:
  `6746b63973d Record point-point barrier Hessian packet evidence`
- Review rule: keep all remaining PLAN-083 follow-up work on this branch and
  PR. Do not open or revive per-packet, per-phase, or per-slice PRs.
- Consolidation context: former stack PRs #2979-#2983 were folded into #2978.
  Earlier runtime/corpus follow-ups #2970, #2971, #2974, and #2976 are already
  merged into `main`.

## Continuation State

The point-edge barrier-Hessian slice is the next atomic checkpoint on #2978. A
fresh session may see it either as dirty local changes or as a local commit
ahead of `origin/simx/plan083-gpu-contact-candidate-packet`, depending on where
the previous session stopped. Inspect `git status -sb` and `git log --oneline`
as authoritative before editing, committing, or pushing.

Expected touched surfaces for this checkpoint:

- modified implementation files:
  `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cpp`,
  `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cu`,
  `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh`
- modified tests/benchmarks/scripts:
  `tests/unit/simulation/cuda/test_barrier_friction_kernel_cuda.cpp`,
  `tests/benchmark/simulation/bm_plan083_gpu_barrier_friction.cpp`,
  `tests/test_plan083_gpu_barrier_friction_packet.py`,
  `scripts/write_plan083_gpu_barrier_friction_packet.py`
- modified plan/dev-task sidecars:
  `docs/dev_tasks/unified_newton_barrier_multibody/HANDOFF.md`,
  `docs/dev_tasks/unified_newton_barrier_multibody/README.md`,
  `docs/dev_tasks/unified_newton_barrier_multibody/RESUME.md`,
  `docs/plans/083-unified-newton-barrier-multibody/completion-audit.md`,
  `docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json`,
  `docs/plans/083-unified-newton-barrier-multibody/paper-deck-manifest.md`,
  `docs/plans/dashboard.md`

Before committing this checkpoint, run the required lint/build/test gates. A
suitable checkpoint commit message is
`Add point-edge barrier Hessian packet parity`. Before pushing a published PR
branch, merge latest `origin/main` into the branch and push only with explicit
maintainer approval.

## Current Local Slice

The current slice extends the private barrier/friction CUDA packet with
point-point and point-edge primitive barrier-Hessian evaluators on the same
#2978 branch. It builds on the already-pushed point-triangle primitive
barrier-gradient plus point-triangle, edge-edge, point-edge, and point-point
tangent-stencil packet work.

Changed implementation and coverage surfaces in this hand-off package:

- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh` adds private
  point-point and point-edge barrier-Hessian input/result packet contracts.
- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cpp` validates
  those inputs, manages device buffers, and exposes
  `evaluatePointPointBarrierHessiansCuda` and
  `evaluatePointEdgeBarrierHessiansCuda`.
- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cu` adds the CUDA
  kernels and launchers for point-point barrier values, gradients, and compact
  6x6 Hessians plus point-edge barrier values, gradients, and compact 9x9
  Hessians.
- `tests/unit/simulation/cuda/test_barrier_friction_kernel_cuda.cpp` adds
  fixed and generated CPU/CUDA parity coverage for the point-point and
  point-edge barrier-Hessian paths, plus non-finite input rejection.
- `tests/benchmark/simulation/bm_plan083_gpu_barrier_friction.cpp`,
  `scripts/write_plan083_gpu_barrier_friction_packet.py`, and
  `tests/test_plan083_gpu_barrier_friction_packet.py` add benchmark packet and
  writer/test coverage for the new primitive barrier-Hessian rows.

The current code has local-output parity evidence for the private
point-triangle primitive barrier gradient, all four primitive-family tangent
stencils, and now the first two primitive-family barrier Hessian rows. The overall
barrier/friction row remains `in-progress` because broader Hessian assembly,
PSD coupling, runtime contact rows, and the top-level/runtime speedup gate
remain future evidence. Durable plan sidecars now record the point-point and
point-edge barrier-Hessian packets as in-progress evidence, not completion.

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
   barrier-Hessian checkpoint may be dirty or locally committed ahead of
   origin, depending on where the previous session stopped.
3. Check hosted CI and new review comments before editing. Do not reply to bot
   comments.
4. Continue on the same PR with the remaining runtime/parity gaps: broader
   Hessian assembly, PSD coupling, runtime contact rows, and packet speedup
   gates.
5. Keep plan/dev-task text honest: packet rows may move from `planned` to
   `in-progress` only with corresponding runtime or packet evidence, and the
   dev-task folder should not be retired until the remaining in-progress work
   is either completed or explicitly relocated by a maintainer.
