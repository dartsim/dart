# Unified Newton-Barrier Handoff

## Stop Directive

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
- Review rule: keep all remaining PLAN-083 follow-up work on this branch and
  PR. Do not open or revive per-packet, per-phase, or per-slice PRs.
- Consolidation context: former stack PRs #2979-#2983 were folded into #2978.
  Earlier runtime/corpus follow-ups #2970, #2971, #2974, and #2976 are already
  merged into `main`.

## Current Local Slice

The current hand-off slice extends the private barrier/friction CUDA packet
with a point-point primitive barrier-Hessian evaluator on the same #2978
branch. It builds on the already-pushed point-triangle primitive
barrier-gradient plus point-triangle, edge-edge, point-edge, and point-point
tangent-stencil packet work.

Changed implementation and coverage surfaces in this hand-off package:

- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh` adds private
  point-point barrier-Hessian input/result packet contracts.
- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cpp` validates
  those inputs, manages device buffers, and exposes
  `evaluatePointPointBarrierHessiansCuda`.
- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cu` adds the CUDA
  kernel and launcher for point-point barrier values, gradients, and compact
  6x6 Hessians.
- `tests/unit/simulation/cuda/test_barrier_friction_kernel_cuda.cpp` adds
  fixed and generated CPU/CUDA parity coverage for the point-point
  barrier-Hessian path, plus non-finite input rejection.
- `tests/benchmark/simulation/bm_plan083_gpu_barrier_friction.cpp`,
  `scripts/write_plan083_gpu_barrier_friction_packet.py`, and
  `tests/test_plan083_gpu_barrier_friction_packet.py` add benchmark packet and
  writer/test coverage for the new point-point barrier-Hessian row.

The current code has local-output parity evidence for the private
point-triangle primitive barrier gradient, all four primitive-family tangent
stencils, and now the first primitive-family barrier Hessian row. The overall
barrier/friction row remains `in-progress` because broader Hessian assembly,
PSD coupling, runtime contact rows, and the top-level/runtime speedup gate
remain future evidence. Durable plan sidecars now record the point-point
barrier-Hessian packet as in-progress evidence, not completion.

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

## Resume Guidance

1. Resume only from `simx/plan083-gpu-contact-candidate-packet` and PR #2978.
2. Inspect the current branch, latest pushed commit, hosted CI, and new review
   comments before editing. Do not reply to bot comments.
3. Continue on the same PR with the remaining runtime/parity gaps: broader
   Hessian assembly, PSD coupling, runtime contact rows, and packet speedup
   gates.
4. Keep plan/dev-task text honest: packet rows may move from `planned` to
   `in-progress` only with corresponding runtime or packet evidence, and the
   dev-task folder should not be retired until the remaining in-progress work
   is either completed or explicitly relocated by a maintainer.
