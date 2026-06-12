# Unified Newton-Barrier Handoff

## Active Branch And PR

- Branch: `simx/plan083-gpu-contact-candidate-packet`
- PR: #2978, `Advance unified Newton-barrier runtime and parity evidence`
- Base: `main`
- Last verified pushed checkpoint before this hand-off:
  `0f2436ce888 Add CUDA edge-edge tangent stencil parity`
- Consolidation rule: keep all remaining PLAN-083 follow-up work on this branch
  and PR. Do not open or revive per-packet/per-phase PRs.

## Critical Stop State

On 2026-06-11 the maintainer instructed the agent to stop implementation and
focus only on hand-off, with no further verification. This hand-off therefore
intentionally preserves unverified work-in-progress code so a fresh session can
resume from one pushed branch instead of reconstructing local edits.

The unverified WIP extends the private barrier/friction CUDA packet from the
already-pushed point-triangle and edge-edge tangent-stencil evidence toward the
remaining primitive-family tangent bases:

- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cuh`
  adds private point-edge and point-point tangent input/result types and
  evaluator declarations.
- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cpp`
  adds host-side validation and wrappers for those evaluators.
- `dart/simulation/compute/cuda/barrier_friction_kernel_cuda.cu`
  adds device coordinate/projection helpers and kernels for point-edge and
  point-point tangent stencils.
- `tests/unit/simulation/cuda/test_barrier_friction_kernel_cuda.cpp`
  adds CPU-parity and non-finite rejection coverage for the new evaluators.
- `tests/benchmark/simulation/bm_plan083_gpu_barrier_friction.cpp`
  adds CPU/GPU benchmark rows for point-edge and point-point tangent stencils.
- `scripts/write_plan083_gpu_barrier_friction_packet.py` and
  `tests/test_plan083_gpu_barrier_friction_packet.py` add packet schema and
  synthetic-row coverage for the new subpackets.

Because verification was explicitly stopped, this WIP must be treated as
unvalidated. Do not update
`docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json`,
`completion-audit.md`, `paper-deck-manifest.md`, or dashboard status counts for
point-edge/point-point tangent-stencil evidence until the fresh session runs
and passes the relevant gates.

## Last Trusted Evidence

The last trusted evidence is for the already-pushed edge-edge tangent-stencil
checkpoint at `0f2436ce888`:

- `pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
- `pixi run -e cuda build-cuda Release`
- focused `test_barrier_friction_kernel_cuda` CTest
- `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- focused Python packet/audit tests
- `pixi run lint`

The packet at that checkpoint measured edge-edge tangent-stencil
`max_result_abs_error=8.881784197001252e-16` and
`speedup=1.339038677334765x`. The overall barrier/friction row remained
`in-progress` because the scalar local-kernel subrow missed its speedup gate
and because point-edge/point-point tangent bases, Hessian assembly, PSD
coupling, and runtime contact rows were still missing.

## Fresh Session Resume Checklist

1. Stay on `simx/plan083-gpu-contact-candidate-packet` and keep PR #2978 as the
   single consolidated review unit.
2. Inspect the hand-off commit and current diff before editing; the current
   point-edge/point-point tangent-stencil work is expected to be unverified.
3. Run `pixi run lint`, CUDA build, focused `test_barrier_friction_kernel_cuda`
   CTest, the barrier/friction benchmark packet, and the packet/audit checkers.
4. Fix any compile/test/packet failures in the WIP before updating durable plan
   sidecars or claiming additional measured rows.
5. After point-edge/point-point evidence is valid, continue on the same PR with
   Hessian assembly, PSD coupling, runtime contact rows, and the remaining
   packet speedup gaps.
