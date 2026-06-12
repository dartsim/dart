# Unified Newton-Barrier Handoff

## Active Continuation Handoff (2026-06-11)

Implementation resumed from the earlier no-verification handoff. Continue only
on `simx/plan083-gpu-contact-candidate-packet` / PR #2978, and keep all
remaining PLAN-083 work on this consolidated branch/PR.

Authoritative current state for a fresh session:

- Branch: `simx/plan083-gpu-contact-candidate-packet`
- PR: #2978, `Advance unified Newton-barrier runtime and parity evidence`
- Base: `main`
- Consolidation rule: keep all remaining PLAN-083 work on this branch/PR. Do
  not open another PLAN-083 PR or revive former stack branches.
- Latest pushed origin head observed before this handoff:
  `6746b63973d Record point-point barrier Hessian packet evidence`.
- Local committed checkpoints ahead of origin:
  - `48dcfb515cf Add point-edge barrier Hessian packet parity`
  - `1dfb21b24da Add point-triangle barrier Hessian packet parity`
  - `1022b609f8c Add point-triangle Hessian PSD packet parity`
  - `c1e6e73b2cf Add point-point and point-edge Hessian PSD packet parity`

The current checkpoint adds reduced off-diagonal sparse-block assembly evidence
on the same branch. A fresh session should inspect `git status -sb` and the
local diff before deciding whether to continue from this checkpoint.

Checkpoint surfaces:

- `dart/simulation/compute/cuda/newton_assembly_solve_cuda.cuh`
- `dart/simulation/compute/cuda/newton_assembly_solve_cuda.cpp`
- `dart/simulation/compute/cuda/newton_assembly_solve_cuda.cu`
- `tests/unit/simulation/cuda/test_newton_assembly_solve_cuda.cpp`
- `tests/benchmark/simulation/bm_newton_assembly_solve_cuda.cpp`
- `scripts/write_newton_assembly_solve_packet.py`
- `tests/test_newton_assembly_solve_packet.py`
- local generated packet:
  `.benchmark_results/plan083/gpu/assembly_linear_solve_parity.json`

The checkpoint adds a reduced pair-slot off-diagonal sparse-block assembly packet:
private CUDA input/result plumbing, a kernel/launcher, CPU/CUDA unit parity
coverage, benchmark rows, and packet-writer synthetic coverage. It is not a
full sparse global factorization, not runtime scene assembly, and not a
completion claim for `assembly-linear-solve`.

Focused evidence gathered for this checkpoint:

- `pixi run python -m pytest tests/test_newton_assembly_solve_packet.py -q`
  passed.
- `pixi run -e cuda build-cuda Release` passed.
- Focused CTest `^test_newton_assembly_solve_cuda$` passed.
- `pixi run -e cuda bm-newton-assembly-solve-packet` passed and wrote the
  generated assembly packet.

The generated benchmark output reported
`Newton assembly/solve packet OK: rows=65536 dofs=49152
max_error=1.7763568394002505e-15 residual=3.0267237170020088e-15
speedup=0.26051227540244215x meets_gate=False`. The observed
off-diagonal row had `pairs=4096`, `active_blocks=4096`,
`block_entries=147456`, `max_result_abs_error=0`, and
`speedup=0.4342273750294942x`.

Likely next steps:

1. Finish the required local gates for this checkpoint if they have not run in
   the current session.
2. Commit this checkpoint locally after the gates pass.
3. Continue with equality reduction, global sparse factorization, runtime scene
   rows, or speedup-gate work only as honest `in-progress` evidence.
4. Before any push, merge latest
   `origin/main` into this published branch, rerun required gates, then push
   only with explicit maintainer approval.

## Current Continuation State

This is the current fresh-session entry point for PR #2978. The earlier
stop-only handoff left point-triangle, then point-point/point-edge Hessian
PSD-projection WIP on this same branch; those slices have now been resumed into
focused packet checkpoints on the consolidated PR branch.

Current local state when this hand-off was updated:

- Branch: `simx/plan083-gpu-contact-candidate-packet`
- PR: #2978, `Advance unified Newton-barrier runtime and parity evidence`
- Base: `main`
- Local branch has three committed checkpoints ahead of
  `origin/simx/plan083-gpu-contact-candidate-packet` before the current
  point-point/point-edge PSD checkpoint:
  - `48dcfb515cf Add point-edge barrier Hessian packet parity`
  - `1dfb21b24da Add point-triangle barrier Hessian packet parity`
  - `1022b609f8c Add point-triangle Hessian PSD packet parity`
- The latest pushed origin head observed in this worktree was
  `6746b63973d Record point-point barrier Hessian packet evidence`.
- The current checkpoint adds point-point and point-edge primitive
  barrier-Hessian PSD-projection parity in the CUDA unit test, benchmark
  packet, packet writer, tracked GPU parity sidecar, completion audit, paper
  manifest, dashboard, and dev-task handoff docs.

- Unit test:
  `ProjectsPointPointBarrierHessiansToPsd` and
  `ProjectsPointEdgeBarrierHessiansToPsd`, using
  `projectSymmetricBlocksToPsdReference()` as the CPU oracle and
  `projectSymmetricBlocksToPsdCuda()` after
  `evaluatePointPointBarrierHessiansCuda()` or
  `evaluatePointEdgeBarrierHessiansCuda()`.
- Benchmark rows:
  `BM_Plan083PointPointBarrierHessianPsdCpu`,
  `BM_Plan083PointPointBarrierHessianPsdCuda`,
  `BM_Plan083PointEdgeBarrierHessianPsdCpu`, and
  `BM_Plan083PointEdgeBarrierHessianPsdCuda`, with focused
  `psd_projection_ns` wall-clock counters for the PSD wrapper.
- Packet-writer support for
  `point_point_barrier_hessian_psd_projection` and
  `point_edge_barrier_hessian_psd_projection`, including synthetic packet test
  rows and assertions.
- Packet evidence from `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`:
  point-point PSD max error `9.592326932761353e-14`, point-point PSD speedup
  `2.6247434778815464x`, point-edge PSD max error
  `7.844391802791506e-12`, point-edge PSD speedup
  `4.388364832151794x`, and top-level speedup
  `0.3604503271533569x` with `meets_speedup_gate = false`.
- Validation for this checkpoint passed `pixi run lint`, `pixi run build`,
  `pixi run test-unit` (161/161), focused packet pytest,
  `pixi run -e cuda build-cuda Release`, focused
  `test_barrier_friction_kernel_cuda` CTest,
  `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`, the PLAN-083 GPU
  parity/completion-audit checker pair, and the focused packet/audit pytest
  trio (13 passed).

Before any future push to #2978, merge the latest `origin/main` into this
published branch, run the required gates for all accumulated changes, and push
only with explicit maintainer approval. Keep all remaining PLAN-083 work on
this same branch and PR; do not open another PLAN-083 PR.

## Historical Stop Directive

On 2026-06-11 the maintainer instructed the agent to stop implementation and
focus only on hand-off, with no further verification. That stop-only handoff is
historical context; continuation later resumed on the same branch and completed
the point-point/point-edge Hessian PSD packet slice described above. Pushes,
PR updates, review actions, and CI re-triggers still require explicit
maintainer approval.

## Active Branch And PR

- Branch: `simx/plan083-gpu-contact-candidate-packet`
- PR: #2978, `Advance unified Newton-barrier runtime and parity evidence`
- Base: `main`
- Latest pushed commit before the local barrier-Hessian slices:
  `6746b63973d Record point-point barrier Hessian packet evidence`
- Local commits not yet pushed:
  `48dcfb515cf Add point-edge barrier Hessian packet parity`
  `1dfb21b24da Add point-triangle barrier Hessian packet parity`
  `1022b609f8c Add point-triangle Hessian PSD packet parity`
- The current checkpoint adds point-point and point-edge Hessian PSD-projection
  packet parity on top of those commits and should be committed locally only
  after the remaining required gates pass.
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
writer/tests, durable sidecar updates, and a point-triangle Hessian
PSD-projection subrow using the existing private PSD wrapper. The refreshed
packet measured
`point_triangle_barrier_hessian.max_result_abs_error =
2.5887393237583516e-12`,
`point_triangle_barrier_hessian.speedup = 1.3808963801674754`,
`point_triangle_barrier_hessian_psd_projection.max_result_abs_error =
2.8990143619012088e-12`,
`point_triangle_barrier_hessian_psd_projection.speedup =
2.6270517059212906`, and top-level `speedup = 0.22046365998499093` with
`meets_speedup_gate = false`.

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
  writer/test coverage for the new primitive barrier-Hessian rows and the
  point-triangle, point-point, and point-edge Hessian PSD-projection subrows.

The current code has local-output parity evidence for the private
point-triangle primitive barrier gradient, all four primitive-family tangent
stencils, the first three primitive-family barrier Hessian rows, and the
primitive-family Hessian PSD-projection rows for point-triangle, point-point,
and point-edge. The overall barrier/friction row remains `in-progress` because
broader sparse Hessian assembly, runtime contact rows, and the top-level/runtime
speedup gate remain future evidence. Durable plan sidecars now record the
point-triangle, point-point, point-edge barrier-Hessian packets and
point-triangle/point-point/point-edge Hessian PSD projection as in-progress
evidence, not completion.

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
   and point-triangle barrier-Hessian checkpoints plus the point-triangle
   Hessian PSD-projection checkpoint are locally committed ahead of origin, and
   the current checkpoint completes point-point/point-edge Hessian
   PSD-projection packet parity locally.
3. Check hosted CI and new review comments before editing. Do not reply to bot
   comments.
4. Continue on the same PR with the remaining runtime/parity gaps: broader
   sparse Hessian assembly, runtime contact rows, and packet speedup gates.
5. Keep plan/dev-task text honest: packet rows may move from `planned` to
   `in-progress` only with corresponding runtime or packet evidence, and the
   dev-task folder should not be retired until the remaining in-progress work
   is either completed or explicitly relocated by a maintainer.
