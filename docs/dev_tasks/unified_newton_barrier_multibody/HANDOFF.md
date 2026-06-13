# Unified Newton-Barrier Handoff

## Combined Scene Runtime CCD Line-Search Packet Checkpoint (2026-06-13)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private CCD/line-search packet with a reduced
combined scene runtime CCD row. The benchmark builds the same DART `World`
deformable surface used by the existing scene-owned runtime CCD rows, extracts
the 512 point-triangle and 1,536 edge-edge runtime CCD pairs from that surface,
and evaluates both existing GPU endpoint-linear CCD paths in one aggregate row.
This is reduced packet evidence only; it does not prove analytic curved CCD,
production scene-level line-search feasibility inside `World::step`, GPU
`World::step`, or a top-level speedup claim.

Fresh packet evidence records one scene body, 2,560 scene nodes, 768 surface
triangles, and 2,048 total scene CCD pairs: 512 point-triangle plus 1,536
edge-edge. The combined row records 512 total hits, 256 point-triangle hits,
256 edge-edge hits, `max_result_abs_error=5.551115123125783e-17`, and
`speedup=0.5277629321620969x` (`meets_speedup_gate=false`). The top-level
CCD/line-search packet covers 266,240 pairs and 1,183,744 sampled/evaluated
segments, records `hit_count=197632`,
`max_result_abs_error=5.551115123125783e-17`, and
`speedup=0.35165112647134533x` (`meets_speedup_gate=false`), so the durable GPU
packet row remains `in-progress`.

Current validation passed:

- `pixi run python -m py_compile scripts/write_plan083_gpu_ccd_line_search_packet.py`
- `pixi run python -m pytest tests/test_plan083_gpu_ccd_line_search_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda python scripts/write_plan083_gpu_ccd_line_search_packet.py`
- `pixi run -e cuda python scripts/write_plan083_gpu_ccd_line_search_packet.py --skip-run`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m json.tool docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json >/dev/null`
- `ctest --test-dir build/cuda/cpp/Release --output-on-failure -R '^test_ccd_line_search_cuda$'`
- `git diff --check`
- `pixi run lint`

## Combined Scene Runtime Sweep-Filter Packet Checkpoint (2026-06-13)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private contact-candidate packet with a reduced
combined scene runtime sweep-filter row. The benchmark builds the same DART
`World` deformable surface used by the existing scene-owned runtime sweep rows
and runs the existing point-triangle and edge-edge GPU sweep builders in one
aggregate row. This is reduced packet evidence only; it does not prove
production runtime scene filtering inside `World::step`, GPU `World::step`
contact candidate construction, or a top-level speedup claim.

Fresh packet evidence records one scene body, 2,560 points, 768 surface
triangles, 2,304 surface edges, and 7,274,496 total possible sweep pairs:
1,966,080 point-triangle plus 5,308,416 edge-edge. The combined row emits
2,048 compact candidates: 512 point-triangle and 1,536 edge-edge. It records
`max_result_abs_error=5.551115123125783e-17` and
`speedup=0.04386519668565386x` (`meets_speedup_gate=false`). The top-level
contact-candidate packet records `max_result_abs_error=5.551115123125783e-17`
and `speedup=0.002822497416176393x` (`meets_speedup_gate=false`), so the
durable GPU packet row remains `in-progress`.

Current validation passed:

- `pixi run python -m py_compile scripts/write_plan083_gpu_contact_candidate_packet.py`
- `pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda python scripts/write_plan083_gpu_contact_candidate_packet.py`
- `pixi run -e cuda python scripts/write_plan083_gpu_contact_candidate_packet.py --skip-run`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m json.tool docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json >/dev/null`
- `ctest --test-dir build/cuda/cpp/Release --output-on-failure -R '^test_contact_candidate_filter_cuda$'`
- `git diff --check`
- `pixi run lint`

## Combined Scene Runtime Candidate-Filter Packet Checkpoint (2026-06-13)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private contact-candidate packet with a reduced
combined scene runtime candidate-filter row. The benchmark builds the same DART
`World` deformable surface used by the existing scene-owned runtime candidate
rows, extracts point-triangle and edge-edge candidate buffers once, and evaluates
both existing CUDA runtime candidate-buffer distance paths in one aggregate row.
This is reduced packet evidence only; it does not prove full runtime scene
filtering, GPU `World::step` contact candidate construction, or a top-level
speedup claim.

Fresh packet evidence records one scene body, 2,560 points, 768 surface
triangles, 2,304 surface edges, and 2,048 total runtime candidates:
512 point-triangle and 1,536 edge-edge. The combined row records
`max_result_abs_error=5.551115123125783e-17` and
`speedup=0.23130939805056844x` (`meets_speedup_gate=false`). The top-level
contact-candidate packet records `max_result_abs_error=5.551115123125783e-17`
and `speedup=0.02137317646447672x` (`meets_speedup_gate=false`), so the durable
GPU packet row remains `in-progress`.

Current validation passed:

- `pixi run python -m py_compile scripts/write_plan083_gpu_contact_candidate_packet.py`
- `pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda python scripts/write_plan083_gpu_contact_candidate_packet.py`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m json.tool docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json >/dev/null`
- `ctest --test-dir build/cuda/cpp/Release --output-on-failure -R '^test_contact_candidate_filter_cuda$'`
- `git diff --check`
- `pixi run lint`

## Combined Scene Runtime Barrier-Hessian Packet Checkpoint (2026-06-13)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private barrier/friction packet with a reduced
combined all-family scene runtime barrier-Hessian row. The benchmark builds the
same DART `World` deformable surface used by the existing scene-owned runtime
barrier rows, extracts the runtime point-triangle and edge-edge candidate sets
once, expands the point-triangle contacts into point-edge and point-point
primitive contacts, and evaluates point-triangle, point-edge, point-point, and
edge-edge barrier-Hessian CUDA paths in one packet row. This is reduced packet
evidence only; it does not prove broader sparse Hessian assembly, full runtime
scene filtering, GPU `World::step`, or a top-level speedup claim.

Fresh packet evidence records one scene body, 2,560 scene nodes, 768 surface
triangles, 512 source point-triangle candidates, 1,536 source edge-edge
candidates, and 5,120 total runtime primitive contacts: 512 point-triangle,
1,536 point-edge, 1,536 point-point, and 1,536 edge-edge. The combined row
records 2,560 active barriers with family counts 256/768/256/1,280,
`max_result_abs_error=2.220446049250313e-15`, and
`speedup=0.30063758288923775x` (`meets_speedup_gate=false`). The top-level
barrier/friction packet records
`max_result_abs_error=7.844391802791506e-12` and
`speedup=0.048260742808231415x` (`meets_speedup_gate=false`), so the durable
GPU packet row remains `in-progress`.

Current validation passed:

- `pixi run python -m py_compile scripts/write_plan083_gpu_barrier_friction_packet.py`
- `pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda python scripts/write_plan083_gpu_barrier_friction_packet.py`
- `ctest --test-dir build/cuda/cpp/Release --output-on-failure -R '^test_barrier_friction_kernel_cuda$'`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`

## Scene-Owned Equality-Reduced Diagonal Solve Packet Checkpoint (2026-06-13)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private Newton assembly/solve packet with a reduced
scene-owned equality-reduced diagonal solve row. The benchmark builds the same
DART `World` deformable surface used by the scene-owned sparse graph rows,
emits one diagonal Newton row per scene node, maps each node's six local DOFs
into three reduced axis coordinates, and runs the existing private
equality-projected diagonal solve on CPU and CUDA. This is reduced packet
evidence only; it does not prove production sparse equality reduction, full
runtime sparse Hessian construction/assembly, unbounded production
direct/global sparse factorization, production nonlinear equality convergence
policy/solving, GPU `World::step` assembly/solve integration, or a top-level
speedup claim.

Fresh packet evidence records `scene_node_count=2560`,
`scene_triangle_count=768`, `full_dof_count=15360`,
`reduction_entry_count=15360`, `reduced_dof_count=7680`,
`active_reduced_dof_count=7680`,
`max_result_abs_error=8.865239993401835e-16`,
`residual_norm=1.096424743963087e-14`,
`step_norm=74.55094721551956`, and
`speedup=0.028643430542350582x` (`meets_speedup_gate=false`). The top-level
assembly/solve packet records `max_result_abs_error=2.9103830456733704e-11`,
`residual_norm=3.6960887605500504e-13`, and
`speedup=0.00027005298586267053x` (`meets_speedup_gate=false`), so the durable
GPU packet row remains `in-progress`.

Current validation passed:

- `pixi run python -m py_compile scripts/write_newton_assembly_solve_packet.py`
- `pixi run python -m pytest tests/test_newton_assembly_solve_packet.py -q`
- `pixi run -e cuda build-cuda`
- `pixi run -e cuda python scripts/write_newton_assembly_solve_packet.py`

## Scene-Derived Bounded Direct Sparse Factor Solve Packet Checkpoint (2026-06-13)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private Newton assembly/solve packet with a reduced
scene-owned bounded direct sparse factor solve row. The benchmark builds the
same DART `World` deformable surface used by the scene-owned sparse graph
packets, selects one connected three-node surface-triangle subset, remaps that
subset into 18 capped DOFs and three 6x6 sparse blocks, and runs the existing
in-kernel dense Cholesky direct sparse solve. This is reduced packet evidence
only; it does not prove production sparse Hessian graph deduplication, full
runtime sparse Hessian construction/assembly, unbounded production
direct/global sparse factorization, production nonlinear equality convergence
policy/solving, GPU `World::step` assembly/solve integration, or a top-level
speedup claim.

Fresh packet evidence records a scene-derived direct sparse factor row with
`scene_node_count=2560`, `scene_triangle_count=768`,
`selected_scene_node_count=3`, `selected_scene_edge_pair_count=3`,
`dof_count=18`, `block_count=3`, `block_entry_count=108`,
`minimum_factor_pivot=0.9617692030835673`,
`max_result_abs_error=1.580969625257288e-18`,
`residual_norm=3.061659904040655e-17`,
`max_residual_abs=1.9734344004336875e-17`,
`step_norm=0.061746963682577156`, and
`speedup=0.007412172587967598x` (`meets_speedup_gate=false`). The top-level
assembly/solve packet records `max_result_abs_error=3.2741809263825417e-11`,
`residual_norm=3.707781567402575e-13`, and
`speedup=0.007412172587967598x` (`meets_speedup_gate=false`), so the durable
GPU packet row remains `in-progress`.

## Bounded Direct Sparse Factor Solve Packet Checkpoint (2026-06-13)

work continued
locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all
remaining PLAN-083 follow-up work consolidated there; do not push, PR-comment,
resolve review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private Newton assembly/solve packet with a bounded
reduced direct sparse factor solve row. The CUDA packet assembles full-space
6-DOF diagonal rows and symmetric 6x6 sparse block entries into a small dense
global matrix, runs an in-kernel Cholesky factor/solve, and checks the solved
step with the existing sparse residual kernels. The benchmark keeps the direct
factorization capped at 24 DOFs (`max_supported_dof_count=48`) while scaling the
assembly row load to 65,536 rows. This is reduced packet evidence only; it does
not prove production sparse Hessian graph deduplication, full runtime sparse
Hessian construction/assembly, unbounded production direct/global sparse
factorization, production nonlinear equality convergence policy/solving, GPU
`World::step` assembly/solve integration, or a top-level speedup claim.

Fresh packet evidence records a direct sparse factor row with `row_count=65536`,
`body_count=4`, `dof_count=24`, `block_count=4`, `block_entry_count=144`,
`minimum_factor_pivot=101.59768698037762`,
`max_result_abs_error=3.092281986027956e-11`,
`residual_norm=5.378686498871594e-16`,
`max_residual_abs=2.8327584905843623e-16`,
`step_norm=0.00012707425595729184`, and
`speedup=0.14347114847141065x` (`meets_speedup_gate=false`). The top-level
assembly/solve packet records `max_result_abs_error=3.092281986027956e-11`,
`residual_norm=3.706539707094154e-13`, and
`speedup=0.1234547960729924x` (`meets_speedup_gate=false`), so the durable GPU
packet row remains `in-progress`.

Current validation passed:

- `pixi run python -m py_compile scripts/write_newton_assembly_solve_packet.py`
- `pixi run python -m pytest tests/test_newton_assembly_solve_packet.py`
- `pixi run -e cuda build-cuda`
- `pixi run -e cuda bash -lc 'build/cuda/cpp/Release/bin/test_newton_assembly_solve_cuda --gtest_filter=NewtonAssemblySolveCuda.*DirectSparseSolve*:NewtonAssemblySolveCuda.MatchesCpuSparseCgSolve:NewtonAssemblySolveCuda.MatchesCpuEqualityReducedSolve'`
- `pixi run -e cuda python scripts/write_newton_assembly_solve_packet.py`
- `pixi run python -m json.tool docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json >/dev/null`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_newton_assembly_solve_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
- `git diff --check`
- `pixi run lint`

A broad `pixi run -e cuda test-cuda` attempt was interrupted after it stalled in
unrelated `test_lcp_jacobi_batch_cuda`; do not count that broad suite as passed
for this checkpoint.

## Scene-Owned Nonlinear Equality Convergence Packet Checkpoint (2026-06-13)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private Newton assembly/solve packet with a reduced
scene-owned capped nonlinear distance-equality convergence row on top of the
existing scene-owned nonlinear equality assembly and one-step solve rows. The
benchmark builds one DART `World` with a deformable surface, extracts
scene-owned nodes and surface triangles, creates one distance-equality
constraint per oriented surface-edge slot, zeros velocity damping for the
geometric convergence fixture, runs up to eight damped Jacobi-style correction
iterations in scene-node generalized coordinates, reports initial/final
nonlinear distance residuals, and assembles final endpoint rows/blocks on the
GPU. This is reduced packet evidence only; it does not prove production
nonlinear equality convergence policy/solving, production constraint graph
ownership, direct/global sparse factorization, full runtime sparse Hessian
construction/assembly, GPU `World::step` assembly/solve integration, or a
top-level speedup claim.

Fresh packet evidence records 2,560 scene nodes, 768 surface triangles, 2,304
surface-edge equality constraints, 4,608 local constraint rows, 2,304 sparse
blocks, 82,944 6x6 block entries, and 15,360 dofs. The scene-owned nonlinear
equality assembly row records
`max_constraint_residual_abs=0.023006792475267046`,
`max_result_abs_error=2.8421709430404007e-13`,
`max_diagonal=1001.9679687500002`,
`max_gradient_abs=1.2260273625132818`,
`max_block_abs=1001.9674687500002`, and
`speedup=0.11643289378198905x`. The scene-owned nonlinear equality solve row
records `regularization=0.05`, `active_dof_count=9215`,
`max_post_solve_linearized_residual_abs=0.07899550902133222`,
`step_norm=0.3288379160439768`,
`max_result_abs_error=2.8421709430404007e-13`, and
`speedup=0.07249443690039502x`. The scene-owned nonlinear equality convergence
row records `regularization=0.05`, `residual_tolerance=1e-05`,
`completed_iteration_count=8`, `converged=false`, `active_dof_count=9216`,
`initial_max_constraint_residual_abs=0.023006792475267046`,
`final_max_constraint_residual_abs=0.022842203667180705`,
`step_norm=0.28479436450197887`,
`max_result_abs_error=4.547473508864641e-13`, and
`speedup=0.09527533471087142x`. The top-level assembly/solve packet records
`max_result_abs_error=4.547473508864641e-13`,
`residual_norm=3.705385775963348e-13`, and
`speedup=0.012632667834780719x` (`meets_speedup_gate=false`), so the durable GPU
packet row remains `in-progress`.

Latest local gates:

- `pixi run python -m py_compile scripts/write_newton_assembly_solve_packet.py`
- `pixi run python -m pytest tests/test_newton_assembly_solve_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `git diff --check`
- `pixi run build`
- `pixi run test-unit`
- `pixi run -e cuda build-cuda Release`
- `ctest --test-dir build/cuda/cpp/Release --output-on-failure -R '^test_newton_assembly_solve_cuda$'`
- `pixi run -e cuda python scripts/write_newton_assembly_solve_packet.py`
- `pixi run lint`
- `pixi run -e cuda test-all`

## Scene-Owned Sparse Graph Assembly Packet Checkpoint (2026-06-12)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private Newton assembly/solve packet with a reduced
scene-owned sparse graph construction/assembly row. The benchmark builds one
DART `World` with a deformable surface, extracts scene-owned nodes and surface
triangles, counts triangle incidence on the GPU, emits deterministic per-node
diagonal Newton rows, and emits one oriented 6x6 sparse block per surface-edge
pair. This is reduced packet evidence only; it does not prove production sparse
graph deduplication, full runtime sparse Hessian construction/assembly,
direct/global sparse factorization, nonlinear equality constraints, GPU
`World::step` assembly/solve integration, or a speedup claim.

Fresh packet evidence records 2,560 scene nodes, 768 surface triangles, 2,304
surface-edge pair slots, 2,560 diagonal rows, 2,304 sparse blocks, 82,944 6x6
block entries, and 15,360 dofs. The scene-owned sparse graph assembly row
records `max_result_abs_error=4.440892098500626e-16`,
`max_diagonal=3.8525`, `max_gradient_abs=7.6674999999999995`,
`max_block_abs=0.0014150000000000002`, and
`speedup=0.034699192240483435x`. The top-level assembly/solve packet records
`max_result_abs_error=4.009584404066394e-15`,
`residual_norm=3.6981539869853577e-13`, and
`speedup=0.01831350563120611x` (`meets_speedup_gate=false`), so the durable
GPU packet row remains `in-progress`.

Latest local gates:

- focused assembly/solve packet pytest
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`
- PLAN-083 GPU parity/completion-audit validators
- focused PLAN-083 packet pytest trio
- `git diff --check`
- `pixi run lint`

## Scene-Owned Runtime CCD/Line-Search Packet Checkpoint (2026-06-12)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private CCD/line-search packet with moving-triangle
point-triangle CCD support plus reduced scene-owned runtime point-triangle and
edge-edge rows from one DART `World` deformable surface. The point-triangle
scene row now consumes the full runtime candidate set: 512 point-triangle CCD
pairs split across 256 static-triangle and 256 moving-triangle candidates. The
edge-edge scene row consumes the runtime edge-edge candidate set directly. This
is still reduced packet evidence only; analytic curved CCD, full scene-level
line-search feasibility, and runtime speedup remain future work.

Fresh packet evidence records 65,536 endpoint-linear point-triangle pairs,
65,536 endpoint-linear edge-edge pairs, 65,536 sampled rigid-curved
point-triangle trajectories over 524,288 segments, 65,536 sampled rigid-curved
edge-edge trajectories over 524,288 segments, 512 runtime point-triangle CCD
pairs split across 256 static-triangle and 256 moving-triangle candidates, and
1,536 runtime edge-edge CCD pairs from the same scene. The top-level packet
covers 264,192 pairs and 1,181,696 sampled/evaluated segments with
`hit_count=197120`, `max_result_abs_error=5.551115123125783e-17`,
`result_abs_error_tolerance=1e-06`, and minimum primitive-family
`speedup=0.3492306596158999x` (`meets_speedup_gate=false`). The durable CCD row
remains `in-progress` until analytic curved CCD, full scene-level line-search
feasibility, and runtime speedup have evidence.

Latest local gates:

- focused CCD packet pytest
- `pixi run -e cuda build-cuda Release`
- focused CUDA CCD CTest
- `pixi run -e cuda bm-plan083-gpu-ccd-line-search-packet`
- PLAN-083 GPU parity/completion-audit validators
- focused PLAN-083 packet pytest trio
- `git diff --check`
- `pixi run lint`

## Scene-Owned Assembly/Solve Packet Checkpoint (2026-06-12)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private Newton assembly/solve packet with a reduced
scene-owned diagonal assembly/solve row. The benchmark builds one DART `World`
with a deformable surface, extracts node masses, positions, velocities, and
surface-triangle incidence into deterministic per-node diagonal Newton rows,
and runs the existing CPU/CUDA diagonal assembly/solve parity path.

Fresh packet evidence records 2,560 scene nodes, 768 surface triangles, 15,360
dofs, and 15,360 active dofs with
`max_result_abs_error=4.4021269828727137e-16`,
`residual_norm=1.0086044953133903e-14`,
`step_norm=92.04417917780955`, and `speedup=0.22366599153516908x` for the
scene-owned diagonal row. The top-level assembly/solve packet records
`max_result_abs_error=3.552713678800501e-15`,
`residual_norm=9.937538231096445e-14`, and
`speedup=0.22366599153516908x` (`meets_speedup_gate=false`), so the durable GPU
packet row remains `in-progress`.

Latest local gates:

- focused assembly/solve packet pytest
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`
- PLAN-083 GPU parity/completion-audit validators
- focused PLAN-083 packet pytest trio
- `git diff --check`
- `pixi run lint`

## Sparse CG Solve Packet Checkpoint (2026-06-12)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private Newton assembly/solve packet with a capped
sparse Conjugate Gradient solve row. The CUDA path assembles full-space
diagonal rows, applies symmetric 6x6 sparse off-diagonal body blocks in each
matrix-vector product, runs a bounded CG loop, and recomputes the final sparse
residual.

Fresh packet evidence records 65,536 rows, 8,192 bodies, 49,152 dofs, 8,192
sparse blocks, 294,912 block entries, 32 maximum iterations, 14 completed
iterations, and 49,152 active dofs with `residual_tolerance=1e-12`,
`initial_residual_norm=66.04823994627199`,
`max_result_abs_error=1.7763568394002505e-15`,
`residual_norm=9.935597225004938e-14`,
`max_residual_abs=5.011832962531493e-15`,
`step_norm=10.694462363332692`, and `speedup=1.965091295559266x` for the
sparse CG row. The top-level assembly/solve packet records
`max_result_abs_error=3.552713678800501e-15`,
`residual_norm=9.935597225004938e-14`, and
`speedup=0.3014575923899792x` (`meets_speedup_gate=false`), so the durable GPU
packet row remains `in-progress`.

Latest local gates:

- focused assembly/solve packet pytest
- PLAN-083 GPU parity/completion-audit validators
- focused PLAN-083 packet pytest trio
- `git diff --check`
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`
- `pixi run lint`

## Sparse Jacobi Solve Packet Checkpoint (2026-06-12)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private Newton assembly/solve packet with a
fixed-iteration sparse Jacobi solve row. The CUDA path assembles full-space
diagonal rows, applies symmetric 6x6 sparse off-diagonal body blocks during a
deterministic diagonal-preconditioned residual update, runs 16 iterations, and
recomputes the final sparse residual.

Fresh packet evidence records 65,536 rows, 8,192 bodies, 49,152 dofs, 8,192
sparse blocks, 294,912 block entries, 16 iterations, and 49,152 active dofs
with `max_result_abs_error=1.7763568394002505e-15`,
`residual_norm=2.987542991182462e-15`,
`max_residual_abs=5.095750210681871e-17`,
`step_norm=10.694462363332693`, and `speedup=1.7149190773941674x` for the
sparse Jacobi row. The top-level assembly/solve packet records
`max_result_abs_error=3.552713678800501e-15`,
`residual_norm=3.025652421429066e-15`, and
`speedup=0.24004618767678942x` (`meets_speedup_gate=false`), so the durable GPU
packet row remains `in-progress`.

Latest local gates:

- focused assembly/solve packet pytest
- PLAN-083 GPU parity/completion-audit validators
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`
- `pixi run lint`

## Sparse Residual Assembly Packet Checkpoint (2026-06-12)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private Newton assembly/solve packet with a sparse
block residual row. The CUDA path assembles full-space diagonal rows, seeds the
residual with the assembled gradient, applies regularized diagonal terms, and
applies symmetric 6x6 off-diagonal body blocks to a supplied step vector.

Fresh packet evidence records 65,536 rows, 8,192 bodies, 49,152 dofs, 8,192
sparse blocks, and 294,912 block entries with
`max_result_abs_error=1.7763568394002505e-15`,
`output_norm=188.03633837316767`, and `speedup=0.41564862959734655x` for the
sparse residual row. The top-level assembly/solve packet records
`max_result_abs_error=3.552713678800501e-15`,
`residual_norm=3.02723585284463e-15`, and
`speedup=0.24114770986042178x` (`meets_speedup_gate=false`), so the durable GPU
packet row remains `in-progress`.

Latest local gates:

- focused assembly/solve packet pytest
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`

## Equality-Reduced Assembly/Solve Packet Checkpoint (2026-06-12)

Work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private Newton assembly/solve packet with a
reduced sparse equality-reduction row. The CUDA path assembles full-space
diagonal rows, projects them through a sparse reduced-coordinate basis, solves
the regularized reduced diagonal Newton step, and expands the step back to
full coordinates.

Fresh packet evidence records 65,536 rows, 8,192 bodies, 49,152 full dofs,
49,152 reduction entries, and 24,576 reduced dofs with
`max_result_abs_error=3.552713678800501e-15`,
`residual_norm=2.927032132859879e-15`, and
`speedup=0.4504406756295252x` for the equality-reduced row. The top-level
assembly/solve packet records `max_result_abs_error=3.552713678800501e-15`,
`residual_norm=3.026790431755041e-15`, and
`speedup=0.2937796651929902x` (`meets_speedup_gate=false`), so the durable GPU
packet row remains `in-progress`.

Latest local gates:

- focused assembly/solve packet pytest
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`

## Edge-Edge Hessian PSD Packet Checkpoint (2026-06-12)

The latest `origin/main` has been merged into
`simx/plan083-gpu-contact-candidate-packet`, bringing in the DART 7
architecture hardening/work-packet harness from PR #2986. Continue to keep
PLAN-083 follow-up work consolidated on PR #2978; do not push, PR-comment,
resolve review threads, trigger CI, open or close PRs, delete branches, or
claim unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private barrier/friction packet with the missing
edge-edge Hessian PSD-projection benchmark row.

Fresh packet evidence records 59,578 active edge-edge PSD-projection barriers
with `max_result_abs_error=4.334310688136611e-13`,
`psd_projection_ns=121906741.0`, and `speedup=2.045170785242439x`. The
conditioned edge-edge primitive Hessian row remains covered with
`max_result_abs_error=3.623767952376511e-13`; the scene-owned edge-edge runtime
row still consumes 1,536 runtime edge-edge candidates from one DART `World`
deformable surface and records 1,280 active barriers with
`max_result_abs_error=2.220446049250313e-15`. The top-level barrier/friction
packet records `speedup=0.17984862275960234x`
(`meets_speedup_gate=false`), so the durable GPU packet row remains
`in-progress`.

Latest local gates so far:

- focused barrier/friction packet pytest
- focused edge-edge PSD CPU/CUDA benchmark smoke
- `pixi run -e cuda run-cpp-target test_barrier_friction_kernel_cuda`
- `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`

## Fresh Codex Goal Prompt

Use this prompt for the next fresh Codex session:

```text
Continue PLAN-083 on branch simx/plan083-gpu-contact-candidate-packet / PR #2978. First read AGENTS.md, docs/ai/principles.md, docs/ai/orchestration.md, docs/dev_tasks/unified_newton_barrier_multibody/RESUME.md, and docs/dev_tasks/unified_newton_barrier_multibody/HANDOFF.md. Keep all PLAN-083 work consolidated on this branch/PR; do not open another PLAN-083 PR. Do not push, comment on PRs, resolve review threads, trigger CI, or delete branches without explicit maintainer approval.

Current local packet evidence covers contact candidates, endpoint-linear plus sampled rigid-curved CCD/line-search, reduced scene-owned runtime CCD rows plus a reduced combined scene runtime CCD line-search row, barrier/friction local kernels, primitive and reduced scene barrier-Hessian rows including edge-edge PSD projection, reduced diagonal/off-diagonal/sparse-residual/sparse-Jacobi/sparse-CG/equality-reduced assembly/solve plus scene-owned diagonal, scene-owned equality-reduced diagonal, sparse off-diagonal surface-edge, sparse graph construction/assembly, and nonlinear distance-equality assembly/solve rows, and reduced scene parity. The remaining high-value gaps are full runtime scene filtering, GPU World::step contact candidate construction, analytic curved CCD, production scene-level line search inside World::step, full runtime sparse Hessian graph construction and assembly, production sparse equality reduction, direct/global sparse factorization, production nonlinear equality convergence policy/solving, and top-level speedup gates. Pick one bounded packet-style slice on this same branch, keep rows in-progress until their full row policy is satisfied, run the required local gates, update the dev-task/plan sidecars honestly, commit with a plain descriptive message, and stop with a clean handoff if a maintainer decision or architecture blocker is needed.
```

## Resumed Barrier-Hessian Packet Checkpoint (2026-06-12)

The maintainer gave a fresh `continue` instruction after the stop-only
handoff. Work may resume locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978, while keeping all
remaining PLAN-083 work consolidated on this single branch/PR. Do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or clean branches without explicit maintainer approval.

This resume validated the uncommitted scene-owned point-triangle, point-edge,
and point-point barrier-Hessian packet checkpoint and refreshed the packet
evidence. The latest generated packet records 512 scene-owned point-triangle
candidates, 1,536 scene-derived point-edge candidates, and 1,536
scene-derived point-point candidates from one deformable scene body with 2,560
nodes and 768 surface triangles. CPU/GPU barrier-Hessian parity holds within
`1.7763568394002505e-15` for the point-triangle runtime row,
`8.881784197001252e-16` for the point-edge runtime row, and `0` for the
point-point runtime row. The top-level barrier/friction packet records
`max_result_abs_error=7.844391802791506e-12` and
`speedup=0.2016402329495093x` (`meets_speedup_gate=false`).

## Stop-Only Handoff (2026-06-12)

The maintainer explicitly stopped further implementation and verification here.
Do not continue work, run validation, push, PR-comment, resolve review threads,
trigger CI, open or close PRs, delete branches, or clean branches unless the
maintainer gives a new resume instruction.

Resume context: branch `simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). The working
tree intentionally contains uncommitted handoff/state changes plus the latest
scene-owned point-triangle barrier-Hessian packet work. Treat those local
changes as intentional in-progress state, not cleanup candidates.

Current local status at handoff: branch is ahead of
`origin/simx/plan083-gpu-contact-candidate-packet` by 16 commits, with
modified dev-task docs, PLAN-083 status docs, packet writer/test files, and the
GPU barrier/friction benchmark. Verification was not continued after this
stop-only handoff request.

## Scene-Owned Runtime Barrier-Hessian Checkpoint (2026-06-12)

Continue from `simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work consolidated on this single branch/PR. Do not push,
PR-comment, resolve review threads, trigger CI, open PRs, close PRs, delete
branches, or clean up branches without explicit maintainer approval. Before any
future push, merge latest `origin/main` into this published branch, rerun the
required gates, and push only with explicit approval.

This checkpoint adds reduced scene-owned point-triangle, point-edge, and
point-point barrier-Hessian runtime rows to the private GPU barrier/friction
packet. The benchmark builds one DART `World` deformable surface, extracts
motion-aware point-triangle contact candidates from the runtime scene surface,
expands each candidate over its three triangle edges for the point-edge row and
over its three triangle vertices for the point-point row, evaluates the
candidate barrier Hessians on CPU and CUDA, and records scene body/node/triangle
counts.

Fresh packet evidence records 512 scene-owned point-triangle candidates, 1,536
scene-derived point-edge candidates, and 1,536 scene-derived point-point
candidates from one deformable scene body with 2,560 nodes and 768 surface
triangles. CPU/GPU barrier-Hessian parity holds within
`1.7763568394002505e-15` for the point-triangle runtime row,
`8.881784197001252e-16` for the point-edge runtime row, and `0` for the
point-point runtime row. The top-level barrier/friction packet records
`max_result_abs_error=7.844391802791506e-12` and
`speedup=0.2016402329495093x` (`meets_speedup_gate=false`).

This is reduced scene-owned point-triangle, point-edge, and point-point
barrier-Hessian evidence only. It does not claim broader sparse Hessian
assembly, broader sparse barrier/contact assembly, full GPU `World::step`, or
barrier/friction speedup-gate completion.

Latest local gates:

- focused barrier/friction packet pytest
- `git diff --check`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`

## Scene-Owned Runtime Sweep Checkpoint (2026-06-12)

Continue from `simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work consolidated on this single branch/PR. Do not push,
PR-comment, resolve review threads, trigger CI, open PRs, close PRs, delete
branches, or clean up branches without explicit maintainer approval. Before any
future push, merge latest `origin/main` into this published branch, rerun the
required gates, and push only with explicit approval.

This checkpoint adds reduced scene-owned point-triangle and edge-edge runtime
sweep broad-phase packet rows. The benchmark builds one DART `World`
deformable surface, extracts node start/end states and surface triangles from
the `DeformableBody` handle, runs private device-sorted swept-AABB
sweep-and-prune rows on that scene-owned surface, and records `scene_body_count`
for the CPU/GPU rows.

Fresh packet evidence records exact CPU/GPU endpoint-distance parity within
`5.551115123125783e-17`. The point-triangle scene sweep covers 1,966,080
possible pairs over 2,560 points and 768 triangles, emits 512 compact
candidates, and records `speedup=0.07057754068474563x`. The edge-edge scene
sweep covers 5,308,416 possible pairs over 2,304 surface edges, emits 1,536
compact candidates, and records `speedup=0.06350297113311432x`. The top-level
contact-candidate packet now records `candidate_pair_count=7667712` and
`speedup=0.024497841563440446x` (`meets_speedup_gate=false`).

This is reduced scene-owned sweep evidence only. It does not claim full
runtime scene filtering, GPU `World::step` contact candidate construction, or
contact-candidate speedup-gate completion.

Latest local gates:

- focused contact-candidate packet pytest
- `git diff --check`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda test-cuda` (8/8)
- `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit` (161/161)

## Current Continuation Handoff (2026-06-12)

Work has resumed after the prior maintainer stop-only handoff. Continue local
PLAN-083 work from this consolidated branch/PR, but do not push,
PR-comment, resolve review threads, trigger CI, open PRs, or delete branches
without explicit maintainer approval.

Resume only from `simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work consolidated on this single branch/PR. The latest local
checkpoint adds reduced scene-owned runtime candidate-buffer rows to the
private GPU contact-candidate packet; runtime scene filtering, GPU
`World::step`, and contact-candidate speedup-gate completion remain future
work.

Before continuing, inspect `git status -sb`, the local commit log, and this
handoff file. Treat any local changes as intentional in-progress state unless
the maintainer says otherwise.

## Resumed Continuation Handoff (2026-06-12)

Work resumed after the prior stop-only handoff. Continue locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978 (`Advance unified
Newton-barrier runtime and parity evidence`). Keep all remaining PLAN-083 work
on this single branch/PR. Do not push, PR-comment, resolve review threads,
trigger CI, open another PLAN-083 PR, or delete branches without explicit
maintainer approval.

The current local work builds on
`51b4e9a695b Add CUDA swept contact broad-phase packet` with reduced
scene-owned runtime candidate-buffer packet rows. The local branch has
unpublished work ahead of `origin/simx/plan083-gpu-contact-candidate-packet`;
inspect `git status -sb` and the local commit log before continuing.

## Scene-Owned Runtime Candidate-Buffer Checkpoint (2026-06-12)

Continue from `simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work on this single branch/PR. Do not push, PR-comment,
resolve review threads, trigger CI, open another PLAN-083 PR, or delete
branches without explicit maintainer approval. Before any future push, merge
latest `origin/main` into this published branch, rerun the required gates, and
push only with explicit approval.

This checkpoint adds reduced scene-owned runtime candidate-buffer rows to the
private GPU contact-candidate packet. The benchmark builds one DART `World`
deformable surface, extracts node start/end states and surface triangles from
the `DeformableBody` handle, reuses the CPU motion-aware sweep builder for
compact candidate keys, and evaluates point-triangle and edge-edge
endpoint-distance buffers on CPU and CUDA.

Fresh local packet evidence records one scene body, 512 point-triangle
candidates over 2,560 points and 768 triangles, and 1,536 edge-edge candidates
over 2,304 surface edges. CPU/GPU endpoint-distance parity remains exact within
`5.551115123125783e-17`, but the top-level packet speedup is
`0.017259032907863573x` (`meets_speedup_gate=false`). This is reduced
scene-owned runtime-buffer evidence only. It does not claim runtime scene
filtering, GPU `World::step`, or contact-candidate speedup-gate completion.

Latest local gates:

- focused contact-candidate packet pytest
- focused contact-candidate/GPU-parity/audit pytest trio
- `git diff --check`
- `pixi run lint`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda test-cuda` (8/8)
- `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`
- `pixi run build`
- `pixi run test-unit` (161/161)

## Validated Sweep Broad-Phase Checkpoint (2026-06-12)

Continue from `simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work on this single branch/PR. Do not push, PR-comment,
resolve review threads, trigger CI, open another PLAN-083 PR, or delete
branches without explicit maintainer approval. Before any future push, merge
latest `origin/main` into this published branch, rerun the required gates, and
push only with explicit approval.

This validated checkpoint adds private CUDA swept-AABB sweep-and-prune packets
for point-triangle and edge-edge candidate construction. The packet device-
builds swept primitive AABBs, device-sorts them by the CPU sweep key, and
deterministically counts/scatters compact overlap candidates in CPU sweep
order.

Fresh local packet evidence records `candidate_pair_count=393216`,
`max_result_abs_error=5.551115123125783e-17`, and
`speedup=0.03107190750809115x` (`meets_speedup_gate=false`). The
point-triangle sweep broad-phase row covers 65,536 pair capacity over 256
points and 256 triangles, emits 256 compact candidates, and records
`speedup=0.03107190750809115x`; the edge-edge sweep broad-phase row covers
65,536 pair capacity over 256 edges, emits 128 compact candidates, and records
`speedup=0.03286696141763437x`.

This is private reduced broad-phase evidence only. It does not claim
scene-owned GPU candidate buffers, runtime scene filtering, or
contact-candidate speedup-gate completion.

Validation passed: `pixi run lint`; `git diff --check`; focused
contact-candidate/GPU-parity/audit pytest trio (18 passed);
`pixi run -e cuda build-cuda Release`; focused
`test_contact_candidate_filter_cuda` CTest; `pixi run -e cuda test-cuda`
(8/8, with the LCP CUDA test taking 231.80s);
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`; PLAN-083 GPU
parity/completion-audit checkers; `pixi run build`; and
`pixi run test-unit` (161/161).

## Validated Runtime Sweep-Buffer Checkpoint (2026-06-12)

Continue from `simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work on this single branch/PR. Do not push, PR-comment,
resolve review threads, trigger CI, open another PLAN-083 PR, or delete
branches without explicit maintainer approval. Before any future push, merge
latest `origin/main` into this published branch, rerun the required gates, and
push only with explicit approval.

This validated checkpoint adds private CUDA evaluators for compact
point-triangle and edge-edge candidate buffers produced by the CPU
motion-aware sweep builder. The packet rows recompute endpoint
squared-distance metadata on the GPU for compact runtime sweep candidate keys:
512 point-triangle candidates over 2,560 points and 768 triangles, plus 1,536
edge-edge candidates over 2,304 surface edges.

Fresh local packet evidence records `candidate_pair_count=262144`,
`max_result_abs_error=5.551115123125783e-17`, and
`speedup=0.11821712677569206x` (`meets_speedup_gate=false`). The runtime
point-triangle sweep-buffer row records `speedup=0.11821712677569206x`; the
runtime edge-edge sweep-buffer row records `speedup=0.28057906386249953x`.
This is runtime-buffer parity evidence only. It does not claim GPU
sweep-and-prune broad-phase sorting, scene-owned GPU candidate buffers,
runtime scene filtering, or contact-candidate speedup-gate completion.

Validation passed: focused contact-candidate packet pytest, PLAN-083 GPU
parity/completion-audit checkers, focused contact-candidate/GPU-parity/audit
pytest trio, `pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run build-simulation-tests`, `pixi run test-simulation` (65/65),
`pixi run check-api-boundaries`, `pixi run -e cuda build-cuda Release`,
`pixi run -e cuda test-cuda` (8/8), and
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`.

## Resumed Compacted-Distance Candidate-Buffer Checkpoint (2026-06-12)

Work resumed after the stop-only handoff. Continue from
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work on this single branch/PR. Do not push, PR-comment,
resolve review threads, trigger CI, or open another PLAN-083 PR without
explicit maintainer approval. Before any future push, merge latest
`origin/main` into this published branch, rerun the required gates, and push
only with explicit approval.

This checkpoint builds on
`f7ee131c457 Add swept contact candidate packet parity` and adds compacted
per-candidate distance metadata to the private GPU contact-candidate buffers.
Static point-triangle and edge-edge candidate-mask packets now compact squared
distance metadata next to accepted primitive ids; swept point-triangle and
edge-edge candidate-list packets now compact endpoint squared-distance metadata
next to accepted primitive ids. This is runtime-buffer prerequisite evidence,
not a sweep-and-prune broad-phase, runtime scene filtering, or speedup-gate
completion claim.

Validation for this checkpoint passed focused contact-candidate packet pytest,
`pixi run -e cuda build-cuda Release`, focused
`test_contact_candidate_filter_cuda` CTest,
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`, the PLAN-083 GPU
parity/completion-audit checker pair, the focused
contact-candidate/GPU-parity/completion-audit pytest trio, `pixi run lint`,
`pixi run build`, and `pixi run test-unit` (161/161).

The regenerated contact-candidate packet records `compacted_distance_count`
for all four candidate-construction rows: 192 static point-triangle candidates,
96 static edge-edge candidates, 256 swept point-triangle candidates, and 128
swept edge-edge candidates. The top-level packet still has
`candidate_pair_count=262144`, `max_result_abs_error=0`, and
`speedup=0.32965959230892933x` (`meets_speedup_gate=false`), so the row stays
`in-progress`.

## Resumed Swept-AABB Validation Checkpoint (2026-06-12)

After the stop-only handoff, this session resumed under the active PLAN-083
goal and validated the swept-AABB contact-candidate packet slice locally. Keep
all remaining PLAN-083 work on
`simx/plan083-gpu-contact-candidate-packet` / PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Do not push,
PR-comment, resolve review threads, trigger CI, or open another PLAN-083 PR
without explicit maintainer approval.

Validation for the resumed checkpoint passed focused packet pytest (7 tests),
`pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run -e cuda build-cuda Release`, focused
`test_contact_candidate_filter_cuda` CTest, `pixi run -e cuda test-cuda`
(8/8), and `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`.
Before any future push, merge latest `origin/main` into this published branch,
rerun the required gates, and push only with explicit approval.

## Latest Swept-AABB Candidate-List Checkpoint (2026-06-12)

Implementation continued on `simx/plan083-gpu-contact-candidate-packet`, the
consolidated PLAN-083 branch for PR #2978 (`Advance unified Newton-barrier
runtime and parity evidence`). Keep all remaining PLAN-083 work on this single
branch/PR. Do not push, PR-comment, resolve review threads, trigger CI, or open
another PLAN-083 PR without explicit maintainer approval. Before any future
push, merge latest `origin/main` into this published branch, rerun the required
gates, and push only with explicit approval.

This checkpoint adds private motion-aware swept-AABB point-triangle and
edge-edge candidate-list packets on top of the previous static all-pairs
candidate masks. It adds
`buildSweptPointTriangleContactCandidateMaskCuda()` and
`buildSweptEdgeEdgeContactCandidateMaskCuda()`, masks incident/duplicate pairs,
records minimum endpoint squared-distance metadata, and compacts accepted pair
ids on the device before readback. Focused CUDA unit coverage checks CPU/GPU
parity and invalid start/end input rejection; packet coverage now requires
`candidate_construction.point_triangle_swept_aabb_candidates` and
`candidate_construction.edge_edge_swept_aabb_candidates`.

The regenerated contact-candidate packet reports exact parity for the previous
stencil/all-pairs rows plus a 65,536-pair swept-AABB point-triangle candidate
list and a 65,536-pair swept-AABB edge-edge candidate list. The swept
point-triangle row has `accepted_count=256`, `compacted_count=256`,
`compacted_triangle_count=256`, `max_result_abs_error=0`, and
`speedup=0.5141495691008232x`. The swept edge-edge row has
`accepted_count=128`, `compacted_edge_a_count=128`,
`compacted_edge_b_count=128`, `max_result_abs_error=0`, and
`speedup=0.2686885092103126x`. The top-level contact-candidate packet records
`candidate_pair_count=262144` and `speedup=0.2686885092103126x`
(`meets_speedup_gate=false`).

This remains reduced private packet evidence only. It does not claim
sweep-and-prune broad-phase sorting, runtime scene candidate buffers, or
contact-candidate speedup-gate completion.

## Latest Edge-Edge Candidate-Mask Checkpoint (2026-06-12)

Implementation resumed on `simx/plan083-gpu-contact-candidate-packet`, the
consolidated PLAN-083 branch for PR #2978 (`Advance unified Newton-barrier
runtime and parity evidence`). Keep all remaining PLAN-083 work on this single
branch/PR. Do not push, PR-comment, resolve review threads, trigger CI, or open
another PLAN-083 PR without explicit maintainer approval. Before any future
push, merge latest `origin/main` into this published branch, rerun the required
gates, and push only with explicit approval.

This checkpoint validates the previously dirty edge-edge all-pairs
contact-candidate mask packet. It adds a private
`buildEdgeEdgeContactCandidateMaskCuda()` path, masks self/duplicate/incident
edge pairs, writes the all-pairs accepted mask, and compacts accepted edge-slot
pairs on the device before readback. Focused CUDA unit coverage checks CPU/GPU
parity, incident-edge rejection, and input validation. Benchmark and packet
writer coverage now require
`candidate_construction.edge_edge_all_pairs_mask` with matching accepted and
compacted edge-a/edge-b counts.

The regenerated contact-candidate packet reports exact parity for 65,536
point-triangle stencils, 65,536 edge-edge stencils, a 65,536-pair
point-triangle all-pairs mask, and a 65,536-pair edge-edge all-pairs mask. The
point-triangle mask row has `accepted_count=192`,
`compacted_count=192`, `compacted_triangle_count=192`,
`max_result_abs_error=0`, and `speedup=0.8860940784354572x`. The edge-edge
mask row has `edge_count=256`, `accepted_count=96`,
`compacted_edge_a_count=96`, `compacted_edge_b_count=96`,
`max_result_abs_error=0`, and `speedup=0.6679555059736084x`. The top-level
contact-candidate packet speedup is `0.37765557710841663x`
(`meets_speedup_gate=false`).

This remains reduced private packet evidence only. It does not claim sweep
broad-phase construction, runtime scene candidate-list construction, or
contact-candidate speedup-gate completion.

Validation for this checkpoint passed focused packet pytest (7 tests),
`pixi run python scripts/check_plan083_gpu_parity_packet.py`,
`pixi run python scripts/check_plan083_completion_audit.py`, the focused
contact-candidate/GPU-parity/completion-audit pytest trio (16 passed),
`pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run build-simulation-tests`, `pixi run test-simulation` (65/65),
`pixi run -e cuda build-cuda Release`, focused
`test_contact_candidate_filter_cuda` CTest, `pixi run -e cuda test-cuda`
(8/8), and `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`.

## Latest Continuation Handoff (2026-06-12)

Implementation resumed from the stop-only handoff on the consolidated PLAN-083
branch. Continue locally on `simx/plan083-gpu-contact-candidate-packet` / PR
#2978, and keep all remaining PLAN-083 work on this consolidated branch/PR. Do
not push, PR-comment, resolve review threads, trigger CI, or open another
PLAN-083 PR without explicit maintainer approval.

Current branch state:

- Branch: `simx/plan083-gpu-contact-candidate-packet`
- PR: #2978, `Advance unified Newton-barrier runtime and parity evidence`
- Base: `main`
- Current checkpoint: deterministic block-prefix device compaction for the
  private point-triangle all-pairs candidate-mask packet.
- Last observed pushed origin head:
  `6746b63973d Record point-point barrier Hessian packet evidence`
- The branch has unpublished local checkpoints beyond origin. Inspect
  `git status -sb` and `git log --oneline --decorate -10` for the exact current
  local head before editing or pushing.

The committed checkpoint at `78fd7061732` moves compacted accepted
point/triangle metadata production from host-side scanning after mask readback
to deterministic CUDA-side compaction. This checkpoint replaces the serial
compaction kernel with block-local accepted counts, a small block-offset prefix
pass, and a stable block-local scatter. It preserves deterministic pair order
and keeps the packet writer/test requirement that
`gpu_compacted_count` and `gpu_compacted_triangle_count` match the accepted
count.

Touched surfaces:

- `dart/simulation/compute/cuda/contact_candidate_filter_cuda.cpp`
- `dart/simulation/compute/cuda/contact_candidate_filter_cuda.cu`
- `dart/simulation/compute/cuda/contact_candidate_filter_cuda.cuh`
- `tests/benchmark/simulation/bm_plan083_gpu_contact_candidates.cpp`
- `scripts/write_plan083_gpu_contact_candidate_packet.py`
- `tests/test_plan083_gpu_contact_candidate_packet.py`

This is still reduced private packet evidence only. It does not claim sweep
broad-phase construction, runtime scene candidate-list construction, or
contact-candidate speedup-gate completion. The regenerated packet reports
`pair_count=65536`, `accepted_count=192`, `compacted_count=192`,
`compacted_triangle_count=192`, `max_result_abs_error=0`, candidate-mask
`speedup=0.9911014836006811x`, and top-level contact-candidate packet
`speedup=0.4065799369175524x` (`meets_speedup_gate=false`).

Validation for this checkpoint passed `pixi run lint`, `pixi run build`,
`pixi run test-unit` (161/161), `pixi run build-simulation-tests`,
`pixi run test-simulation` (65/65), `pixi run check-api-boundaries`,
`pixi run python scripts/check_plan083_gpu_parity_packet.py`,
`pixi run python scripts/check_plan083_completion_audit.py`,
`pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
(15 passed), `pixi run -e cuda build-cuda Release`,
`pixi run -e cuda test-cuda` (8/8), and
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`.

Before any future push to #2978, merge latest `origin/main` into this published
branch, rerun the required gates for all accumulated changes, and push only
with explicit maintainer approval.

## Previous Readback-Compaction Handoff (2026-06-11)

Implementation resumed on the consolidated PLAN-083 branch. Continue locally on
`simx/plan083-gpu-contact-candidate-packet` / PR #2978, and keep all remaining
PLAN-083 work on this consolidated branch/PR. Do not push, PR-comment, resolve
review threads, or trigger CI without explicit maintainer approval.

Current local state for a fresh session:

- Branch: `simx/plan083-gpu-contact-candidate-packet`
- PR: #2978, `Advance unified Newton-barrier runtime and parity evidence`
- Base: `main`
- Last observed pushed origin head:
  `6746b63973d Record point-point barrier Hessian packet evidence`
- Earlier local committed checkpoints ahead of origin:
  - `48dcfb515cf Add point-edge barrier Hessian packet parity`
  - `1dfb21b24da Add point-triangle barrier Hessian packet parity`
  - `1022b609f8c Add point-triangle Hessian PSD packet parity`
  - `c1e6e73b2cf Add point-point and point-edge Hessian PSD packet parity`
  - `e41941db91c Add off-diagonal sparse block assembly packet parity`
  - `bed8ab7569b Add point-triangle candidate mask packet parity`
- Current checkpoint: compacted point-triangle candidate-pair metadata on top
  of `bed8ab7569b`. It records accepted point/triangle index lists after GPU
  mask readback and teaches the contact-candidate packet writer/test to require
  `gpu_compacted_count == accepted_count`.

This checkpoint is reduced private packet evidence. It strengthens the
brute-force all-pairs point-triangle candidate-mask packet with readback
compaction metadata, but it still does not claim sweep broad-phase
construction, runtime scene candidate-list construction, or completion of the
contact-candidate speedup gate. The regenerated packet reports
`pair_count=65536`, `accepted_count=192`, `compacted_count=192`,
`max_result_abs_error=0`, candidate-mask `speedup=1.5943055078570016x`, and
top-level contact-candidate packet `speedup=0.35669978298935295x`
(`meets_speedup_gate=false`).

Validation for this checkpoint passed focused contact-candidate packet pytest
(5 tests), `pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run -e cuda build-cuda Release`, `pixi run -e cuda test-cuda` (8/8),
and `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`. Before any
future push, merge latest `origin/main` into this published branch, rerun the
required gates, and push only with explicit maintainer approval. Keep all
remaining PLAN-083 work consolidated on PR #2978; do not open another PLAN-083
PR.

## Earlier Active Continuation Handoff (2026-06-11)

Implementation resumed after the stop-only handoff. Continue locally on
`simx/plan083-gpu-contact-candidate-packet` / PR #2978, and keep all remaining
PLAN-083 work on this consolidated branch/PR. Do not push, PR-comment, resolve
review threads, or trigger CI without explicit maintainer approval.

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
  - `e41941db91c Add off-diagonal sparse block assembly packet parity`
- Current checkpoint: the private CUDA point-triangle all-pairs candidate-mask
  slice touches the contact-candidate CUDA wrapper/kernel/header, focused CUDA
  unit test, benchmark packet, packet writer/test, GPU parity sidecar,
  completion audit, dashboard, and these dev-task handoff docs. Keep it on PR
  #2978 and do not open a separate PR.

The preceding committed checkpoint adds reduced off-diagonal sparse-block
assembly evidence on the same branch. The current checkpoint adds a private
CUDA point-triangle all-pairs candidate mask to the
contact-candidate packet, extending the earlier preassembled
point-triangle/edge-edge stencil filters without claiming sweep broad-phase,
compacted runtime scene candidate lists, or the speedup gate. A fresh session
should inspect `git status -sb`, `git log --oneline`, and PR #2978 before
editing.

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

The contact-candidate continuation adds:

- `buildPointTriangleContactCandidateMaskCuda()` and a CUDA all-pairs
  point-triangle mask kernel.
- Focused CUDA unit parity for a two-point/two-triangle candidate mask.
- Contact-candidate benchmark rows
  `BM_Plan083PointTriangleCandidateMaskCpu/Cuda`.
- Packet-writer/test coverage for
  `candidate_construction.point_triangle_all_pairs_mask`.
- Tracked GPU parity, completion-audit, dashboard, and dev-task sidecar updates.

Focused evidence gathered so far for the candidate-mask checkpoint:

- `pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py -q`
  passed.
- `pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
  passed (13 tests).
- `pixi run lint` passed.
- `pixi run build` passed.
- `pixi run test-unit` passed (161/161).
- `pixi run -e cuda build-cuda Release` passed after fixing the benchmark
  fixture helper.
- `pixi run -e cuda test-cuda` passed, including
  `test_contact_candidate_filter_cuda`.
- `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet` passed and wrote
  the generated contact-candidate packet.
- `pixi run python scripts/check_plan083_gpu_parity_packet.py` passed.
- Focused packet pytest for the contact-candidate and GPU parity packet tests
  passed.

The generated contact-candidate packet reported exact parity for the new
all-pairs row with `pair_count=65536`, `point_count=256`,
`triangle_count=256`, `accepted_count=192`, `compacted_count=192`,
`max_result_abs_error=0`, `speedup=1.5943055078570016x`, and
`meets_speedup_gate=true`. The top-level contact-candidate packet reported
`speedup=0.35669978298935295x`, so the row stays `in-progress`.

Likely next steps:

1. Inspect `git status -sb`, `git log --oneline`, and PR #2978 before editing.
2. Continue with runtime scene filtering, full runtime sparse Hessian graph
   construction and assembly, direct/global sparse factorization, nonlinear
   equality constraints, GPU `World::step` assembly/solve integration, or speedup-gate work only as
   honest `in-progress` evidence.
3. Before any push, merge latest
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
point-triangle, point-point, point-edge, and edge-edge barrier-Hessian packets
and point-triangle/point-point/point-edge/edge-edge Hessian PSD projection as in-progress
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

## Scene Sparse Graph Edge-Dedup Checkpoint (2026-06-13)

Work continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978,
without pushing or mutating GitHub. This local slice added a reduced
scene-owned sparse graph unique-edge assembly row to the private Newton
assembly/solve packet. The benchmark builds the same DART `World` deformable
surface as the scene sparse graph row, appends one reversed duplicate triangle,
marks canonical surface-edge keys on GPU, and emits one 6x6 sparse block per
unique canonical edge in deterministic order.

The generated packet records `scene_node_count=2560`,
`scene_triangle_count=769`, `edge_slot_count=2307`,
`unique_edge_count=2304`, `duplicate_edge_slot_count=3`,
`block_count=2304`, `block_entry_count=82944`,
`max_result_abs_error=4.440892098500626e-16`, and
`speedup=0.0010437835446294721x` for the reduced unique-edge row. The top-level
assembly/solve row records `max_result_abs_error=3.637978807091713e-11`,
`residual_norm=3.70262388185975e-13`, and
`speedup=0.0010437835446294721x`, so `assembly-linear-solve` remains
`in-progress`.

Validation passed for this checkpoint:

- `pixi run python -m py_compile scripts/write_newton_assembly_solve_packet.py`
- `pixi run python -m pytest tests/test_newton_assembly_solve_packet.py -q`
- `pixi run -e cuda build-cuda`
- `pixi run -e cuda bash -lc 'build/cuda/cpp/Release/bin/test_newton_assembly_solve_cuda --gtest_filter=NewtonAssemblySolveCuda.MatchesCpuSceneSparseGraphAssembly:NewtonAssemblySolveCuda.MatchesCpuSceneSparseGraphUniqueAssembly:NewtonAssemblySolveCuda.RejectsInvalidSceneSparseGraphInputs'`
- `pixi run -e cuda python scripts/write_newton_assembly_solve_packet.py`
- `pixi run python -m json.tool docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json >/dev/null`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_newton_assembly_solve_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
- `git diff --check`
- `pixi run lint`

This reduces the sparse graph dedup evidence gap only for a private reduced
packet. Production sparse Hessian graph construction/assembly, unbounded
production direct/global sparse factorization, production nonlinear equality
convergence policy/solving, GPU `World::step`, paper-scale assets, and speedup
gates remain future evidence.

## Resume Guidance

1. Resume only from `simx/plan083-gpu-contact-candidate-packet` and PR #2978.
2. Inspect local status before editing, committing, or pushing. The current
   branch is the single consolidated #2978 branch and now includes the reduced
   sampled rigid-curved CCD/line-search rows, scene-owned runtime CCD rows plus
   a reduced combined scene runtime CCD line-search row, and scene-owned sparse
   graph assembly/solve rows, including the reduced unique-edge dedup row, on
   top of the earlier
   contact-candidate, barrier/friction, assembly/solve, and scene-parity packet
   checkpoints.
3. Check hosted CI and new review comments before editing. Do not reply to bot
   comments.
4. Continue on the same PR with the remaining runtime/parity gaps: runtime
   scene filtering, analytic curved CCD, production scene-level line search
   inside `World::step`, broader production sparse Hessian graph
   construction/assembly, direct/global sparse factorization, nonlinear
   equality constraints, GPU `World::step`
   integration, broader sparse barrier/contact assembly, and packet speedup
   gates.
5. Keep plan/dev-task text honest: packet rows may move from `planned` to
   `in-progress` only with corresponding runtime or packet evidence, and the
   dev-task folder should not be retired until the remaining in-progress work
   is either completed or explicitly relocated by a maintainer.
