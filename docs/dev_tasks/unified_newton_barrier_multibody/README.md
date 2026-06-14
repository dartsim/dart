# Unified Newton-Barrier Multibody - Dev Task

## Current Status

Latest `World::step` inter-body candidate witness follow-up (2026-06-14): work
continues locally on `simx/plan083-worldstep-gpu-candidate-bridge` (local-only,
no PR), stacked on the milestone checkpoint PR #3000 branch. This slice extends
the private GPU contact-candidate packet's `world_step_surface_contact` witness
with an `inter_body` sub-witness: inter-body deformable surface-contact candidate
counters from a dedicated minimal two-body `World::step` (a moving-point body
crossing a stationary triangle obstacle). The writer CPU/GPU-parity-checks the
inter-body candidate builds, capacity, rejected pairs, point-triangle, and
edge-edge candidates, requires a nonzero build and rejection pressure, and
enforces `capacity == emitted + rejected`. Fresh CUDA evidence records
`inter_body.candidate_builds=33`, `candidate_pair_capacity=528`,
`candidate_rejected_pairs=495`, `point_triangle_candidates=33`, and
`edge_edge_candidates=0`; the recorded `max_result_abs_error` and sub-1x speedup
(`meets_speedup_gate=false`) are unchanged by this slice. This proves a reduced
inter-body deformable `World::step` candidate filter-pressure witness only; it
does not prove production runtime scene filtering inside `World::step`, GPU
`World::step` contact-candidate construction, inter-body CCD parity, or a
top-level runtime speedup claim. Validation passed: `pixi run python -m pytest
tests/test_plan083_gpu_contact_candidate_packet.py
tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py
-q`, the CUDA build + `pixi run -e cuda python
scripts/write_plan083_gpu_contact_candidate_packet.py`,
`scripts/check_plan083_gpu_parity_packet.py`,
`scripts/check_plan083_completion_audit.py`, `pixi run lint`, and
`git diff --check`.

Latest `World::step` self-surface CCD witness follow-up (2026-06-14): the
checkpoint branch `simx/plan083-worldstep-contact-filter-stats` has been pushed
and opened as the milestone checkpoint **PR #3000** against `main` (with explicit
maintainer approval); it is an honest work-in-progress checkpoint, not a
completion claim. Work continues locally on
`simx/plan083-worldstep-gpu-candidate-bridge` (local-only, no PR). This slice
extends the private GPU contact-candidate packet's reduced scene-owned filtered
candidate-buffer `world_step_surface_contact` witness with the matching
self-surface continuous-collision (CCD) counters from the same generated DART
`World::step`. The packet writer now CPU/GPU-parity-checks the runtime CCD
point-triangle checks, edge-edge checks, hits, limited steps, and zero-step
counts, requires nonzero CCD checks, and rejects CCD hits exceeding total checks.
Fresh CUDA packet evidence records
`world_step_surface_contact.ccd_point_triangle_checks=47488`,
`ccd_edge_edge_checks=101888`, `ccd_hits=39168`, `ccd_limited_steps=1`, and
`ccd_zero_step_count=16384`; the recorded `max_result_abs_error` and sub-1x
speedup (`meets_speedup_gate=false`) are unchanged by this slice. This proves a
richer reduced packet-level bridge between GPU filtered candidate construction
evidence and a same-scene runtime `World::step` self-surface CCD witness only. It
does not prove production runtime scene filtering inside `World::step`, GPU
`World::step` contact-candidate construction, or a top-level runtime speedup
claim. Validation passed: `pixi run python -m pytest
tests/test_plan083_gpu_contact_candidate_packet.py
tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py
-q`, the CUDA build + `pixi run -e cuda python
scripts/write_plan083_gpu_contact_candidate_packet.py`,
`scripts/check_plan083_gpu_parity_packet.py`,
`scripts/check_plan083_completion_audit.py`, `pixi run lint`, and
`git diff --check`.

Latest `World::step` GPU contact-candidate bridge follow-up (2026-06-14):
work continues locally on `simx/plan083-worldstep-gpu-candidate-bridge`,
stacked on the clean checkpoint branch
`simx/plan083-worldstep-contact-filter-stats` at `2a32080431d`. The branch
already contains `origin/main` at `9de4ac6af87`, has not been pushed, and has
no open PR. Keep `simx/plan083-worldstep-contact-filter-stats` available as
the checkpoint/milestone PR candidate, and push/open this stacked follow-up
branch only after explicit maintainer approval. Do not push, PR-comment,
resolve review threads, trigger CI, open or close PRs, delete branches, or
claim production GPU `World::step` coverage without explicit maintainer
approval.

This slice extends the private GPU contact-candidate packet's reduced
scene-owned filtered candidate-buffer row with a matching `World::step`
self-surface contact-filter diagnostic witness from the same generated DART
`World` surface. The packet writer now rejects missing or mismatched
`World::step` diagnostic counters and requires the runtime per-build pair
capacity to match the reduced filtered packet capacity. Fresh CUDA packet
evidence records `pair_capacity=7274496`, `candidate_count=2048`,
`rejected_count=7272448`,
`world_step_surface_contact.candidate_builds=33`,
`world_step_surface_contact.candidate_pair_capacity_per_build=7274496`,
`world_step_surface_contact.candidate_pair_capacity=240058368`,
`world_step_surface_contact.rejected_pairs=239908992`,
`world_step_surface_contact.point_triangle_candidates=47488`,
`world_step_surface_contact.edge_edge_candidates=101888`,
`max_result_abs_error=5.551115123125783e-17`, and
`speedup=0.09728698147728942x` (`meets_speedup_gate=false`). The top-level
contact-candidate packet records `max_result_abs_error=5.551115123125783e-17`
and `speedup=0.024982985278583135x` (`meets_speedup_gate=false`).

This proves a reduced packet-level bridge between GPU filtered candidate
construction evidence and a same-scene runtime `World::step` self-surface
filter-pressure witness only. It does not prove production runtime scene
filtering inside `World::step`, GPU `World::step` contact-candidate
construction, or a top-level runtime speedup claim.

Current validation passed:

- `pixi run python -m py_compile scripts/write_plan083_gpu_contact_candidate_packet.py`
- `pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py -q`
- `pixi run -e cuda cmake --build build/cuda/cpp/Release --target bm_plan083_gpu_contact_candidates --parallel`
- `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`
- `pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
- `pixi run python -m json.tool docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json >/dev/null`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run lint`
- `git diff --check`

Latest `World::step` contact-filter stats follow-up (2026-06-14): after PR
#2978 merged and the old remote branch was deleted, work continues locally on
the new branch `simx/plan083-worldstep-contact-filter-stats`, created from
`simx/plan083-runtime-scene-filter-followup` at `fdb649da6c4`. The branch
already contains `origin/main` at `9de4ac6af873`, has not been pushed, and has
no open PR. Keep `simx/plan083-runtime-scene-filter-followup` available as the
possible checkpoint/milestone PR branch, and push/open this follow-up branch
only after explicit maintainer approval. Do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This slice promotes runtime contact-filter pressure accounting into
`World::step` diagnostics. The built-in deformable diagnostics now report
candidate pair capacity and rejected pair counts for self-surface, inter-body
deformable, static rigid surface CCD, and moving rigid surface CCD candidate
paths. The counters are exposed through C++, dartpy/stubs, focused deformable
unit assertions, `bm_deformable_body`, the PLAN-083 CPU scene benchmark, and
the CPU packet writer's serialization/consistency checks.

This proves runtime `World::step` candidate-filter pressure accounting only. It
does not prove GPU `World::step` contact-candidate construction, production
GPU runtime filtering, a speedup gate, or paper-scale scene behavior.

Current validation passed:

- `pixi run python scripts/cmake_build.py --build-dir build/default/cpp/Release --target dart-simulation --target test_deformable_body --target bm_deformable_body --target bm_plan083_cpu_scene_corpus --target dartpy`
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_deformable_body$'`
- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py -q`
- `pixi run lint`
- `pixi run build`
- `pixi run -e cuda test-all`
- `git diff --check`

Latest filtered scene-runtime contact-candidate follow-up (2026-06-13): work
continues locally on `simx/plan083-runtime-scene-filter-followup`, stacked on
the clean checkpoint branch `simx/plan083-rigid-curved-ccd-worldstep-followup`
at `4c7c0ca1614`. This branch has not been pushed and has no open PR. Keep the
checkpoint branch available as the possible milestone PR branch, and push/open
this follow-up branch only after explicit maintainer approval. Do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

This slice adds a reduced end-to-end scene-owned contact-candidate row to the
private GPU packet. The new benchmark builds point-triangle and edge-edge
candidate ids with the existing GPU scene sweep filter, then feeds those
GPU-compacted ids directly into the GPU candidate-buffer distance kernels. The
packet writer now rejects missing/mismatched filtered-row counters and requires
the filtered row to prove real rejection pressure. Fresh CUDA packet evidence
records `pair_capacity=7274496`, `candidate_count=2048`,
`accepted_count=2048`, `rejected_count=7272448`,
`point_triangle_candidate_count=512`, `edge_edge_candidate_count=1536`,
`max_result_abs_error=5.551115123125783e-17`, and
`speedup=0.07484292773973288x` (`meets_speedup_gate=false`). The top-level
contact-candidate packet records `max_result_abs_error=5.551115123125783e-17`
and `speedup=0.020550464331653258x` (`meets_speedup_gate=false`).

This remains reduced packet evidence only. It does not prove production runtime
scene filtering inside `World::step`, GPU `World::step` contact-candidate
construction, or a top-level runtime speedup claim.

Current validation passed:

- `pixi run python -m py_compile scripts/write_plan083_gpu_contact_candidate_packet.py`
- `pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py -q`
- `pixi run -e cuda cmake --build build/cuda/cpp/Release --target bm_plan083_gpu_contact_candidates --parallel`
- `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`
- `pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
- `pixi run python -m json.tool docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json >/dev/null`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run lint`
- `git diff --check`

Latest scene direct-sparse assembly/solve follow-up (2026-06-13): after PR
#2978 merged and the old remote branch was deleted, work continues locally on
`simx/plan083-rigid-curved-ccd-worldstep-followup`. This branch has merged the
latest `origin/main` at `9de4ac6af873` through local merge commit
`46a4993def6f`; it has not been pushed and has no open PR. Keep the checkpoint
branch `simx/plan083-curved-ccd-reference-packet` available as the possible
milestone PR branch, but keep this follow-up branch local unless the maintainer
approves a push or new PR. Do not push, PR-comment, resolve review threads,
trigger CI, open or close PRs, delete branches, or claim unrelated PLAN-091
packets without explicit maintainer approval.

This slice expands the private Newton assembly/solve packet's reduced
scene-runtime direct sparse factor-solve row from three selected scene nodes to
eight selected scene nodes, exercising the existing 48-DOF direct-sparse cap and
six selected scene edge blocks. The packet writer now rejects inconsistent
direct-sparse DOF, active-DOF, selected-node, and 6x6 block-entry accounting.
Fresh CUDA packet evidence records `selected_scene_node_count=8`,
`selected_scene_edge_pair_count=6`, `dof_count=48`, `block_count=6`,
`block_entry_count=216`,
`scene_runtime_direct_sparse_factor_solve.max_result_abs_error=4.551414082424e-18`,
`residual_norm=4.596089064510693e-17`, `step_norm=0.15520625719772183`, and
`speedup=0.006653527066632866x` (`meets_speedup_gate=false`). The top-level
assembly/solve packet records
`max_result_abs_error=3.2741809263825417e-11`,
`residual_norm=3.7103006033468554e-13`, and
`speedup=0.001142831863949895x` (`meets_speedup_gate=false`).

This remains reduced packet evidence only. It does not prove unbounded
production direct/global sparse factorization, full runtime sparse Hessian graph
construction and assembly, GPU `World::step` assembly/solve integration, or a
top-level runtime speedup claim.

Current validation passed:

- `pixi run python -m py_compile scripts/write_newton_assembly_solve_packet.py`
- `pixi run python -m pytest tests/test_newton_assembly_solve_packet.py -q`
- `pixi run python -m json.tool docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json >/dev/null`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run -e cuda bm-newton-assembly-solve-packet`
- `pixi run lint`

Latest interval-reference CCD line-search follow-up (2026-06-13): work now
continues locally on `simx/plan083-rigid-curved-ccd-worldstep-followup`, branched
from local checkpoint branch `simx/plan083-curved-ccd-reference-packet`. The
checkpoint branch remains available as the potential milestone PR branch; this
follow-up branch is local-only and has not been pushed. Keep all remaining
PLAN-083 follow-up work consolidated on the follow-up branch unless the
maintainer chooses to publish the checkpoint branch first; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

This slice adds parameter-box interval reference counters beside the existing
continuous curved rigid IPC ACCD prefix counters for sampled rigid-curved
point-triangle and edge-edge CCD/line-search rows. The full CUDA packet run
records 512 interval reference prefix pairs, 384 interval reference hits, zero
hit mismatches, zero sampled-interval-reference overshoot, and
`max_sampled_interval_reference_conservative_gap=0.041665077209472656`. The
top-level packet still covers 266,240 pairs and 1,183,744 sampled/evaluated
segments with `max_result_abs_error=5.551115123125783e-17` and
`speedup=0.43369838227282087x` (`meets_speedup_gate=false`).

This is still reduced packet/reference evidence only. It does not prove a
production analytic curved CCD backend, production scene-level line search
inside `World::step`, GPU `World::step`, or any runtime speedup claim.

Current validation passed:

- `pixi run python -m py_compile scripts/write_plan083_gpu_ccd_line_search_packet.py`
- `pixi run python -m pytest tests/test_plan083_gpu_ccd_line_search_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
- `pixi run python -m json.tool docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json >/dev/null`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `git diff --check`
- `pixi run -e cuda python scripts/write_plan083_gpu_ccd_line_search_packet.py --benchmark-min-time 0.01s --benchmark-repetitions 3 --benchmark-json .benchmark_results/plan083/gpu/ccd_line_search_benchmark.json --output .benchmark_results/plan083/gpu/ccd_line_search_parity.json`
- `pixi run lint`
- `pixi run test-all`
- `pixi run -e cuda test-all`

Previous curved-reference CCD line-search packet checkpoint (2026-06-13): after
PR #2978 merged into `origin/main` at `5bf908100da4` and the remote
`simx/plan083-gpu-contact-candidate-packet` branch was deleted, work moved to
the new local branch `simx/plan083-curved-ccd-reference-packet` from the
preserved local continuation stack. The clean branch is based at
`origin/main`/`5bf908100da4`, is 42 commits ahead, ends at the local
`Add rigid-curved CCD reference counters` checkpoint, and backup branch
`simx/plan083-curved-ccd-reference-packet-stack` preserves the pre-cleanup
local stack. The older pre-merge preservation stash remains `stash@{0}`. Keep
all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

This checkpoint tightens the private CCD/line-search packet's sampled
rigid-curved rows. The benchmark now builds rigid-curved point-triangle and
edge-edge trajectories from rigid IPC pose interpolation, reduces sampled
segment hits to the segment lower bound for conservativeness, and records a
continuous curved rigid IPC ACCD reference prefix for each sampled family. The
latest generated packet records 512 reference prefix pairs, 384 reference hits,
zero hit mismatches, zero sampled-reference overshoot, and
`max_sampled_reference_conservative_gap=0.04166632272181661`. The top-level
packet still covers 266,240 pairs and 1,183,744 sampled/evaluated segments,
with `max_result_abs_error=5.551115123125783e-17` and
`speedup=0.45472262951920356x` (`meets_speedup_gate=false`).

This is still reduced packet/reference evidence only. It does not prove a
production analytic curved CCD backend, production scene-level line search
inside `World::step`, GPU `World::step`, or any runtime speedup claim.

Current validation passed:

- `pixi run python -m py_compile scripts/write_plan083_gpu_ccd_line_search_packet.py`
- `pixi run python -m pytest tests/test_plan083_gpu_ccd_line_search_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
- `pixi run python -m json.tool docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json >/dev/null`
- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `git diff --check`
- `pixi run test-all`
- `pixi run -e cuda cmake --build build/cuda/cpp/Release --target bm_plan083_gpu_ccd_line_search --parallel`
- `pixi run -e cuda python scripts/write_plan083_gpu_ccd_line_search_packet.py`
- `pixi run -e cuda test-all`

Latest precession external surface CCD sidecar CPU packet checkpoint
(2026-06-13): after fetching and locally merging `origin/main` at
`a3509146afe` into `simx/plan083-gpu-contact-candidate-packet`, work
continued locally for PR #2978. Keep all remaining PLAN-083 follow-up work
consolidated there; do not push, PR-comment, resolve review threads, trigger
CI, open or close PRs, delete branches, or claim unrelated PLAN-091 packets
without explicit maintainer approval.

This checkpoint upgrades the reduced `unb-fig-23` precession CPU benchmark row
from a rigid-only runtime smoke packet to a rigid IPC `World::step` rolling
wheel step plus a deformable IPC `World::step` external surface CCD sidecar.
The precession row still validates 2 rigid bodies, 1 dynamic wheel body, 1
active contact constraint, 1 active friction constraint, zero final equality
residual, zero ground clearance, and a spin rate near 8.09 rad/s, while
`scripts/write_plan083_cpu_scene_packet.py` now requires a 3-body/31-node
external CCD sidecar with positive inter-body, static-rigid, and moving-rigid
surface CCD witness counters for `scene=precession`. The latest median packet
records 68 self-surface candidate builds, 660 point-triangle CCD checks, 1,224
edge-edge CCD checks, 67 inter-body candidate builds, 33 inter-body hits and
one limited step, 35 static-rigid candidate builds, 68 point-triangle checks,
102 edge-edge checks, 34 static-rigid hits and one limited step, plus 3
moving-rigid candidate builds, 2 point-triangle checks, 41 edge-edge checks, 1
moving-rigid hit, and one limited step with `failed_steps=0`.

This is still a reduced rolling-wheel runtime smoke packet plus sidecar
evidence only. It does not prove angular-velocity sweeps, a rolling-contact
model, the paper-scale 5-body/4k-node reproduction, accepted Table 2 counts,
production runtime scene filtering, analytic curved CCD, GPU `World::step`,
accepted reference timings, or any speedup gate.

Current validation passed:

- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py -q`
- `pixi run bm-plan083-cpu-precession-packet`
- `pixi run python scripts/check_plan083_cpu_scene_corpus.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py tests/test_plan083_cpu_scene_corpus.py tests/test_plan083_completion_audit.py -q`
- `git diff --check`
- `pixi run lint`
- `pixi run build`
- `pixi run -e cuda test-all`

Latest umbrella external surface CCD sidecar CPU packet checkpoint
(2026-06-13): work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint upgrades the reduced `unb-fig-04` umbrella CPU benchmark row
from a rigid-only runtime smoke packet to a rigid IPC `World::step` umbrella
step plus a deformable IPC `World::step` external surface CCD sidecar. The
umbrella row still validates 4 rigid bodies, 3 dynamic bodies, 2
point-connected ribs, 1 revolute hinge, and 8 active articulation constraints,
while `scripts/write_plan083_cpu_scene_packet.py` now requires a
3-body/31-node external CCD sidecar with positive inter-body, static-rigid, and
moving-rigid surface CCD witness counters for `scene=umbrella`. The latest
median packet records 68 self-surface candidate builds, 660 point-triangle CCD
checks, 1,224 edge-edge CCD checks, 67 inter-body candidate builds, 33
inter-body hits and one limited step, 35 static-rigid candidate builds, 68
point-triangle checks, 102 edge-edge checks, 34 static-rigid hits and one
limited step, plus 3 moving-rigid candidate builds, 2 point-triangle checks, 41
edge-edge checks, 1 moving-rigid hit, and one limited step with
`failed_steps=0`.

This is still a reduced rigid umbrella smoke packet plus sidecar evidence only.
It does not prove umbrella-owned deformable contact, cloth shrinking,
wrinkling, sliding constraints, rod/codimensional paper-scale coupling,
accepted Table 2 counts, production runtime scene filtering, analytic curved
CCD, GPU `World::step`, accepted reference timings, or any speedup gate.

Current validation passed:

- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py -q`
- `pixi run bm-plan083-cpu-umbrella-packet`
- `pixi run python scripts/check_plan083_cpu_scene_corpus.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py tests/test_plan083_cpu_scene_corpus.py tests/test_plan083_completion_audit.py -q`
- `git diff --check`
- `pixi run lint`
- `pixi run build`
- `pixi run -e cuda test-all`

Latest hanging-bridge external surface CCD sidecar CPU packet checkpoint
(2026-06-13): work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint upgrades the reduced `unb-fig-02` hanging-bridge CPU benchmark
row from a rigid-only runtime smoke packet to a rigid IPC `World::step` bridge
step plus a deformable IPC `World::step` external surface CCD sidecar. The
bridge row still validates 7 rigid bodies, 5 dynamic bodies, 4 point-connection
joints, and 12 active articulation constraints, while
`scripts/write_plan083_cpu_scene_packet.py` now requires a 3-body/31-node
external CCD sidecar with positive inter-body, static-rigid, and moving-rigid
surface CCD witness counters for `scene=hanging_bridge`. The latest median
packet records 68 self-surface candidate builds, 660 point-triangle CCD checks,
1,224 edge-edge CCD checks, 67 inter-body candidate builds, 33 inter-body hits
and one limited step, 35 static-rigid candidate builds, 68 point-triangle
checks, 102 edge-edge checks, 34 static-rigid hits and one limited step, plus
3 moving-rigid candidate builds, 2 point-triangle checks, 41 edge-edge checks,
1 moving-rigid hit, and one limited step with `failed_steps=0`.

This is still a reduced rigid bridge smoke packet plus sidecar evidence only.
It does not prove bridge-owned deformable contact, 10 elastic rods, 40 rigid
boards, codimensional paper-scale coupling, accepted Table 2 counts, production
runtime scene filtering, analytic curved CCD, GPU `World::step`, accepted
reference timings, or any speedup gate.

Current validation passed:

- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py -q`
- `pixi run bm-plan083-cpu-hanging-bridge-packet`
- `pixi run python scripts/check_plan083_cpu_scene_corpus.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py tests/test_plan083_cpu_scene_corpus.py tests/test_plan083_completion_audit.py -q`
- `git diff --check`
- `pixi run lint`
- `pixi run build`
- `pixi run -e cuda test-all`

Latest ABD/FEM external surface CCD sidecar CPU packet checkpoint
(2026-06-13): work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint repairs and tightens the reduced
`abd-fem-coupling` CPU benchmark row after the deformable sidecar inherited the
lying-flat external CCD witnesses. The benchmark now treats the reduced FEM
cloth plus two inter-body witness deformables as the expected 3-body/31-node
deformable sidecar inside
`BM_Plan083CpuScene_abd_fem_coupling_reduced_side_by_side_step`, and
`scripts/write_plan083_cpu_scene_packet.py` requires positive inter-body,
static-rigid, and moving-rigid surface CCD witness counters for
`scene=abd_fem_coupling`. The latest median packet records 68 self-surface
candidate builds, 660 point-triangle CCD checks, 1,224 edge-edge CCD checks,
67 inter-body candidate builds, 33 inter-body hits and one limited step, 35
static-rigid candidate builds, 68 point-triangle checks, 102 edge-edge checks,
34 static-rigid hits and one limited step, plus 3 moving-rigid candidate
builds, 2 point-triangle checks, 41 edge-edge checks, 1 moving-rigid hit, and
one limited step with `failed_steps=0`.

This is still reduced affine pair runtime-step, deformable IPC sidecar, and
affine/deformable coupled contact micro-solve evidence only. It does not prove
full runtime affine/FEM coupling, 1.1M-triangle assets, accepted reference
timings, paper-scale reproduction, GPU `World::step`, or any speedup gate.

Current validation passed:

- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py -q`
- `pixi run bm-plan083-cpu-abd-fem-coupling-packet`
- `pixi run python scripts/check_plan083_cpu_scene_corpus.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py tests/test_plan083_cpu_scene_corpus.py tests/test_plan083_completion_audit.py -q`
- `git diff --check`
- `pixi run lint`
- `pixi run build`
- `pixi run -e cuda test-all`

Latest Candy static/moving-rigid surface CCD CPU packet checkpoint (2026-06-13):
work continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR
#2978. Keep all remaining PLAN-083 follow-up work consolidated there; do not
push, PR-comment, resolve review threads, trigger CI, open or close PRs,
delete branches, or claim unrelated PLAN-091 packets without explicit
maintainer approval.

This checkpoint upgrades the reduced `unb-fig-22` Candy CPU benchmark row from
static-rigid-only external CCD evidence to reduced static-rigid and
moving-rigid surface CCD witnesses inside
`BM_Plan083CpuScene_candy_reduced_world_step`. The packet now steps the
reduced deformable cloth/static shell scene plus one isolated static CCD
witness box/point pair and one isolated moving CCD witness box/point pair
through deformable IPC `World::step`, and
`scripts/write_plan083_cpu_scene_packet.py` requires positive static-rigid and
moving-rigid candidate, point-triangle CCD check, hit, and limited-step
counters for `scene=candy`. The latest median packet records 3 self-surface
candidate builds, 64 point-triangle CCD checks, 207 edge-edge CCD checks, 69
static-rigid surface CCD candidate builds, 72 point-triangle checks, 72 hits,
and one limited step, plus 37 moving-rigid surface CCD candidate builds, 184
point-triangle checks, 184 hits, and one limited step with `failed_steps=0`.

This is still a reduced CPU benchmark packet only. It does not prove affine
body packing, twisted shell geometry, cloth self-contact parity, paper-scale
Candy, production runtime scene filtering, analytic curved CCD, GPU
`World::step`, full runtime affine/FEM coupling, accepted reference timings, or
any speedup gate.

Current validation passed:

- `pixi run bm-plan083-cpu-candy-packet`
- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py -q`
- `pixi run python scripts/check_plan083_cpu_scene_corpus.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py tests/test_plan083_cpu_scene_corpus.py tests/test_plan083_completion_audit.py -q`
- `git diff --check`
- `pixi run lint`
- `pixi run build`
- `pixi run -e cuda test-all`

Latest lying-flat inter-body/static/moving-rigid surface CCD CPU packet
checkpoint (2026-06-13): work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint upgrades the reduced `unb-fig-01` lying-flat CPU benchmark row
from static/moving-rigid-only external evidence to inter-body, static-rigid,
and moving-rigid surface CCD witnesses inside
`BM_Plan083CpuScene_lying_flat_reduced_world_step`. The packet now steps the
reduced deformable cloth plus two reduced inter-body witness deformables
through one static CCD witness obstacle and one moving CCD witness obstacle in
the deformable IPC `World::step` path, and
`scripts/write_plan083_cpu_scene_packet.py` requires positive inter-body,
static-rigid, and moving-rigid candidate, CCD check, hit, and limited-step
counters for `scene=lying_flat`. The latest median packet records 68
self-surface candidate builds, 660 point-triangle CCD checks, 1,224 edge-edge
CCD checks, 33 inter-body surface CCD hits with one inter-body limited step, 34
static-rigid surface CCD hits with one static-rigid limited step, and 1
moving-rigid surface CCD hit with one moving-rigid limited step and
`failed_steps=0`.

This is still a reduced CPU benchmark packet only. It does not prove
paper-scale lying-flat coupling, production runtime scene filtering, analytic
curved CCD, GPU `World::step`, full runtime affine/FEM coupling, accepted
reference timings, or any speedup gate.

Current validation passed:

- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py -q`
- `pixi run bm-plan083-cpu-lying-flat-packet`
- `pixi run python scripts/check_plan083_cpu_scene_corpus.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py tests/test_plan083_cpu_scene_corpus.py tests/test_plan083_completion_audit.py -q`
- `git diff --check`
- `pixi run lint`
- `pixi run build`
- `pixi run -e cuda test-all`

Latest mixed external surface CCD CPU packet checkpoint (2026-06-13): work
continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978.
Keep all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

This checkpoint adds a dedicated reduced `unb-alg-barriers` CPU packet row for
external surface CCD diagnostics. `bm_plan083_cpu_scene_corpus` now runs three
isolated built-in deformable IPC `World::step` witnesses plus one mixed reduced
`World::step` scene that exercises inter-body deformable,
deformable-vs-static-rigid, and deformable-vs-moving-rigid surface CCD limiter
paths in a single step. `scripts/write_plan083_cpu_scene_packet.py` serializes
the row as `plan083_external_surface_ccd` and requires positive aggregate and
mixed-scene candidate, CCD check, hit, and limited-step counters for all three
external families. The latest median packet records aggregate 66 inter-body
hits, 136 static-rigid hits, and 152 moving-rigid hits; the mixed scene alone
records 33/68/76 hits with `failed_steps=0`.

This is a reduced CPU diagnostic packet only. It does not prove paper-scale
external contact, production runtime scene filtering, analytic curved CCD, GPU
`World::step`, full runtime affine/FEM coupling, or any speedup gate. The
later lying-flat inter-body/static/moving-rigid witnesses, Candy
static/moving-rigid witnesses, and ABD/FEM external sidecar witnesses are the
only broader rows with nonzero external candidate/check/hit counters so far;
other figure/demo rows still have zero external candidate/check/hit counts in
their mixed fixtures.

Current validation passed:

- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py -q`
- `pixi run cmake --build build/default/cpp/Release --target bm_plan083_cpu_scene_corpus`
- `pixi run bm-plan083-cpu-external-surface-ccd-packet`

Latest CPU scene external surface-contact packet counters checkpoint
(2026-06-13): work continued locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all remaining
PLAN-083 follow-up work consolidated there; do not push, PR-comment, resolve
review threads, trigger CI, open or close PRs, delete branches, or claim
unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends `bm_plan083_cpu_scene_corpus` and
`scripts/write_plan083_cpu_scene_packet.py` so the reduced deformable CPU scene
rows serialize and validate the public external surface CCD counters exposed at
the diagnostics boundary: inter-body deformable, deformable-vs-static-rigid,
and deformable-vs-moving-rigid candidate, CCD check/hit/limiter, and snapshot
counts. The regenerated `lying_flat`, `candy`, and `abd_fem_coupling` packet
rows now carry those fields alongside the self-surface counters. At that
checkpoint the inter-body/static-rigid/moving-rigid candidate, check, and hit
counters remained zero, while static-rigid and moving-rigid snapshot builds
recorded 1 on the median rows; the later lying-flat packet adds
inter-body/static-rigid/moving-rigid witnesses, but this earlier checkpoint was
packet schema and
observability evidence, not paper-scale behavior.

This is CPU observability and reduced packet evidence only. It does not prove
nonzero paper-scale external contact, GPU `World::step`, production
scene-level GPU contact candidate construction, production runtime scene
filtering, analytic curved CCD, full runtime affine/FEM coupling, or any
speedup gate.

Current validation passed:

- `pixi run lint`
- `pixi run build`
- `pixi run cmake --build build/default/cpp/Release --target bm_plan083_cpu_scene_corpus`
- `pixi run python scripts/write_plan083_cpu_scene_packet.py --scene lying_flat`
- `pixi run python scripts/write_plan083_cpu_scene_packet.py --scene candy`
- `pixi run python scripts/write_plan083_cpu_scene_packet.py --scene abd_fem_coupling`
- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py tests/test_plan083_cpu_scene_corpus.py tests/test_plan083_completion_audit.py -q`
- `pixi run python scripts/check_plan083_cpu_scene_corpus.py`
- `pixi run python scripts/check_plan083_completion_audit.py`
- `git diff --check`

Previous external World-step surface-contact diagnostics checkpoint (2026-06-13):
work continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR
#2978. Keep all remaining PLAN-083 follow-up work consolidated there; do not
push, PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

This checkpoint extends public `DeformableSolverDiagnostics`
(`World::getLastDeformableSolverDiagnostics` and dartpy
`last_deformable_solver_diagnostics`) with existing default deformable
`World::step` inter-body deformable, deformable-vs-static-rigid, and
deformable-vs-moving-rigid surface CCD activity counters. Focused built-in
`World::step` C++ tests now cover all three external families at the public
diagnostics boundary. This is CPU observability only; it does not change the
reduced CPU scene packet rows, prove GPU `World::step`, production scene-level
GPU contact candidate construction, production runtime scene filtering,
analytic curved CCD, full runtime affine/FEM coupling, or any speedup gate.

Current validation passed:

- `pixi run cmake --build build/default/cpp/Release --target test_deformable_body`
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_deformable_body$'`

Previous built-in World-step self-surface diagnostics checkpoint (2026-06-13):
work continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR
#2978. Keep all remaining PLAN-083 follow-up work consolidated there; do not
push, PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

This checkpoint exposes existing default deformable `World::step` self-surface
contact candidate and CCD counters through `DeformableSolverDiagnostics`
(`World::getLastDeformableSolverDiagnostics` and dartpy
`last_deformable_solver_diagnostics`). The reduced CPU scene benchmark rows now
record those counters for `lying_flat`, `candy`, and the deformable side of
`abd_fem_coupling`, and the packet writer validates the fields without
claiming paper-scale reproduction. Fresh packet evidence records:

- `lying_flat`: 37 self-surface candidate builds, 1,038 point-triangle CCD
  checks, 1,852 edge-edge CCD checks, 0 CCD hits, and 0 limited steps on the
  median row.
- `candy`: 3 self-surface candidate builds, 64 point-triangle CCD checks, 207
  edge-edge CCD checks, 0 CCD hits, and 0 limited steps on the median row.
- `abd_fem_coupling`: at that checkpoint, the deformable `World::step` side
  recorded the same 37 / 1,038 / 1,852 self-surface diagnostic counts while the
  existing affine/FEM mixed-candidate and coupled micro-solve evidence remained
  bounded.

The later ABD/FEM external sidecar checkpoint above supersedes the current
`abd_fem_coupling` counts after the row inherited the lying-flat witness
sidecar.

This is CPU observability and reduced packet evidence only. It does not prove
GPU `World::step`, production scene-level GPU contact candidate construction,
production runtime scene filtering, analytic curved CCD, full runtime
affine/FEM coupling, or any speedup gate. External inter-body, static-rigid,
and moving-rigid surface-contact diagnostics are covered by the later public
diagnostics checkpoint above.

Current validation passed:

- `pixi run python -m pytest tests/test_plan083_cpu_scene_packet.py -q`
- `pixi run build`
- `pixi run cmake --build build/default/cpp/Release --target test_deformable_body`
- `ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_deformable_body$'`
- `pixi run python scripts/write_plan083_cpu_scene_packet.py --scene lying_flat`
- `pixi run python scripts/write_plan083_cpu_scene_packet.py --scene candy`
- `pixi run python scripts/write_plan083_cpu_scene_packet.py --scene abd_fem_coupling`

Latest combined scene runtime CCD line-search checkpoint (2026-06-13): work
continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978.
Keep all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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

Latest combined scene runtime sweep-filter checkpoint (2026-06-13): work
continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978.
Keep all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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

Previous combined scene runtime candidate-filter checkpoint (2026-06-13): work
continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978.
Keep all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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

Latest combined scene runtime barrier-Hessian checkpoint (2026-06-13): work
continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978.
Keep all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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

Latest scene-owned equality-reduced diagonal solve checkpoint (2026-06-13):
work continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR
#2978. Keep all remaining PLAN-083 follow-up work consolidated there; do not
push, PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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

Latest scene-owned sparse graph edge-dedup checkpoint (2026-06-13): work
continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978.
Keep all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

This checkpoint extends the private Newton assembly/solve packet with a reduced
scene-owned sparse graph unique-edge assembly row. The benchmark builds the same
DART `World` deformable surface used by the existing scene-owned sparse graph
packet, appends one reversed duplicate surface triangle for deterministic
deduplication evidence, marks canonical surface-edge keys on GPU, and emits one
6x6 sparse block per unique canonical scene edge in deterministic order. This is
reduced packet evidence only; it does not prove full production sparse Hessian
graph construction/assembly, unbounded production direct/global sparse
factorization, production nonlinear equality convergence policy/solving, GPU
`World::step` assembly/solve integration, or a top-level speedup claim.

Fresh packet evidence records a scene sparse graph unique-edge row with
`scene_node_count=2560`, `scene_triangle_count=769`, `edge_slot_count=2307`,
`unique_edge_count=2304`, `duplicate_edge_slot_count=3`, `block_count=2304`,
`block_entry_count=82944`, `max_result_abs_error=4.440892098500626e-16`,
`max_diagonal=3.8525`, `max_gradient_abs=7.6674999999999995`,
`max_block_abs=0.0014150000000000002`, and
`speedup=0.0010437835446294721x` (`meets_speedup_gate=false`). The top-level
assembly/solve packet records `max_result_abs_error=3.637978807091713e-11`,
`residual_norm=3.70262388185975e-13`, and
`speedup=0.0010437835446294721x` (`meets_speedup_gate=false`), so the durable
GPU packet row remains `in-progress`.

Current validation passed:

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

Latest scene-derived bounded direct sparse factor solve checkpoint (2026-06-13):
work continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR
#2978. Keep all remaining PLAN-083 follow-up work consolidated there; do not
push, PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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

Latest bounded direct sparse factor solve checkpoint (2026-06-13): work continued
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

Latest scene-owned nonlinear equality convergence checkpoint (2026-06-13): work continued
locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978.
Keep all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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

Latest validation passed:

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

Previous scene-owned sparse graph assembly checkpoint (2026-06-12): work continued
locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978.
Keep all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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

Latest validation passed:

- focused assembly/solve packet pytest
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`
- PLAN-083 GPU parity/completion-audit validators
- focused PLAN-083 packet pytest trio
- `git diff --check`
- `pixi run lint`

Previous scene-owned runtime CCD/line-search checkpoint (2026-06-12): work
continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978.
Keep all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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
`speedup=0.3492306596158999x` (`meets_speedup_gate=false`). The durable CCD
row remains `in-progress` until analytic curved CCD, full scene-level
line-search feasibility, and runtime speedup have evidence.

Latest validation passed:

- focused CCD packet pytest
- `pixi run -e cuda build-cuda Release`
- focused CUDA CCD CTest
- `pixi run -e cuda bm-plan083-gpu-ccd-line-search-packet`
- PLAN-083 GPU parity/completion-audit validators
- focused PLAN-083 packet pytest trio
- `git diff --check`
- `pixi run lint`

Previous scene-owned assembly/solve checkpoint (2026-06-12): work continued
locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all
remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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

Latest validation passed:

- focused assembly/solve packet pytest
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`
- PLAN-083 GPU parity/completion-audit validators
- focused PLAN-083 packet pytest trio
- `git diff --check`
- `pixi run lint`

Latest sparse CG solve checkpoint (2026-06-12): work continued locally on
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

Latest validation passed:

- focused assembly/solve packet pytest
- PLAN-083 GPU parity/completion-audit validators
- focused PLAN-083 packet pytest trio
- `git diff --check`
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`
- `pixi run lint`

Latest sparse Jacobi solve checkpoint (2026-06-12): work continued locally on
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

Latest validation passed:

- focused assembly/solve packet pytest
- PLAN-083 GPU parity/completion-audit validators
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`
- `pixi run lint`

Latest sparse residual assembly checkpoint (2026-06-12): work continued
locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978. Keep all
remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

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
`speedup=0.24114770986042178x` (`meets_speedup_gate=false`), so the durable
GPU packet row remains `in-progress`.

Latest validation passed:

- focused assembly/solve packet pytest
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`

Latest equality-reduced assembly/solve checkpoint (2026-06-12): work
continued locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978.
Keep all remaining PLAN-083 follow-up work consolidated there; do not push,
PR-comment, resolve review threads, trigger CI, open or close PRs, delete
branches, or claim unrelated PLAN-091 packets without explicit maintainer
approval.

This checkpoint extends the private Newton assembly/solve packet with a reduced
sparse equality-reduction row. The CUDA path assembles full-space diagonal
rows, projects them through a sparse reduced-coordinate basis, solves the
regularized reduced diagonal Newton step, and expands the step back to full
coordinates. Fresh packet evidence records 65,536 rows, 8,192 bodies, 49,152
full dofs, 49,152 reduction entries, and 24,576 reduced dofs with
`max_result_abs_error=3.552713678800501e-15`,
`residual_norm=2.927032132859879e-15`, and
`speedup=0.4504406756295252x` for the equality-reduced row. The top-level
assembly/solve packet records `max_result_abs_error=3.552713678800501e-15`,
`residual_norm=3.026790431755041e-15`, and
`speedup=0.2937796651929902x` (`meets_speedup_gate=false`), so the durable GPU
packet row remains `in-progress`.

Latest validation passed:

- focused assembly/solve packet pytest
- `pixi run -e cuda build-cuda Release`
- focused `test_newton_assembly_solve_cuda` CTest
- `pixi run -e cuda bm-newton-assembly-solve-packet`

Latest edge-edge Hessian PSD checkpoint (2026-06-12): the latest
`origin/main` has been merged into
`simx/plan083-gpu-contact-candidate-packet`, bringing in the DART 7
architecture hardening/work-packet harness from PR #2986. Continue to keep
PLAN-083 follow-up work consolidated on PR #2978; do not push, PR-comment,
resolve review threads, trigger CI, open or close PRs, delete branches, or
claim unrelated PLAN-091 packets without explicit maintainer approval.

This checkpoint extends the private barrier/friction packet with the missing
edge-edge Hessian PSD-projection benchmark row. Fresh packet evidence records
59,578 active edge-edge PSD-projection barriers with
`max_result_abs_error=4.334310688136611e-13`,
`psd_projection_ns=121906741.0`, and `speedup=2.045170785242439x`. The
conditioned edge-edge primitive Hessian row remains covered with
`max_result_abs_error=3.623767952376511e-13`; the scene-owned edge-edge runtime
row still consumes 1,536 runtime edge-edge candidates from one DART `World`
deformable surface and records 1,280 active barriers with
`max_result_abs_error=2.220446049250313e-15`. The top-level barrier/friction
packet records `speedup=0.17984862275960234x`
(`meets_speedup_gate=false`), so the durable GPU packet row remains
`in-progress`.

Resumed barrier-Hessian packet checkpoint (2026-06-12): the maintainer gave a
fresh `continue` instruction after the stop-only handoff. Work may resume
locally on `simx/plan083-gpu-contact-candidate-packet`, PR #2978, while
keeping all remaining PLAN-083 work consolidated on this single branch/PR. Do
not push, PR-comment, resolve review threads, trigger CI, open or close PRs,
delete branches, or clean branches without explicit maintainer approval.

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

Stop-only handoff (2026-06-12): the maintainer explicitly stopped further
implementation and verification here. Do not continue work, run validation,
push, PR-comment, resolve review threads, trigger CI, open or close PRs,
delete branches, or clean branches unless the maintainer gives a new resume
instruction. Resume context remains
`simx/plan083-gpu-contact-candidate-packet`, PR #2978; the working tree
intentionally contains uncommitted handoff/state changes plus the latest
scene-owned point-triangle barrier-Hessian packet work.

Scene-owned runtime barrier-Hessian checkpoint (2026-06-12): work continued on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work consolidated on that single branch/PR. Do not push,
PR-comment, resolve review threads, trigger CI, open PRs, close PRs, delete
branches, or clean up branches without explicit maintainer approval.

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
`speedup=0.2016402329495093x` (`meets_speedup_gate=false`). This is reduced
scene-owned point-triangle, point-edge, and point-point barrier-Hessian evidence
only: broader sparse Hessian assembly, broader sparse barrier/contact assembly,
full GPU
`World::step`, and speedup-gate completion remain future work.

Latest validation passed:

- focused barrier/friction packet pytest
- `git diff --check`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`

Scene-owned runtime sweep checkpoint (2026-06-12): work continued on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work consolidated on that single branch/PR. Do not push,
PR-comment, resolve review threads, trigger CI, open PRs, close PRs, delete
branches, or clean up branches without explicit maintainer approval.

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
`speedup=0.024497841563440446x` (`meets_speedup_gate=false`). This is reduced
scene-owned sweep evidence only: full runtime scene filtering, GPU
`World::step` contact candidate construction, and speedup-gate completion
remain future work.

Latest validation passed:

- focused contact-candidate packet pytest
- `git diff --check`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda test-cuda` (8/8)
- `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit` (161/161)

Current continuation handoff (2026-06-12): work has resumed after the prior
maintainer stop-only handoff. Continue only from
`simx/plan083-gpu-contact-candidate-packet`, PR #2978, keep all PLAN-083 work
consolidated on that branch/PR, and inspect `HANDOFF.md`, `git status -sb`,
and the local commit log before doing anything else. Do not push,
PR-comment, resolve review threads, trigger CI, open PRs, or delete branches
without explicit maintainer approval.

Resumed continuation handoff (2026-06-12): work resumed after the prior
stop-only handoff. Continue locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978, keep all PLAN-083
follow-up consolidated there, and read `HANDOFF.md` first. Do not push,
PR-comment, resolve review threads, trigger CI, open another PLAN-083 PR, or
delete branches without explicit maintainer approval.

Validated sweep broad-phase checkpoint (2026-06-12): work continued on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978, and added private CUDA
swept-AABB sweep-and-prune packets for point-triangle and edge-edge candidate
construction. The packet now device-builds swept primitive AABBs, device-sorts
them by the CPU sweep key, and deterministically counts/scatters compact
overlap candidates in CPU sweep order.

Fresh packet evidence records `candidate_pair_count=393216`,
`max_result_abs_error=5.551115123125783e-17`, and
`speedup=0.03107190750809115x` (`meets_speedup_gate=false`). The
point-triangle sweep broad-phase row covers 65,536 pair capacity over 256
points and 256 triangles, emits 256 compact candidates, and records
`speedup=0.03107190750809115x`; the edge-edge sweep broad-phase row covers
65,536 pair capacity over 256 edges, emits 128 compact candidates, and records
`speedup=0.03286696141763437x`. This is private reduced broad-phase evidence
only: scene-owned GPU candidate buffers, runtime scene filtering, and the
speedup gate remain future work.

Scene-owned runtime candidate-buffer checkpoint (2026-06-12): work continued on
the same consolidated branch/PR and added reduced scene-owned runtime
candidate-buffer packet rows. The benchmark now builds one DART `World`
deformable surface, extracts node start/end states and surface triangles from
the `DeformableBody` handle, reuses the CPU motion-aware sweep builder for
candidate keys, and evaluates the compact point-triangle and edge-edge
candidate buffers on CPU and CUDA.

Fresh packet evidence records one scene body, 512 point-triangle candidates
over 2,560 points and 768 triangles, and 1,536 edge-edge candidates over 2,304
surface edges. CPU/GPU endpoint-distance parity remains exact within
`5.551115123125783e-17`, but the top-level packet speedup is
`0.017259032907863573x` (`meets_speedup_gate=false`). This is reduced
scene-owned runtime-buffer evidence only: runtime scene filtering, GPU
`World::step`, and speedup-gate completion remain future work.

Latest validation passed:

- focused contact-candidate packet pytest
- focused contact-candidate/GPU-parity/audit pytest trio (18 passed)
- `git diff --check`
- `pixi run lint`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda test-cuda` (8/8)
- `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`
- PLAN-083 GPU parity/completion-audit checkers
- `pixi run build`
- `pixi run test-unit` (161/161)

Validated runtime sweep-buffer checkpoint (2026-06-12): work continued on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978, and added private CUDA
evaluators for compact point-triangle and edge-edge candidate buffers produced
by the CPU motion-aware sweep builder. The packet rows recompute endpoint
squared-distance metadata on the GPU for compact runtime sweep candidate keys:
512 point-triangle candidates over 2,560 points and 768 triangles, plus 1,536
edge-edge candidates over 2,304 surface edges.

Fresh local packet evidence records `candidate_pair_count=262144`,
`max_result_abs_error=5.551115123125783e-17`, and
`speedup=0.11821712677569206x` (`meets_speedup_gate=false`). The runtime
point-triangle sweep-buffer row records `speedup=0.11821712677569206x`; the
runtime edge-edge sweep-buffer row records `speedup=0.28057906386249953x`.
This is runtime-buffer parity evidence only: GPU sweep-and-prune broad-phase
sorting, scene-owned GPU candidate buffers, runtime scene filtering, and the
speedup gate remain future work.

Validation passed: focused contact-candidate packet pytest, PLAN-083 GPU
parity/completion-audit checkers, focused contact-candidate/GPU-parity/audit
pytest trio, `pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run build-simulation-tests`, `pixi run test-simulation` (65/65),
`pixi run check-api-boundaries`, `pixi run -e cuda build-cuda Release`,
`pixi run -e cuda test-cuda` (8/8), and
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`.

Latest compacted-distance candidate-buffer checkpoint (2026-06-12): work
resumed after the stop-only handoff. Keep all remaining PLAN-083 work
consolidated on `simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). This checkpoint
builds on `f7ee131c457 Add swept contact candidate packet parity` and adds
compacted per-candidate distance metadata to the private GPU
contact-candidate buffers. Static point-triangle and edge-edge candidate-mask
packets now compact squared-distance metadata next to accepted primitive ids;
swept point-triangle and edge-edge candidate-list packets now compact endpoint
squared-distance metadata next to accepted primitive ids.

The regenerated contact-candidate packet records `compacted_distance_count`
for all four candidate-construction rows: 192 static point-triangle candidates,
96 static edge-edge candidates, 256 swept point-triangle candidates, and 128
swept edge-edge candidates. The top-level packet still has
`candidate_pair_count=262144`, `max_result_abs_error=0`, and
`speedup=0.32965959230892933x` (`meets_speedup_gate=false`), so the row stays
`in-progress`. This is runtime-buffer prerequisite evidence, not
sweep-and-prune broad-phase sorting, runtime scene filtering, or speedup-gate
completion.

Validation for this checkpoint passed focused contact-candidate packet pytest,
`pixi run -e cuda build-cuda Release`, focused
`test_contact_candidate_filter_cuda` CTest,
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`, the PLAN-083 GPU
parity/completion-audit checker pair, the focused
contact-candidate/GPU-parity/completion-audit pytest trio, `pixi run lint`,
`pixi run build`, and `pixi run test-unit` (161/161).

Before any future push, merge latest `origin/main` into this published branch,
rerun the required gates, and push only with explicit maintainer approval.

Resumed swept-AABB validation checkpoint (2026-06-12): after the stop-only
handoff, this session resumed under the active PLAN-083 goal and validated the
swept-AABB contact-candidate packet slice locally. Keep all remaining PLAN-083
work consolidated on `simx/plan083-gpu-contact-candidate-packet` / PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Do not push,
comment on PRs, resolve review threads, trigger CI, or open another PLAN-083 PR
without explicit maintainer approval.

Validation for the resumed checkpoint passed focused packet pytest (7 tests),
`pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run -e cuda build-cuda Release`, focused
`test_contact_candidate_filter_cuda` CTest, `pixi run -e cuda test-cuda`
(8/8), and `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`. Before
any future push, merge latest `origin/main` into this published branch, rerun
required gates, and get explicit maintainer approval.

Latest swept-AABB candidate-list checkpoint (2026-06-12): implementation
continued on `simx/plan083-gpu-contact-candidate-packet`, PR #2978, and added
private motion-aware point-triangle and edge-edge swept-AABB candidate-list
packets. The checkpoint adds
`buildSweptPointTriangleContactCandidateMaskCuda()` and
`buildSweptEdgeEdgeContactCandidateMaskCuda()`, rejects incident/duplicate
pairs, records minimum endpoint squared-distance metadata, and compacts
accepted pair ids on the device before readback. This moves the
contact-candidate packet beyond static all-pairs distance masks while keeping
sweep-and-prune sorting and runtime scene candidate buffers as future work.

The regenerated packet records exact parity for the existing stencil and
all-pairs rows plus 65,536 swept-AABB point-triangle pairs and 65,536
swept-AABB edge-edge pairs. The swept point-triangle row records
`accepted_count=256`, `compacted_count=256`,
`compacted_triangle_count=256`, `max_result_abs_error=0`, and
`speedup=0.5141495691008232x`. The swept edge-edge row records
`accepted_count=128`, `compacted_edge_a_count=128`,
`compacted_edge_b_count=128`, `max_result_abs_error=0`, and
`speedup=0.2686885092103126x`. The top-level contact-candidate packet records
`candidate_pair_count=262144` and `speedup=0.2686885092103126x`
(`meets_speedup_gate=false`). This remains reduced private packet evidence
only; sweep-and-prune broad-phase sorting, runtime candidate buffers, and the
speedup gate remain future work.

Latest edge-edge candidate-mask checkpoint (2026-06-12): implementation
resumed on `simx/plan083-gpu-contact-candidate-packet`, PR #2978, and
validated the private edge-edge all-pairs contact-candidate mask packet. The
checkpoint adds `buildEdgeEdgeContactCandidateMaskCuda()`, rejects
self/duplicate/incident edge pairs, compacts accepted edge-slot pairs on the
device, extends focused CUDA unit coverage, and updates the contact-candidate
benchmark/packet writer to require
`candidate_construction.edge_edge_all_pairs_mask`.

The regenerated packet records exact parity for 65,536 point-triangle stencils,
65,536 edge-edge stencils, a 65,536-pair point-triangle all-pairs mask, and a
65,536-pair edge-edge all-pairs mask. The edge-edge mask row records
`edge_count=256`, `accepted_count=96`, `compacted_edge_a_count=96`,
`compacted_edge_b_count=96`, `max_result_abs_error=0`, and
`speedup=0.6679555059736084x`; the top-level contact-candidate packet records
`speedup=0.37765557710841663x` (`meets_speedup_gate=false`). This remains
reduced private packet evidence only; sweep broad-phase construction, runtime
candidate-list construction, and the speedup gate remain future work.

Validation for this checkpoint passed focused packet pytest (7 tests), the
PLAN-083 GPU parity/completion-audit checker pair, the focused
contact-candidate/GPU-parity/completion-audit pytest trio (16 passed),
`pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run build-simulation-tests`, `pixi run test-simulation` (65/65),
`pixi run -e cuda build-cuda Release`, focused
`test_contact_candidate_filter_cuda` CTest, `pixi run -e cuda test-cuda`
(8/8), and `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`.

Latest continuation checkpoint (2026-06-12): implementation resumed on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work on this consolidated branch/PR. Do not push,
PR-comment, resolve review threads, trigger CI, or open another PLAN-083 PR
without explicit maintainer approval.

The latest checkpoint replaces the committed serial device compaction kernel
with deterministic block-prefix CUDA compaction for the private point-triangle
all-pairs candidate-mask packet. It uses block-local accepted counts, a small
block-offset prefix pass, and a stable block-local scatter so compacted
point/triangle lists stay in deterministic pair order. This is reduced private
packet evidence only: it does not claim sweep broad-phase construction,
runtime candidate-list construction, or a top-level contact-candidate
speedup-gate completion.

Validation for this checkpoint passed `pixi run lint`, `pixi run build`,
`pixi run test-unit` (161/161), `pixi run build-simulation-tests`,
`pixi run test-simulation` (65/65), `pixi run check-api-boundaries`, the
PLAN-083 GPU parity/completion-audit checker pair, the focused
contact-candidate/GPU-parity/completion-audit pytest trio (15 passed),
`pixi run -e cuda build-cuda Release`, `pixi run -e cuda test-cuda` (8/8), and
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`. The regenerated
packet records `accepted_count=192`, `compacted_count=192`,
`compacted_triangle_count=192`, `max_result_abs_error=0`, candidate-mask
`speedup=0.9911014836006811x`, and top-level contact-candidate packet
`speedup=0.4065799369175524x` (`meets_speedup_gate=false`). Before any future
push, merge latest `origin/main` into this published branch, rerun required
gates, and push only with explicit maintainer approval.

Previous readback-compaction checkpoint (2026-06-11): implementation resumed on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work on this consolidated branch/PR. Do not push,
PR-comment, resolve review threads, trigger CI, or open another PLAN-083 PR
without explicit maintainer approval.

That checkpoint builds on
`bed8ab7569b Add point-triangle candidate mask packet parity` by adding
compacted point-triangle candidate-pair metadata after GPU mask readback. The
contact-candidate CUDA result now records accepted point/triangle index lists,
the benchmark emits `gpu_compacted_count`, and the packet writer/test require
the compacted count to match the accepted count. This is reduced private packet
evidence only: it does not claim sweep broad-phase construction, runtime
candidate-list construction, or a contact-candidate speedup-gate completion.

Validation for this checkpoint passed focused contact-candidate packet pytest
(5 tests), `pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run -e cuda build-cuda Release`, `pixi run -e cuda test-cuda` (8/8), and
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`. The regenerated
packet records `accepted_count=192`, `compacted_count=192`,
`max_result_abs_error=0`, candidate-mask `speedup=1.5943055078570016x`, and
top-level contact-candidate packet `speedup=0.35669978298935295x`
(`meets_speedup_gate=false`). Before any future push, merge latest
`origin/main` into this published branch, rerun required gates, and push only
with explicit maintainer approval.

Earlier continuation state (2026-06-11): implementation resumed after the
stop-only handoff. Resume only from
`simx/plan083-gpu-contact-candidate-packet`, the single consolidated #2978 PR
(`Advance unified Newton-barrier runtime and parity evidence`). Do not push,
PR-comment, resolve review threads, or trigger CI without explicit maintainer
approval.

Current branch shape for a fresh session:

- Last observed pushed head:
  `6746b63973d Record point-point barrier Hessian packet evidence`.
- Local committed checkpoints ahead of origin:
  `48dcfb515cf Add point-edge barrier Hessian packet parity`,
  `1dfb21b24da Add point-triangle barrier Hessian packet parity`,
  `1022b609f8c Add point-triangle Hessian PSD packet parity`, and
  `c1e6e73b2cf Add point-point and point-edge Hessian PSD packet parity`,
  followed by
  `e41941db91c Add off-diagonal sparse block assembly packet parity`.
- The preceding committed checkpoint adds a reduced pair-slot off-diagonal
  sparse-block assembly packet under the private Newton assembly/solve CUDA
  packet. The current checkpoint adds a private CUDA
  point-triangle all-pairs candidate mask to the contact-candidate packet. It is
  not sweep broad-phase construction, not compacted runtime scene candidate
  list construction, and not a speedup-gate claim.

Focused evidence gathered for the current assembly checkpoint passed packet
pytest, CUDA build, focused `test_newton_assembly_solve_cuda` CTest, and
`pixi run -e cuda bm-newton-assembly-solve-packet`. The generated packet
measured top-level `speedup=0.26051227540244215x` with
`meets_speedup_gate=false`, so the row stays `in-progress`. `HANDOFF.md` is
the authoritative fresh-session handoff.

Focused evidence gathered for the earlier candidate-mask checkpoint passed the
contact-candidate packet pytest, focused packet/audit pytest trio, lint, build,
unit tests, CUDA build, `pixi run -e cuda test-cuda`, and
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`. The generated
packet measured exact parity for 65,536 point-triangle all-pairs candidates
with `accepted_count=192`, `compacted_count=192`,
`speedup=1.5943055078570016x`, and `meets_speedup_gate=true`; the top-level
contact-candidate packet measured `speedup=0.35669978298935295x`, so the row
stays `in-progress`.

Fresh-session branch discipline: all remaining PLAN-083 follow-up work is
consolidated on `simx/plan083-gpu-contact-candidate-packet` and PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Do not open or
revive per-packet/per-phase PRs. Former stacked PRs #2979-#2983 were folded into
#2978; earlier runtime/corpus follow-ups #2970, #2971, #2974, and #2976 are
merged into `main`. A fresh Claude/Codex session should resume from #2978,
check hosted CI/review state, and keep any remaining PLAN-083 commits on the
same branch.

Current continuation (2026-06-11): the formerly dirty point-point and
point-edge primitive barrier-Hessian PSD-projection WIP on #2978 has been
completed as packet evidence on the same consolidated branch. The branch is
still unpublished beyond
`origin/simx/plan083-gpu-contact-candidate-packet`; treat `HANDOFF.md` as the
authoritative fresh-session entry point before any future edit or push.

Current #2978 checkpoint (2026-06-11): the active branch now carries private
CUDA point-triangle primitive barrier-gradient parity plus point-triangle,
edge-edge, point-edge, and point-point tangent-stencil parity in the
barrier/friction packet, and the current package adds private CUDA
point-triangle, point-point, and point-edge primitive barrier-Hessian rows plus
point-triangle, point-point, and point-edge primitive barrier-Hessian
PSD-projection parity. The latest packet measured
`max_result_abs_error=7.844391802791506e-12` across those local rows;
point-triangle barrier-Hessian speedup was `1.3792236932217794x`,
point-triangle Hessian PSD-projection speedup was `2.4324816046452007x`,
point-point barrier-Hessian speedup was `1.3681853755001632x`, point-point
Hessian PSD-projection speedup was `2.6247434778815464x`, point-edge
barrier-Hessian speedup was `1.8065117930616938x`, and point-edge Hessian
PSD-projection speedup was `4.388364832151794x`. The top-level packet still
measured `speedup=0.3604503271533569x` with `meets_speedup_gate=false`, so the
row remains `in-progress` because broader sparse Hessian assembly, runtime
contact rows, and the top-level speedup gate remain future evidence.

Historical stop-only handoff (2026-06-11): the maintainer previously asked for
handoff only with no further verification. That left uncommitted PSD-coupling
WIP on this same branch. The WIP has since been resumed as the
point-triangle, point-point, and point-edge Hessian PSD-projection packet
slices; keep `HANDOFF.md` synchronized for fresh sessions.

Latest validation checkpoint (2026-06-11): the point-edge barrier-Hessian slice
is committed locally as
`48dcfb515cf Add point-edge barrier Hessian packet parity` and is not yet
pushed. The follow-on point-triangle barrier-Hessian slice is committed locally
as `1dfb21b24da Add point-triangle barrier Hessian packet parity` and is not
yet pushed. It was validated locally on the same branch with private CUDA
implementation, focused unit parity, benchmark packet rows, packet writer/test
coverage, and durable sidecar updates. The current point-triangle Hessian
PSD-projection slice adds unit parity, benchmark rows, packet writer/test
coverage, and sidecar updates on top of those commits. The follow-on
point-point/point-edge Hessian PSD-projection slice adds the same evidence
shape for the remaining primitive Hessian packet families. `HANDOFF.md` is the
authoritative fresh-session entry point.

Validation for the current PSD-projection checkpoint passed `pixi run lint`,
`pixi run build`, `pixi run test-unit` (161/161),
`pixi run -e cuda build-cuda Release`, focused
`test_barrier_friction_kernel_cuda` CTest,
`pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
(4 passed), `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`, the
PLAN-083 GPU parity/completion-audit checker pair, and the focused packet/audit
pytest trio (13 passed).

Validation for the point-point/point-edge Hessian PSD-projection checkpoint
passed `pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run -e cuda build-cuda Release`, focused
`test_barrier_friction_kernel_cuda` CTest,
`pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
(4 passed), `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`, the
PLAN-083 GPU parity/completion-audit checker pair, and the focused packet/audit
pytest trio (13 passed).

Validated hand-off checkpoint (2026-06-11): the formerly unverified
point-edge/point-point tangent-stencil WIP has now passed lint, CUDA build,
focused CUDA CTest, the barrier/friction benchmark packet, and packet/audit
checks. Durable packet docs record it as parity evidence while keeping the row
`in-progress`.

Historical critical stop hand-off (2026-06-11): the maintainer then instructed
the agent to stop implementation and focus only on hand-off, with no further
verification. Continuation later resumed on the same consolidated branch; use
`HANDOFF.md` plus `RESUME.md` as the fresh-session entry point.

Durable sidecar sync (2026-06-11): `gpu-parity-packet.json`,
`completion-audit.md`, `paper-deck-manifest.md`, and `docs/plans/dashboard.md`
now record the point-triangle, point-point, and point-edge barrier-Hessian
packets plus point-triangle, point-point, and point-edge Hessian PSD-projection
packets as in-progress evidence.

Fresh-session continuation: first inspect `git status -sb`, the local diff,
#2978, the branch head, hosted CI, and new review comments. Then continue with
broader sparse Hessian assembly, broader sparse barrier/contact assembly, or
speedup-gate work on the same PR. Do not open another PLAN-083 PR.

- [x] Phase 1: promote shared world-primitive math into an internal
      Newton-barrier owner.
  - [x] Add `detail/newton_barrier` owners for primitive distances, clamped-log
        barriers, and tangent stencils.
  - [x] Preserve `detail/deformable_contact` compatibility forwarding so active
        deformable IPC branches keep compiling during reconciliation.
  - [x] Update rigid IPC primitive calls to consume the Newton-barrier owner
        directly.
  - [x] Add cross-variant primitive tests for old/new alias parity, rigid
        consumer parity, and the internal namespace boundary.
  - [x] Record smoke output from the existing distance, barrier, tangent, and
        rigid IPC benchmark binaries without renaming them.
- [x] Phase 2: add the ABD first-slice internal prototype: affine body state,
      affine surface adapter, orthogonality energy, affine primitive/friction
      chain rules, rigid-equivalence tests, and first benchmark packet shape.
  - [x] Add internal `AffineBodyState` and `AffineSurfaceAdapter` owners under
        `detail/` with 12-DOF affine state vectorization and per-vertex
        Jacobians.
  - [x] Map shared Newton-barrier point-triangle, point-edge, edge-edge, and
        point-point primitive rows through affine Jacobians.
  - [x] Add stiff affine orthogonality energy with analytic gradient/Hessian
        coverage.
  - [x] Add rigid-equivalence oracle by projecting affine gradients and
        Hessians onto the rigid tangent space, including the rotation-vector
        curvature term needed to match PLAN-082 rigid IPC.
  - [x] Add the first ABD benchmark packet shape with a matched rigid IPC
        point-triangle oracle row and orthogonality-energy row.
  - [x] Add a reduced affine point-triangle solved-state micro-solve diagnostic
        and benchmark row without claiming an ABD runtime solver.
  - [x] Add reduced friction equivalence rows through affine tangent stencils
        for point-point, point-edge, edge-edge, and point-triangle primitives.
  - [x] Promote the first packet into the PLAN-083 manifest as an in-progress
        `abd-alg-affine-body` comparison row. Add
        `pixi run bm-abd-comparison-packet` to generate/validate
        `.benchmark_results/abd_comparison_packet.json`.
- [x] Phase 3: generalize second-use PSD projection and projected-Newton
      contracts when ABD or another solver-family slice needs the shared
      contract; use the PLAN-083 variant consolidation map to keep IPC-family
      responsibilities in the right owner while promoting only proven
      second-use contracts.
  - [x] Promote the first second-use PSD projection contract into
        `detail/newton_barrier` and route rigid IPC plus ABD Hessian projection
        through it with shared tests.
  - [x] Wrap the deformable batched PSD backend behind a
        `detail/newton_barrier` owner and route deformable projected-Newton
        Hessian batches through it while preserving the existing CPU/CUDA
        backend seam.
  - [x] Promote the first shared line-search option/stat contract into
        `detail/newton_barrier` and route rigid IPC plus deformable CCD stats
        accumulation through it with shared tests.
  - [x] Promote the shared line-search positive-step predicate into
        `detail/newton_barrier` and route rigid IPC plus deformable CCD result
        methods through it while keeping hit/limited result ownership
        variant-local.
  - [x] Promote the shared conservative native-CCD option adapter into
        `detail/newton_barrier` and route rigid IPC plus deformable CCD
        line-search queries through it while keeping their CCD implementations
        and limiting-primitive payloads variant-local.
  - [x] Promote the shared line-search step-scale policy into
        `detail/newton_barrier` and route rigid IPC Newton step scaling plus
        deformable/world CCD limiters through it while keeping their result
        payloads and zero-step diagnostics variant-local.
  - [x] Promote the shared native-CCD primitive outcome accounting policy into
        `detail/newton_barrier` and route rigid IPC plus deformable CCD hit,
        miss, indeterminate, and step-bound clamping through it while keeping
        limiting-payload ownership and indeterminate-result policy
        variant-local.
  - [x] Count zero-step indeterminate native-CCD outcomes through the shared
        line-search accounting helper so rigid IPC and deformable CCD
        diagnostics report blocking indeterminate queries consistently while
        keeping their limiting payloads variant-local.
  - [x] Promote the shared Armijo sufficient-decrease and backtracking scalar
        policy into `detail/newton_barrier` and route rigid IPC plus deformable
        projected-Newton line-search checks through it while keeping
        variant-specific acceptance/fallback semantics local.
  - [x] Promote the shared full-step line-search feasibility predicate into
        `detail/newton_barrier` and route rigid IPC kinematic feasibility plus
        deformable CCD result methods through it while keeping limited/hit
        payloads variant-local.
  - [x] Promote shared projected-Newton residual/tolerance helpers into
        `detail/newton_barrier` and route rigid IPC plus deformable
        convergence diagnostics through them while keeping solver status enums
        variant-local.
  - [x] Promote shared lagged-friction work diagnostics into
        `detail/newton_barrier` and route deformable, rigid IPC, and ABD
        friction diagnostics through the same smoothed Coulomb work contract.
  - [x] Promote the first shared Google Benchmark packet row parser into
        `scripts/benchmark_packet_utils.py` and route the ABD comparison packet
        checker plus the Phase 5 GPU packet checker through it while keeping
        packet-specific metadata rules in their owners.
  - [x] Route the Phase 5 CUDA packet writer through the shared Google
        Benchmark canonical row identity helper, removing its duplicate row
        parser while keeping packet-specific max-error extraction local.
  - [x] Promote the first benchmark packet timing schema for per-step counts
        and solver-subphase timing fields into `scripts/benchmark_packet_utils.py`
        while keeping packet-specific required subphases and go/no-go gates in
        their owners.
  - [x] Close the remaining Phase 3 scouting by routing deformable
        projected-Newton backtracking through the shared Newton-barrier default
        scale and rigid IPC's option defaults through the same shared scalar
        constants. The implementation-roadmap Phase 2 shared solver contracts
        are complete; remaining solver result/status payloads and
        packet-specific gates stay variant-local until another phase proves
        identical behavior.
- [x] Implementation-roadmap Phase 3: add the unified articulation constraint
      family behind internal DART-owned Newton-barrier names.
  - [x] Add point-connection and fixed-point residual/Jacobian helpers.
  - [x] Add hinge-axis residuals and cone-twist bend/twist decomposition.
  - [x] Add sliding and relative-sliding residual/Jacobian helpers.
  - [x] Add distance residual/Jacobian helpers and bounded-distance barrier
        diagnostics.
  - [x] Add sliding-range and rotation-range barrier diagnostics with PSD
        Hessian approximations.
- [x] Implementation-roadmap Phase 4: add internal restitution, BDF-2, and
      Rayleigh damping diagnostic contracts.
  - [x] Add restitution target, BDF-2 inertial target, restart, velocity-update,
        and serializable history helpers under `detail/newton_barrier`.
  - [x] Add falling-box energy diagnostics across timestep, Young's modulus,
        barrier stiffness, activation distance, and gravity sweeps.
  - [x] Add a Fig. 17-style barrier-force curve diagnostic for activation
        cutoff, kappa scaling, and near-contact slope growth.
  - [x] Add semi-implicit Rayleigh damping terms for contact/barrier and
        articulation potentials using PSD-projected Hessians.
  - [x] Add hinge damping sweep evidence through the scalar articulation
        damping path.
- [x] Implementation-roadmap Phase 5: add internal mixed-domain coupling
      contracts.
  - [x] Add shared surface adapters for rigid, deformable, affine, particle,
        rod, shell, and codimensional domains.
  - [x] Add deterministic mixed-domain candidate generation for point-point,
        point-edge, edge-edge, and point-triangle primitive families.
  - [x] Add conservative point-pair CCD reduction plus deterministic restart
        keys for mixed-domain candidate sets.
  - [x] Add barrier, friction, and oracle-owner diagnostics that preserve
        variant-specific correctness owners.
- [x] Implementation-roadmap Phase 6: port the CPU scene corpus and py-demos
      categories after the relevant solver slices exist and have
      correctness/performance evidence.
  - [x] Add launchable planned py-demo placeholders for the PLAN-083 mixed,
        constraint, robot, and ABD CPU corpus categories.
  - [x] Add a checked CPU scene corpus sidecar that records each row's smoke
        command, long-horizon visual capture command, benchmark/profile packet
        path, invariant, and current limitation.
  - [x] Add local validation coverage for the corpus manifest and py-demo
        placeholder panels/catalog categories.
  - [x] Keep real paper-scene reproduction, runtime mixed stepping, and
        performance claims gated behind the row-specific limitations.
- [x] Implementation-roadmap Phase 7: add private GPU parity and speed packets.
  - [x] Add a checked private GPU parity packet sidecar for contact
        stencils/candidates, CCD/line search, local barrier/friction kernels,
        PSD projection, assembly/linear solve, and scene-level speedup rows.
  - [x] Encode same-scene parity, tolerance, transfer/setup/readback timing,
        kernel/solve timing, speedup, and no-public-API policies for every row.
  - [x] Keep rows without landed kernels explicit as planned/in-progress
        limitations instead of GPU speed claims.
  - [x] Add the measured private PSD projection packet generator and benchmark
        row; keep it scoped to local-kernel evidence, not scene-level GPU
        parity.
  - [x] Add private point-triangle and edge-edge contact-stencil filter packets
        plus brute-force all-pairs point-triangle and edge-edge candidate-mask
        packets plus motion-aware swept-AABB point-triangle and edge-edge
        candidate-list packets with exact CPU/GPU parity and device-side
        compacted candidate ids plus distance metadata, and compact runtime
        sweep-buffer endpoint-distance packets that consume CPU sweep candidate
        keys plus reduced scene-owned runtime candidate-buffer packets from one
        DART `World` deformable surface plus scene-owned runtime sweep
        broad-phase packets from that same surface; keep the row in-progress
        because full runtime scene filtering, GPU `World::step` contact
        candidate construction, and speedup remain unproven.
  - [x] Add private endpoint-linear point-triangle/edge-edge plus sampled
        rigid-curved point-triangle/edge-edge CCD/line-search packets plus
        reduced scene-owned runtime point-triangle/edge-edge CCD rows with
        CPU/GPU step-bound parity; keep the row in-progress because analytic
        curved CCD, full scene-level line-search feasibility, and runtime
        speedup remain unproven.
  - [x] Add private scalar barrier/friction local-kernel, point-triangle
        primitive barrier-gradient, point-triangle/edge-edge/point-edge/
        point-point tangent-stencil, and point-triangle/point-point/point-edge/
        edge-edge primitive barrier-Hessian packets, plus point-triangle/
        point-point/point-edge Hessian PSD-projection parity, plus reduced
        scene-owned point-triangle/point-edge/point-point/edge-edge
        barrier-Hessian runtime rows plus a reduced combined all-family scene
        runtime barrier-Hessian row, with exact CPU/GPU local-output parity;
        keep the row in-progress because broader sparse Hessian assembly,
        full GPU `World::step`, and runtime speedup remain unproven.
  - [x] Add private reduced diagonal assembly/solve, reduced scene-owned
        diagonal assembly/solve, pair-slot off-diagonal sparse-block assembly,
        reduced scene-owned sparse off-diagonal surface-edge assembly, sparse
        block residual matvec, reduced scene-owned sparse residual matvec,
        fixed-iteration sparse Jacobi solve, reduced scene-owned sparse Jacobi
        solve, capped sparse CG solve, reduced scene-owned sparse CG solve,
        reduced scene-owned nonlinear distance-equality assembly/solve, and
        sparse equality-reduced diagonal solve plus reduced scene-owned
        equality-reduced diagonal solve packets with exact CPU/GPU local-output
        parity; keep the row in-progress because full runtime sparse Hessian
        graph construction and assembly, direct/global sparse factorization,
        production nonlinear equality convergence policy/solving, GPU
        `World::step` assembly/solve integration, and speedup remain unproven.
  - [x] Add a private reduced scene state-batch parity packet with exact
        CPU/GPU rollout parity and speedup; keep the row in-progress because
        GPU `World::step`, contact candidate construction, CCD,
        barrier/friction assembly, sparse equality reduction, and global Newton
        solving remain unproven.
- [ ] Implementation-roadmap Phase 8: complete the PLAN-083 audit and retire
      temporary task state.
  - [x] Add a checked completion audit sidecar that records the current
        manifest, CPU corpus, and GPU packet status counts.
  - [x] Record that PLAN-083 is not complete while reduced in-progress
        CPU/GPU/scene rows remain.
  - [ ] Retire the temporary dev-task folder only after maintainer direction;
        material in-progress work remains.
- [x] Runtime wiring follow-up: consume landed Newton-barrier contracts from
      rigid IPC runtime paths without opening per-slice PRs.
  - [x] Route point-connection/fixed-joint and hinge-axis constraints through
        the rigid IPC `World::step` projected-Newton loop.
  - [x] Add opt-in BDF-2 integration to the rigid IPC contact stage while
        keeping semi-implicit stepping as the default.
  - [x] Route deformable surfaces as fixed mixed-domain obstacles through the
        rigid IPC contact solve.
  - [x] Add a reduced hanging-bridge py-demo smoke scene that steps through
        `World::step` and supports headless capture.
- [ ] CPU corpus evidence follow-up: add reduced scene packets without claiming
      paper-scale reproduction.
  - [x] Add the first reduced hanging-bridge CPU scene packet benchmark and
        writer for the runtime smoke path.
  - [x] Add a reduced lying-flat deformable-cloth/static-obstacle CPU scene
        packet and runtime status panel while keeping paper-scale mixed
        coupling future work.
  - [x] Add a reduced pulley hinged-wheel CPU scene packet and runtime status
        panel while keeping force comparison and rope/rod coupling future work.
  - [x] Add a reduced umbrella hinged-rib CPU scene packet and runtime status
        panel while keeping cloth, sliding, and paper-scale rod coupling future
        work.
  - [x] Add a reduced nunchaku hinge CPU scene packet and runtime status panel
        for the public revolute-joint path.
  - [x] Add a reduced terrain vehicle CPU scene packet and runtime status panel
        for chassis/passive wheel hinges over flat terrain.
  - [x] Add a reduced windmill hinge/contact CPU scene packet and runtime
        status panel while keeping Bullet/reference comparison future work.
  - [x] Add a reduced Candy deformable-cloth/static-shell CPU scene packet,
        static-rigid surface CCD witness, and runtime status panel while
        keeping affine packing, twisted shell, and self-contact parity future
        work.
  - [x] Add a reduced precession wheel CPU scene packet and runtime status
        panel while keeping angular-velocity sweeps future work.
  - [x] Add a reduced ragdoll CPU scene packet and runtime status panel while
        keeping cone-twist joints and 60-ragdoll scale future work.
  - [x] Add a reduced nunchaku scaling CPU packet for sizes 20, 40, 60, 80,
        and 100 while keeping cone-twist and coupled contact scaling future
        work.
  - [x] Add a reduced timing-breakdown CPU packet that aggregates runnable
        reduced corpus rows while keeping paper subphase timing future work.
  - [x] Add a reduced Table 2 setup/statistics CPU packet for runnable reduced
        rows while keeping missing mixed/ABD rows future work.
  - [x] Add a sparse equality change-of-variable internal contract with rank,
        residual, sparse free-coordinate basis, and KKT solve-equivalence tests
        and route the rigid IPC equality step through it while keeping
        paper-scale pulley/sliding scenes future work.
  - [x] Add a reduced ABD complex-geometry CPU packet using generated affine
        point-triangle pair runtime steps while keeping the 1.2M-triangle scene
        asset and contact-force visualization future work.
  - [x] Add a reduced ABD/FEM CPU packet that combines affine pair runtime
        steps, a deformable IPC smoke step, mixed candidate diagnostics, and a
        coupled affine/deformable contact micro-solve while keeping full
        runtime affine/FEM coupling future work.
  - [ ] Add paper-scale CPU packets only after the corresponding scene assets,
        mixed-domain stepping, and comparison baselines exist.

## Goal

Implement PLAN-083 incrementally until DART owns the IPC solver family through
the shared Newton-barrier primitive layer, affine stiff-body track, CPU/GPU
evidence, paper/deck parity rows, and py-demos examples behind DART-owned DART 7
`World` capabilities without exposing upstream solver names, registries, ECS
storage, or backend resources as public API.

## Non-Goals For Current Internal Slices

- No public API changes.
- No dartpy binding changes.
- No paper-scale runtime scene reproduction or fixture asset migration beyond
  the reduced lying-flat, hanging-bridge, pulley, umbrella, terrain vehicle,
  ragdoll, nunchaku, windmill, Candy, and precession smoke scenes, the reduced
  timing-breakdown and Table 2 packets, launchable planned py-demo placeholders,
  and checked corpus manifest.
- No analytic rigid curved-trajectory CCD move or runtime line-search
  integration.
- No sparse Newton loop merge.
- No rigid IPC default behavior change.
- No public or `World`-integrated ABD runtime stage.
- No full scene-level or paper-scale GPU speedup claim.

## Key Decisions

- Phase 1 is an internal ownership and contract slice, not a behavior change.
- Batch work into the smallest reviewer-useful PR unit: default to one branch
  and PR per implementation-roadmap phase, but consolidate adjacent small
  internal phases into one PR when that keeps review clearer. Commits within the
  branch stay atomic and self-describing. Split only for public-API boundaries,
  unrelated CI/build infrastructure, or reviewability.
- For the remaining PLAN-083 follow-up work, PR #2978 is the consolidated
  review unit; keep additional packet/runtime slices on that branch instead of
  opening per-packet PRs.
- Current PR #2978 head includes the CUDA CCD review fix, sampled
  rigid-curved CCD/line-search packet rows, plus the private
  barrier/friction point-triangle gradient, all four primitive-family
  tangent-stencil packet rows, point-triangle/point-point/point-edge/edge-edge
  primitive barrier-Hessian, point-triangle/point-point/point-edge Hessian
  PSD-projection, and reduced scene-owned point-triangle/point-edge/
  point-point/edge-edge barrier-Hessian runtime packet evidence plus a reduced
  combined all-family scene runtime barrier-Hessian row. Continue
  monitoring CI/review on the same branch and keep further PLAN-083 slices
  consolidated there.
- The old `deformable_contact` include paths remain as forwarding
  compatibility headers to avoid unnecessary PLAN-081 merge conflicts.
- Rigid IPC should include the new Newton-barrier owner directly because it is
  the second active consumer and the reduced-coordinate correctness oracle.
- Existing benchmark binary names stay stable for performance-history
  continuity.
- ABD starts only after the primitive contract has cross-variant tests.
- The first ABD chain-rule slice uses the affine map `x = a + A X`, so its
  Hessian has no transform-curvature term. Rigid Hessian equivalence projects
  onto the rigid tangent space and adds the rotation-vector exponential
  curvature term before comparing against rigid IPC.
- The first second-use friction contract is the shared tangent-displacement
  friction kernel under `detail/newton_barrier`, consumed by rigid IPC and the
  affine primitive-family friction rows.
- Public docs and APIs keep method/capability names DART-owned; internal tests
  and manifests may cite IPC, rigid IPC, ABD, and paper row provenance.
- Treat IPC as the representative solver-family name for PLAN-083 when the
  unified Newton-barrier method is the most advanced shared IPC variant.
  `Newton-barrier` names the implementation contracts and paper lineage;
  variant names such as rigid IPC, deformable IPC, and ABD remain internal
  provenance until shared behavior is proven.
- The first `abd-alg-affine-body` micro-packet is primitive/oracle evidence and
  now includes a reduced point-triangle solved-state diagnostic row with
  residual/convergence counters plus reduced runtime-step packets for the
  cards, wrecking-ball, and chain-net ABD rows. Broader ABD packets still need
  scene-level runtime residuals, scene assets, and comparison baselines before
  any paper-scale ABD row moves.

## Immediate Next Steps

1. Use merged PRs #2960, #2961, #2970, #2971, #2974, and #2976 as the baseline,
   and keep remaining work in consolidated PR #2978 instead of reopening the
   old phase-scoped stack. A fresh session should resume the branch/PR context
   first, check hosted #2978 CI/review state for actionable failures, then
   continue runtime scene filtering, speedup-gate work, runtime sparse
   Hessian graph construction/assembly beyond the reduced dedup row, broader
   sparse barrier/contact assembly,
   direct/global sparse factorization, or analytic/runtime CCD line-search
   evidence on the same PR.
2. Keep private GPU scene-level parity limited to reduced scene state-batch
   rollout parity; do not mark the row measured until GPU `World::step`,
   contact candidate construction, CCD, barrier/friction assembly, sparse
   Hessian graph construction/assembly, direct/global sparse factorization,
   and global Newton solving have concrete evidence.
3. Use the reduced ABD runtime-step, ABD/FEM coupled micro-solve, and GPU
   contact-stencil packets only as internal runtime evidence; broader ABD CPU
   packets still require scene-level runtime residuals, scene assets, full
   runtime affine/FEM coupling, and comparison baselines.
4. Get maintainer direction before retiring
   `docs/dev_tasks/unified_newton_barrier_multibody/`: the Phase 8 audit found
   that PLAN-083 still has in-progress CPU/GPU/scene rows and cannot honestly
   be called complete yet.
5. Promote only the smallest proven shared contract, with cross-variant tests
   showing identical behavior; keep variant-specific terms in their owner plans.
6. Keep paper-scale runtime stepping and non-PSD GPU claims out of scope until
   the row-specific CPU corpus packets and Phase 7 GPU parity evidence exist.

## Validation Gates For Current Slices

```bash
pixi run lint
pixi run build-simulation-tests
pixi run test-simulation
pixi run check-api-boundaries
pixi run bm bm_ipc_distance_kernels -- --benchmark_min_time=0.05s
pixi run bm bm_ipc_barrier_kernel -- --benchmark_min_time=0.05s
pixi run bm bm_ipc_tangent_stencil -- --benchmark_min_time=0.05s
pixi run bm bm_rigid_ipc_solver -- --benchmark_min_time=0.05s
```

CUDA evidence is required only if the slice touches the PSD wrapper or another
GPU-backed path:

```bash
pixi run -e cuda test-cuda
pixi run -e cuda bm-plan083-gpu-contact-candidates-packet
```

Phase 8 local evidence:

- `pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_plan083_completion_audit.py`
- `pixi run lint`
- `pixi run build`

Phase 7 PSD packet evidence:

- `pixi run python -m pytest tests/test_plan083_gpu_psd_packet.py`
- `pixi run -e cuda build-cuda`
- `pixi run -e cuda bm-plan083-gpu-psd-packet`

Phase 7 contact-candidate packet evidence:

- `pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py`
- `pixi run -e cuda build-cuda`
- `pixi run -e cuda test-cuda`
- `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`

Phase 7 CCD/line-search packet evidence:

- `pixi run python -m pytest tests/test_plan083_gpu_ccd_line_search_packet.py`
- `pixi run -e cuda build-cuda`
- `pixi run -e cuda test-cuda`
- `pixi run -e cuda bm-plan083-gpu-ccd-line-search-packet`

Latest PR #2978 CUDA CCD review-fix evidence:

- `pixi run lint`
- `pixi run -e cuda build-cuda Release`
- `ctest --test-dir build/cuda/cpp/Release --output-on-failure -R '^test_ccd_line_search_cuda$'`
- `pixi run -e cuda bm-plan083-gpu-ccd-line-search-packet` (point-triangle and
  edge-edge parity for 131,072 total pairs; minimum primitive-family speedup
  2.057x)
- `pixi run python scripts/check_plan083_gpu_parity_packet.py && pixi run python scripts/check_plan083_completion_audit.py`
- `pixi run python -m pytest tests/test_plan083_gpu_ccd_line_search_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`
  (13 passed)

Phase 7 barrier/friction local-kernel packet evidence:

- `pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py`
- `pixi run -e cuda build-cuda Release`
- `ctest --test-dir build/cuda/cpp/Release --output-on-failure -R '^test_barrier_friction_kernel_cuda$'`
- `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`

Phase 7 reduced assembly/solve packet evidence:

- `pixi run python -m pytest tests/test_newton_assembly_solve_packet.py`
- `pixi run -e cuda build-cuda`
- `pixi run -e cuda test-cuda`
- `pixi run -e cuda bm-newton-assembly-solve-packet`

Phase 1 local evidence:

- `pixi run lint`
- `pixi run build-simulation-tests`
- `pixi run test-simulation`
- `pixi run check-api-boundaries`
- `pixi run bm bm_ipc_distance_kernels -- --benchmark_min_time=0.05s`
- `pixi run bm bm_ipc_barrier_kernel -- --benchmark_min_time=0.05s`
- `pixi run bm bm_ipc_tangent_stencil -- --benchmark_min_time=0.05s`
- `pixi run bm bm_rigid_ipc_solver -- --benchmark_min_time=0.05s`

Phase 2 local evidence so far:

- `pixi run build-simulation-tests`
- `pixi run test-simulation` (64/64)
- `pixi run bm bm_affine_body_dynamics -- --benchmark_min_time=0.05s`
- `pixi run bm-abd-comparison-packet`

Phase 3 line-search option/stat slice local evidence:

- `pixi run lint`
- `pixi run build-simulation-experimental-tests`

Phase 3 PSD backend wrapper slice local evidence:

- `pixi run -- cmake --build build/default/cpp/Release --target test_deformable_psd_backend test_world --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_deformable_psd_backend|test_world)$'`
- `pixi run test-simulation-experimental` (65/65)
- `pixi run check-api-boundaries`

Phase 3 line-search positive-step predicate slice local evidence:

- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_newton_barrier_primitives$'`
- `pixi run build-simulation-tests`
- `pixi run test-simulation` (65/65)
- `pixi run check-api-boundaries`
- `pixi run lint`

Phase 3 benchmark packet utility slice local evidence:

- `pixi run python -m pytest tests/test_benchmark_packet_utils.py`
- `pixi run lint`

Phase 3 benchmark row identity slice local evidence:

- `pixi run python -m pytest tests/test_benchmark_packet_utils.py`
- `pixi run lint`

Phase 3 line-search step-scale slice local evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives test_rigid_ipc_barrier test_world --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_newton_barrier_primitives|test_rigid_ipc_barrier|test_world)$'`
- `pixi run build`
- `pixi run test-unit`

Phase 3 native-CCD primitive outcome accounting slice local evidence:

- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives test_continuous_collision_step test_rigid_ipc_barrier --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_newton_barrier_primitives|test_continuous_collision_step|test_rigid_ipc_barrier)$'`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`
- `pixi run test-all`
- `pixi run -e cuda test-all` (docs passed with the existing
  `dartpy._world_render_bridge` autodoc warnings)

Phase 3 native-CCD zero-step diagnostic accounting slice local evidence:

- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_newton_barrier_primitives$'`

Phase 3 sufficient-decrease policy slice local evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives test_rigid_ipc_barrier test_world --parallel <safe-jobs>`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_newton_barrier_primitives|test_rigid_ipc_barrier|test_world)$'`

Phase 3 closeout local evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives test_rigid_ipc_barrier test_world --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_newton_barrier_primitives|test_rigid_ipc_barrier|test_world)$'`

Implementation-roadmap Phase 2 branch-local closeout evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives test_affine_body_dynamics test_rigid_ipc_barrier test_world test_deformable_body --parallel <safe-jobs>`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -j <safe-jobs> -R '^(test_newton_barrier_primitives|test_affine_body_dynamics|test_rigid_ipc_barrier|test_world|test_deformable_body)$'`
- `pixi run python -m pytest tests/test_benchmark_packet_utils.py`

Implementation-roadmap Phase 3 branch-local evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives --parallel <safe-jobs>`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -j <safe-jobs> -R '^test_newton_barrier_primitives$'`

Implementation-roadmap Phase 4 branch-local evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives --parallel <safe-jobs>`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -j <safe-jobs> -R '^test_newton_barrier_primitives$'`
- `pixi run build`
- `pixi run test-unit`

Implementation-roadmap Phase 5 branch-local evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives --parallel <safe-jobs>`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -j <safe-jobs> -R '^test_newton_barrier_primitives$'`
- `pixi run build`
- `pixi run test-unit`

Implementation-roadmap Phase 6 branch-local evidence:

- `pixi run lint`
- `pixi run build`
- `pixi run python scripts/check_plan083_cpu_scene_corpus.py`
- `PYTHONPATH=build/default/cpp/Release-docking/python:build/default/cpp/Release/python:python pixi run python -m pytest tests/test_plan083_cpu_scene_corpus.py python/tests/unit/test_py_demo_panels.py::test_plan083_cpu_corpus_placeholders_expose_status_panels python/tests/unit/test_py_demo_panels.py::test_registered_world_scenes_receive_shared_replay_controls python/tests/integration/test_demos_cycle.py::test_world_scenes_use_solver_focused_categories`
- `PYTHONPATH=build/default/cpp/Release-docking/python:build/default/cpp/Release/python:python pixi run py-demos -- --list`
- `pixi run test-py`

Implementation-roadmap Phase 7 branch-local evidence:

- `pixi run python scripts/check_plan083_gpu_parity_packet.py`
- `pixi run python -m pytest tests/test_plan083_gpu_parity_packet.py`
- `pixi run lint`
- `pixi run build`
- `pixi run -e cuda test-cuda`

Runtime-wiring branch-local evidence:

- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`
- `pixi run test-py`
- `ctest --test-dir build/default/cpp/Release -R 'test_world|test_rigid_ipc_barrier' --output-on-failure`
- `LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe pixi run py-demos -- --scene plan083_hanging_bridge --headless --frames 4 --width 320 --height 240`
- `LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe pixi run py-demo-capture -- --scene plan083_hanging_bridge --frames 4 --width 320 --height 240 --output-dir /tmp/dart_plan083_hanging_bridge_capture`
- `pixi run bm-plan083-cpu-hanging-bridge-packet`
- `pixi run bm-plan083-cpu-lying-flat-packet`
- `pixi run bm-plan083-cpu-pulley-packet`
- `pixi run bm-plan083-cpu-umbrella-packet`
- `pixi run bm-plan083-cpu-candy-packet`
- `pixi run bm-plan083-cpu-nunchaku-packet`
- `pixi run bm-plan083-cpu-nunchaku-scaling-packet`
- `pixi run bm-plan083-cpu-ragdoll-packet`
- `pixi run python scripts/check_plan083_cpu_scene_corpus.py`
- `pixi run python scripts/check_plan083_completion_audit.py`

## Owner Docs

- PLAN-083 owner:
  [`../../plans/083-unified-newton-barrier-multibody.md`](../../plans/083-unified-newton-barrier-multibody.md)
- IPC-family variant consolidation:
  [`../../plans/083-unified-newton-barrier-multibody/ipc-variant-consolidation.md`](../../plans/083-unified-newton-barrier-multibody/ipc-variant-consolidation.md)
- Implementation roadmap:
  [`../../plans/083-unified-newton-barrier-multibody/implementation-roadmap.md`](../../plans/083-unified-newton-barrier-multibody/implementation-roadmap.md)
- Primitive promotion slice:
  [`../../plans/083-unified-newton-barrier-multibody/primitive-promotion-slice.md`](../../plans/083-unified-newton-barrier-multibody/primitive-promotion-slice.md)
- ABD first-slice design:
  [`../../plans/083-unified-newton-barrier-multibody/abd-first-slice-design.md`](../../plans/083-unified-newton-barrier-multibody/abd-first-slice-design.md)
- Shared primitive audit:
  [`../../plans/083-unified-newton-barrier-multibody/shared-primitive-audit.md`](../../plans/083-unified-newton-barrier-multibody/shared-primitive-audit.md)
- Deformable IPC variant:
  [`../../plans/081-deformable-implicit-barrier-solver.md`](../../plans/081-deformable-implicit-barrier-solver.md)
- Rigid IPC variant:
  [`../../plans/082-rigid-implicit-barrier-contact.md`](../../plans/082-rigid-implicit-barrier-contact.md)
