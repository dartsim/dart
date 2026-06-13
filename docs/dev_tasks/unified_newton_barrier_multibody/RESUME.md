# Resume: Unified Newton-Barrier Multibody

## Current Reality (2026-06-09)

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

Fresh-session prompt: use the "Fresh Codex Goal Prompt" section in
`docs/dev_tasks/unified_newton_barrier_multibody/HANDOFF.md` before selecting
the next bounded packet-style slice.

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
instruction.

Resume context remains `simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). The working
tree intentionally contains uncommitted handoff/state changes plus the latest
scene-owned point-triangle barrier-Hessian packet work. Treat those local
changes as intentional in-progress state, not cleanup candidates. At handoff,
the branch was ahead of `origin/simx/plan083-gpu-contact-candidate-packet` by
16 commits, with modified dev-task docs, PLAN-083 status docs, packet
writer/test files, and the GPU barrier/friction benchmark. Verification was
not continued after the stop-only request.

Scene-owned runtime barrier-Hessian checkpoint (2026-06-12): continue from
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
only: broader sparse Hessian assembly, additional runtime rows, full GPU
`World::step`, and speedup-gate completion remain future work.

Latest local gates:

- focused barrier/friction packet pytest
- `git diff --check`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`

Scene-owned runtime sweep checkpoint (2026-06-12): continue from
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

Latest local gates:

- focused contact-candidate packet pytest
- `git diff --check`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda test-cuda` (8/8)
- `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit` (161/161)

Current continuation handoff (2026-06-12): work has resumed after the prior
maintainer stop-only handoff. Resume only from
`simx/plan083-gpu-contact-candidate-packet`, PR #2978, keep all PLAN-083 work
consolidated on that branch/PR, and inspect `HANDOFF.md`, `git status -sb`,
and the local commit log before doing anything else. Do not push, PR-comment,
resolve review threads, trigger CI, open PRs, or delete branches without
explicit maintainer approval.

Resumed continuation handoff (2026-06-12): work resumed after the prior
stop-only handoff. Continue locally on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978, keep all PLAN-083
follow-up consolidated there, and read `HANDOFF.md` first. Do not push,
PR-comment, resolve review threads, trigger CI, open another PLAN-083 PR, or
delete branches without explicit maintainer approval.

Scene-owned runtime candidate-buffer checkpoint (2026-06-12): continue from
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). This checkpoint
adds reduced scene-owned runtime candidate-buffer rows to the private GPU
contact-candidate packet. The benchmark builds one DART `World` deformable
surface, extracts node start/end states and surface triangles from the
`DeformableBody` handle, reuses the CPU motion-aware sweep builder for compact
candidate keys, and evaluates point-triangle and edge-edge endpoint-distance
buffers on CPU and CUDA.

Fresh packet evidence records one scene body, 512 point-triangle candidates
over 2,560 points and 768 triangles, and 1,536 edge-edge candidates over 2,304
surface edges. CPU/GPU endpoint-distance parity remains exact within
`5.551115123125783e-17`, but the top-level packet speedup is
`0.017259032907863573x` (`meets_speedup_gate=false`). This is reduced
scene-owned runtime-buffer evidence only: runtime scene filtering, GPU
`World::step`, and speedup-gate completion remain future work.

Latest local gates:

- focused contact-candidate packet pytest
- focused contact-candidate/GPU-parity/audit pytest trio (18 passed)
- `git diff --check`
- `pixi run lint`
- `pixi run -e cuda build-cuda Release`
- `pixi run -e cuda test-cuda` (8/8)
- `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`
- `pixi run build`
- `pixi run test-unit` (161/161)

Validated sweep broad-phase checkpoint (2026-06-12): continue from
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). This checkpoint
adds private CUDA swept-AABB sweep-and-prune packets for point-triangle and
edge-edge candidate construction. The packet device-builds swept primitive
AABBs, device-sorts them by the CPU sweep key, and deterministically
counts/scatters compact overlap candidates in CPU sweep order.

Fresh local packet evidence records `candidate_pair_count=393216`,
`max_result_abs_error=5.551115123125783e-17`, and
`speedup=0.03107190750809115x` (`meets_speedup_gate=false`). The
point-triangle sweep broad-phase row covers 65,536 pair capacity over 256
points and 256 triangles, emits 256 compact candidates, and records
`speedup=0.03107190750809115x`; the edge-edge sweep broad-phase row covers
65,536 pair capacity over 256 edges, emits 128 compact candidates, and records
`speedup=0.03286696141763437x`. This is private reduced broad-phase evidence
only: scene-owned GPU candidate buffers, runtime scene filtering, and the
speedup gate remain future work.

Validation passed: `pixi run lint`; `git diff --check`; focused
contact-candidate/GPU-parity/audit pytest trio (18 passed);
`pixi run -e cuda build-cuda Release`; focused
`test_contact_candidate_filter_cuda` CTest; `pixi run -e cuda test-cuda`
(8/8, with the LCP CUDA test taking 231.80s);
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`; PLAN-083 GPU
parity/completion-audit checkers; `pixi run build`; and
`pixi run test-unit` (161/161). Before any future push, merge latest
`origin/main` into this published branch, rerun the required gates, and get
explicit maintainer approval. Keep all remaining PLAN-083 work consolidated on
PR #2978.

Validated runtime sweep-buffer checkpoint (2026-06-12): continue from
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). This validated
checkpoint adds private CUDA evaluators for compact point-triangle and
edge-edge candidate buffers produced by the CPU motion-aware sweep builder.
The packet rows recompute endpoint squared-distance metadata on the GPU for
512 compact point-triangle runtime sweep candidates over 2,560 points and 768
triangles, plus 1,536 compact edge-edge runtime sweep candidates over 2,304
surface edges.

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

Before any future push, merge latest `origin/main` into this published branch,
rerun the required gates, and get explicit maintainer approval. Keep all
remaining PLAN-083 work consolidated on PR #2978.

Latest compacted-distance candidate-buffer checkpoint (2026-06-12): work
resumed after the stop-only handoff. Continue from
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
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
swept-AABB contact-candidate packet slice locally. Resume only from
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
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
accepted pair ids on the device before readback.

The regenerated contact-candidate packet records exact parity for 65,536
swept-AABB point-triangle pairs and 65,536 swept-AABB edge-edge pairs in
addition to the previous stencil and all-pairs rows. The swept point-triangle
row records `accepted_count=256`, `compacted_count=256`,
`compacted_triangle_count=256`, `max_result_abs_error=0`, and
`speedup=0.5141495691008232x`; the swept edge-edge row records
`accepted_count=128`, `compacted_edge_a_count=128`,
`compacted_edge_b_count=128`, `max_result_abs_error=0`, and
`speedup=0.2686885092103126x`. The top-level packet records
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
(8/8), and `pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`. Read
`HANDOFF.md` and inspect `git status -sb` before continuing.

Latest continuation checkpoint (2026-06-12): implementation resumed on
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). Keep all
remaining PLAN-083 work on this consolidated branch/PR; do not push,
PR-comment, resolve review threads, trigger CI, or open another PLAN-083 PR
without explicit maintainer approval.

The latest checkpoint replaces the committed serial device compaction kernel
with deterministic block-prefix CUDA compaction for the private point-triangle
all-pairs candidate-mask packet. It uses block-local accepted counts, a small
block-offset prefix pass, and a stable block-local scatter so compacted
point/triangle lists stay in deterministic pair order. It keeps packet coverage
for matching compacted point and triangle counts. This is still reduced
private packet evidence only: sweep broad-phase construction, runtime
candidate-list construction, and the top-level speedup gate remain future work.

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
`speedup=0.4065799369175524x` (`meets_speedup_gate=false`).

Previous readback-compaction checkpoint (2026-06-11): resume only from
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`), after reading
`HANDOFF.md`, `git status -sb`, and the PR state. Keep all remaining PLAN-083
work on this consolidated branch/PR. Do not push, PR-comment, resolve review
threads, trigger CI, or open another PLAN-083 PR without explicit maintainer
approval.

That checkpoint builds on
`bed8ab7569b Add point-triangle candidate mask packet parity` by adding
compacted point-triangle candidate-pair metadata after GPU mask readback. The
contact-candidate CUDA result records accepted point/triangle index lists, the
benchmark emits `gpu_compacted_count`, and the packet writer/test reject
compaction-count mismatches. This remains reduced private packet evidence: it
does not claim sweep broad-phase construction, runtime scene candidate-list
construction, or a speedup-gate completion claim.

Validation for this checkpoint passed focused contact-candidate packet pytest
(5 tests), `pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run -e cuda build-cuda Release`, `pixi run -e cuda test-cuda` (8/8), and
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`. The regenerated
packet records `pair_count=65536`, `accepted_count=192`,
`compacted_count=192`, `max_result_abs_error=0`, candidate-mask
`speedup=1.5943055078570016x`, and top-level contact-candidate packet
`speedup=0.35669978298935295x` (`meets_speedup_gate=false`). Before any future
push, merge latest `origin/main` into this published branch, rerun the required
gates, and push only with explicit maintainer approval.

Earlier continuation handoff (2026-06-11): implementation resumed after the
stop-only handoff. Resume only from
`simx/plan083-gpu-contact-candidate-packet`, the single consolidated #2978 PR
(`Advance unified Newton-barrier runtime and parity evidence`). Do not push,
PR-comment, resolve review threads, or trigger CI without explicit maintainer
approval.

The branch has five local committed checkpoints ahead of the last observed
origin head `6746b63973d Record point-point barrier Hessian packet evidence`:
`48dcfb515cf Add point-edge barrier Hessian packet parity`,
`1dfb21b24da Add point-triangle barrier Hessian packet parity`,
`1022b609f8c Add point-triangle Hessian PSD packet parity`, and
`c1e6e73b2cf Add point-point and point-edge Hessian PSD packet parity`, followed
by `e41941db91c Add off-diagonal sparse block assembly packet parity`.

The preceding checkpoint adds reduced off-diagonal sparse-block assembly
evidence on the same branch. Expected implementation and packet surfaces are
`dart/simulation/compute/cuda/newton_assembly_solve_cuda.cuh`,
`dart/simulation/compute/cuda/newton_assembly_solve_cuda.cpp`,
`dart/simulation/compute/cuda/newton_assembly_solve_cuda.cu`,
`tests/unit/simulation/cuda/test_newton_assembly_solve_cuda.cpp`,
`tests/benchmark/simulation/bm_newton_assembly_solve_cuda.cpp`,
`scripts/write_newton_assembly_solve_packet.py`,
`tests/test_newton_assembly_solve_packet.py`, and generated local packet
`.benchmark_results/plan083/gpu/assembly_linear_solve_parity.json`. This
checkpoint adds private pair-slot off-diagonal sparse-block assembly parity,
not full sparse global factorization or runtime scene assembly. It should
remain an `in-progress` evidence claim after commit.

Focused evidence for this checkpoint passed packet pytest, CUDA build, focused
`test_newton_assembly_solve_cuda` CTest, and
`pixi run -e cuda bm-newton-assembly-solve-packet`. The generated benchmark
reported `rows=65536`, `dofs=49152`,
`max_result_abs_error=1.7763568394002505e-15`,
`residual_norm=3.0267237170020088e-15`,
`speedup=0.26051227540244215x`, and `meets_speedup_gate=false`; the
off-diagonal row observed `pairs=4096`, `active_blocks=4096`,
`block_entries=147456`, `max_result_abs_error=0`, and
`speedup=0.4342273750294942x`.

The earlier candidate-mask checkpoint adds a private CUDA point-triangle all-pairs candidate
mask to the contact-candidate packet. It extends the preassembled stencil
filter path with
`buildPointTriangleContactCandidateMaskCuda()`, a CUDA mask kernel, focused
CUDA unit parity, benchmark rows, packet-writer/test coverage, and tracked
plan/dev-task sidecar updates. This is still reduced primitive-fixture
evidence: it is not sweep broad-phase construction, compacted runtime scene
candidate list construction, or a speedup-gate claim. Focused evidence passed
`pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py -q`,
`pixi run python -m pytest tests/test_plan083_gpu_contact_candidate_packet.py tests/test_plan083_gpu_parity_packet.py tests/test_plan083_completion_audit.py -q`,
`pixi run lint`, `pixi run build`, `pixi run test-unit`,
`pixi run -e cuda build-cuda Release`, `pixi run -e cuda test-cuda`,
`pixi run -e cuda bm-plan083-gpu-contact-candidates-packet`,
`pixi run python scripts/check_plan083_gpu_parity_packet.py`, and the focused
contact-candidate/GPU-parity/completion-audit pytest trio. The generated packet
recorded
`pair_count=65536`, `point_count=256`, `triangle_count=256`,
`accepted_count=192`, `compacted_count=192`, `max_result_abs_error=0`,
`speedup=1.5943055078570016x`, and `meets_speedup_gate=true`; top-level
contact-candidate packet speedup was `0.35669978298935295x`.

Fresh-session next step: inspect `HANDOFF.md`, `git status -sb`, and the local
diff. Continue with sweep broad-phase/runtime compacted candidate lists,
equality reduction, global sparse factorization, runtime scene rows, or
speedup-gate work only as honest `in-progress` evidence. Before any future push
to #2978, merge latest `origin/main` into the published branch, run required
gates, and push only with explicit maintainer approval.

PR granularity: default to one branch and PR per implementation-roadmap phase,
but consolidate adjacent small internal phases into one PR when that keeps
review clearer. Keep commits atomic within the branch. Split only for
public-API boundaries, unrelated CI/build infrastructure, or reviewability.
For the current remaining PLAN-083 follow-up work, use PR #2978 as the single
consolidated review unit and keep additional packet/runtime slices on that
branch.

Fresh-session handoff (2026-06-11): resume only from
`simx/plan083-gpu-contact-candidate-packet`, PR #2978
(`Advance unified Newton-barrier runtime and parity evidence`). This branch is
the consolidated PLAN-083 follow-up branch targeting `main`; do not open or
revive per-packet/per-phase PRs. Former stack PRs #2979-#2983 were folded into
this branch, and earlier runtime/corpus follow-ups #2970, #2971, #2974, and
#2976 are already merged into `main`. Treat local branches/worktrees such as
`feature/newton-barrier-runtime-wiring` as stale historical context unless a
maintainer explicitly asks to inspect them; the active work and PR state live on
PR #2978.

Current continuation (2026-06-11): the former point-point and point-edge
primitive barrier-Hessian PSD-projection WIP on #2978 has been completed as a
checked packet slice on the same consolidated branch. It adds CUDA unit parity,
benchmark rows, packet writer/test coverage, refreshed GPU parity sidecar data,
and plan/dev-task status updates for
`point_point_barrier_hessian_psd_projection` and
`point_edge_barrier_hessian_psd_projection`. The packet measured
`point_point_barrier_hessian_psd_projection.max_result_abs_error =
9.592326932761353e-14`,
`point_point_barrier_hessian_psd_projection.speedup =
2.6247434778815464x`,
`point_edge_barrier_hessian_psd_projection.max_result_abs_error =
7.844391802791506e-12`, and
`point_edge_barrier_hessian_psd_projection.speedup =
4.388364832151794x`. The top-level barrier/friction packet still measured
`speedup = 0.3604503271533569x` with `meets_speedup_gate=false`, so the row
remains `in-progress`.

Validation for this checkpoint passed `pixi run lint`, `pixi run build`,
`pixi run test-unit` (161/161), `pixi run -e cuda build-cuda Release`,
focused `test_barrier_friction_kernel_cuda` CTest, focused barrier/friction
packet pytest (4 passed),
`pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`, the PLAN-083 GPU
parity/completion-audit checker pair, and the focused packet/audit pytest trio
(13 passed).

Current continuation checkpoint (2026-06-11): the prior stop-only handoff left
uncommitted point-triangle Hessian PSD-projection WIP on this branch. That WIP
has now been resumed into a focused packet slice with CUDA unit parity,
benchmark rows, packet writer/test coverage, and durable sidecar updates. The
branch is locally ahead of
`origin/simx/plan083-gpu-contact-candidate-packet` by
`48dcfb515cf Add point-edge barrier Hessian packet parity` and
`1dfb21b24da Add point-triangle barrier Hessian packet parity`, plus the
current PSD-projection slice. A fresh session should keep working on PR #2978
only, inspect `HANDOFF.md`, and avoid pushing until latest `origin/main` has
been merged into this published branch and required gates have passed.

Latest validation checkpoint (2026-06-11): the branch is locally ahead of
origin by `48dcfb515cf Add point-edge barrier Hessian packet parity` and
`1dfb21b24da Add point-triangle barrier Hessian packet parity`, which completed
the point-edge and point-triangle primitive barrier-Hessian packet slices. The
point-triangle slice adds private CUDA result/wrapper/kernel plumbing,
focused CPU/CUDA parity coverage for non-degenerate Hessian fixtures, benchmark
packet rows, packet writer/test coverage, and durable sidecar updates. The
latest packet evidence measured
`point_triangle_barrier_hessian.max_result_abs_error =
2.5887393237583516e-12`,
`point_triangle_barrier_hessian.speedup = 1.3808963801674754x`,
`point_triangle_barrier_hessian_psd_projection.max_result_abs_error =
2.8990143619012088e-12`,
`point_triangle_barrier_hessian_psd_projection.speedup =
2.6270517059212906x`, and top-level
`speedup = 0.22046365998499093x` with `meets_speedup_gate=false`; keep the
barrier/friction row `in-progress` because broader sparse Hessian assembly,
runtime contact rows, and the top-level/runtime speedup gate remain future
evidence.

Validation for the point-triangle barrier-Hessian checkpoint passed
`pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161),
`pixi run -e cuda build-cuda Release`, focused
`test_barrier_friction_kernel_cuda` CTest,
`pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
(4 passed), `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`, the
PLAN-083 GPU parity/completion-audit checker pair, and the focused packet/audit
pytest trio (13 passed). The checkpoint has not been pushed unless
`origin/simx/plan083-gpu-contact-candidate-packet` has moved past
`6746b63973d`. The later PSD-projection continuation has passed
`pixi run lint`, `pixi run build`, `pixi run test-unit` (161/161), focused
packet pytest, CUDA build, focused barrier/friction CUDA CTest, the
barrier/friction benchmark packet, and the PLAN-083 packet/audit checkers.

Continuation checkpoint (2026-06-11, point-edge barrier-Hessian slice): the
previously dirty point-edge barrier-Hessian prototype on #2978 has been
completed with a launcher, CPU/CUDA unit parity coverage, benchmark rows,
packet writer/test coverage, and durable sidecar updates. The refreshed packet
measured `point_edge_barrier_hessian.max_result_abs_error =
7.716494110354688e-12`,
`point_edge_barrier_hessian.speedup = 1.278816738709317x`, and top-level
`speedup = 0.36913439757406363x` with `meets_speedup_gate=false`; keep the
barrier/friction row `in-progress` because broader Hessian assembly, PSD
coupling, runtime contact rows, and the top-level/runtime speedup gate remain
future evidence.

Validation checkpoint (2026-06-11, point-edge barrier-Hessian slice): local
gates for this checkpoint passed `pixi run lint`, `pixi run build`,
`pixi run test-unit` (161/161), `pixi run -e cuda build-cuda Release`,
focused `test_barrier_friction_kernel_cuda` CTest,
`pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
(4 passed), and `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`.
The refreshed benchmark packet is recorded in
`docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json`.

Resumed hand-off checkpoint (2026-06-11): the earlier stop-only hand-off left
barrier-Hessian packet work on
`simx/plan083-gpu-contact-candidate-packet`. A fresh session should inspect
`git status -sb` and `git log --oneline` as authoritative: the point-edge slice
is locally committed ahead of origin, and the point-triangle slice may be either
the next local commit or dirty WIP depending on where the previous session
stopped. Before committing, run the required lint/build/test gates. Do not open
another PLAN-083 PR; continue using #2978 as the single consolidated review
unit.

No-verification handoff update (2026-06-11): the maintainer explicitly stopped
implementation and asked for hand-off only, with no further verification. The
current hand-off package on #2978 adds a private CUDA point-point
barrier-Hessian packet slice: private input/result contracts,
`evaluatePointPointBarrierHessiansCuda`, a CUDA kernel/launcher, fixed and
generated CPU/CUDA parity coverage, benchmark packet rows, packet writer/test
coverage, and this dev-task handoff text. Durable plan sidecars now record the
point-point barrier-Hessian row as in-progress evidence in
`gpu-parity-packet.json`, `completion-audit.md`, `paper-deck-manifest.md`, and
`docs/plans/dashboard.md`. The stop-only handoff package itself did not run
lint, build, test, benchmark, hosted-CI polling, or review actions.

Latest PR #2978 checkpoint: the current branch adds private CUDA
point-triangle primitive barrier-gradient parity plus point-triangle,
edge-edge, point-edge, and point-point tangent-stencil parity, plus a
point-triangle, point-point, point-edge, and edge-edge primitive
barrier-Hessian packet rows and point-triangle/point-point/point-edge/edge-edge
Hessian PSD-projection parity to
the existing
barrier/friction local-kernel packet. This closes the first primitive-gradient,
primitive-family tangent-stencil, primitive point-triangle/point-point/
point-edge Hessian, and primitive-family PSD-projected Hessian portions of that
row while keeping broader sparse Hessian assembly, runtime contact rows, and
the overall speedup gate as future evidence. Earlier Codex review threads for the
degenerate-triangle contact predicate and winding-independent point-triangle
CCD were resolved without bot replies.

Continuation checkpoint (2026-06-11): the stop-handoff dirty worktree was
inspected and continued on `simx/plan083-gpu-contact-candidate-packet`. The
branch contains the CUDA point-triangle barrier-gradient evaluator, focused
CUDA unit coverage, benchmark packet writer/test updates, and honest
plan/dev-task text that keeps the row `in-progress`. Validation evidence for
this slice:
`pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
(4 passed), `pixi run -e cuda build-cuda Release`, focused
`test_barrier_friction_kernel_cuda` CTest, `pixi run -e cuda
bm-plan083-gpu-barrier-friction-packet`, `pixi run python
scripts/check_plan083_gpu_parity_packet.py`, `pixi run python
scripts/check_plan083_completion_audit.py`, and the focused Python packet/audit
pytest trio (13 passed). A fresh session should preserve the single #2978
branch/PR discipline, confirm hosted CI and fresh review for the current head,
and push further PLAN-083 work only to #2978.

Critical hand-off checkpoint (2026-06-11): maintainer explicitly instructed the
agent to stop implementation and focus only on hand-off, with no further
verification. The active branch remains
`simx/plan083-gpu-contact-candidate-packet` and the active PR remains #2978.
Do not open another PLAN-083 PR or revive former stack branches. This hand-off
captures the current point-point barrier-Hessian slice on the same consolidated
branch, building on the already-pushed point-edge/point-point tangent-stencil
slice. The new slice adds CUDA input/result contracts, device-buffer wrapper,
kernel/launcher, unit parity coverage, benchmark packet coverage, packet
writer/test updates, and honest dev-task text that keeps the row
`in-progress` until broader Hessian assembly, PSD coupling, runtime contact
rows, and the top-level speedup gate are proven.

Last gathered evidence before the critical stop request: `pixi run lint`,
`pixi run python -m pytest
tests/test_plan083_gpu_barrier_friction_packet.py -q` (4 passed), `pixi run -e
cuda build-cuda Release`, focused `test_barrier_friction_kernel_cuda` CTest,
`pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`, the packet/audit
checker pair, the focused Python packet/audit pytest trio (13 passed), and
`git diff --check`. The updated packet measured point-edge tangent-stencil
`max_result_abs_error=1.1268763699945339e-14` and
`speedup=1.623508438779482x`, point-point tangent-stencil
`max_result_abs_error=3.3306690738754696e-16` and
`speedup=0.9786534468698057x`, top-level
`max_result_abs_error=3.982848877516439e-14`, and top-level
`speedup=0.9375876271274418x` with `meets_speedup_gate=false`. The overall
packet remains `in-progress`. A later point-point barrier-Hessian run, also
before the critical stop request, passed the packet pytest, CUDA build, focused
CUDA CTest, and barrier/friction benchmark packet; it measured
`point_point_barrier_hessian.max_result_abs_error=2.4868995751603507e-14`,
`point_point_barrier_hessian.speedup=2.2036790873726364x`, and top-level
`speedup=0.18351053106151646x` with `meets_speedup_gate=false`. The overall
row remains `in-progress` because broader Hessian assembly, PSD coupling,
runtime contact rows, and the top-level/runtime speedup gate remain future
evidence. No lint, build, test, benchmark, PR-check polling, or fresh review
request was run after the maintainer's critical stop request.

Fresh validation checkpoint (2026-06-11): after continuation resumed, the
branch passed `pixi run python scripts/check_plan083_gpu_parity_packet.py`,
`pixi run python scripts/check_plan083_completion_audit.py`, focused
packet/audit pytest (13 passed), `pixi run lint`, `pixi run build`, `pixi run
test-unit` (161/161), `pixi run -e cuda build-cuda Release`, focused
`test_barrier_friction_kernel_cuda` CTest, and `pixi run -e cuda
bm-plan083-gpu-barrier-friction-packet`. The refreshed packet measured
`point_point_barrier_hessian.max_result_abs_error=2.4868995751603507e-14`,
`point_point_barrier_hessian.speedup=1.1162432610528892x`, and top-level
`speedup=0.4709699971084976x` with `meets_speedup_gate=false`; keep the row
`in-progress`.

Continuation checkpoint (2026-06-11, edge-edge tangent-stencil slice): after
implementation resumed, the same #2978 branch added private CUDA edge-edge
tangent-stencil parity to the barrier/friction packet. Focused evidence passed
`pixi run python -m pytest tests/test_plan083_gpu_barrier_friction_packet.py -q`
(4 passed), `pixi run -e cuda build-cuda Release`, focused
`test_barrier_friction_kernel_cuda` CTest, and `pixi run -e cuda
bm-plan083-gpu-barrier-friction-packet`. The updated packet measured edge-edge
tangent-stencil `max_result_abs_error=8.881784197001252e-16` and
`speedup=1.339038677334765x`, but the overall packet remains `in-progress`
because the scalar local-kernel subrow missed the speedup gate in that run
(`speedup=0.11388679300181546x`) and because point-edge/point-point tangent
bases, Hessian assembly, PSD coupling, and runtime contact rows remain future
evidence.

Continuation checkpoint (2026-06-11, point-edge/point-point tangent-stencil
slice): the formerly unverified hand-off WIP on #2978 was validated and fixed.
The branch now adds private CUDA point-edge and point-point tangent-stencil
parity to the same barrier/friction packet, with a deterministic point-point
basis tie-break shared by CPU and CUDA. Focused evidence passed `pixi run
lint`, `pixi run python -m pytest
tests/test_plan083_gpu_barrier_friction_packet.py -q` (4 passed), `pixi run -e
cuda build-cuda Release`, focused `test_barrier_friction_kernel_cuda` CTest,
and `pixi run -e cuda bm-plan083-gpu-barrier-friction-packet`. The updated
packet measured point-edge tangent-stencil
`max_result_abs_error=1.1268763699945339e-14` and
`speedup=1.623508438779482x`, and point-point tangent-stencil
`max_result_abs_error=3.3306690738754696e-16` and
`speedup=0.9786534468698057x`. The overall barrier/friction row remains
`in-progress` because Hessian assembly, PSD coupling, runtime contact rows, and
the top-level/runtime speedup gate remain future evidence.

Use this folder's `README.md`, PLAN-083, `docs/plans/dashboard.md`, and the
current code as the live status. The branch-local "Current Branch" section below
is historical handoff context, not current checkout state. Treat IPC as the
representative solver-family name when PLAN-083's unified Newton-barrier method
is the most advanced shared IPC variant. Unified Newton-barrier work should
continue by promoting shared primitives and solver contracts consumed by rigid
IPC, deformable IPC, ABD, and future couplers rather than adding another public
solver stack.

The first Phase 3 shared-contract slice promotes fixed-size symmetric PSD
projection into `detail/newton_barrier`. Rigid IPC and ABD now use the same
helper for reduced/affine Hessian projection; this is intentionally smaller
than the existing deformable batched CPU/CUDA PSD backend and does not rename or
generalize that backend yet.

The PSD backend wrapper slice promotes only the solver-family owner name for
the existing deformable batched CPU/CUDA seam: `detail/newton_barrier` now wraps
the compute PSD backend, and deformable projected-Newton Hessian batching calls
through that internal owner. The core CPU/CUDA backend hooks, acceleration
control, and public compute names stay unchanged.

The sufficient-decrease policy slice promotes only the scalar Armijo/backtracking
policy that rigid IPC and deformable projected-Newton line search already share:
default sufficient-decrease factor, default backtracking scale, factor/scale
sanitization, and the finite candidate-value threshold check. Rigid IPC still
owns best-decreasing fallback, kinematic-candidate rejection, projected-Newton
result status, and solve stats. Deformable IPC still owns its CCD limiter chain,
steepest-descent fallback, minimum-step loop, and per-body solver diagnostics.

A Phase 3 slice promoted the first shared line-search option/stat
contract into `detail/newton_barrier`. Rigid IPC and deformable continuous
collision detection now share option defaults, primitive check counters, and the
stats accumulation helper while keeping their distinct result semantics and CCD
implementations in variant-local owners.

The positive-step predicate slice promoted only
`allowsPositiveLineSearchStep(stepBound, indeterminate)` into
`detail/newton_barrier`. Rigid IPC and deformable continuous collision result
methods route through it, but hit/limited fields and limiting-primitive payloads
remain variant-local.

The native-CCD option adapter slice merged as PR #2942. Rigid IPC and
deformable continuous collision detection now use one `LineSearchOptions` ->
`CcdOption` adapter for non-negative separation/tolerance clamping, positive
iteration count clamping, and conservative advancement selection, while their
CCD implementations, hit/limited result fields, and limiting-primitive payloads
remain variant-local.

The line-search step-scale slice promoted the shared line-search step-scale
policy into `detail/newton_barrier`. Rigid IPC Newton step scaling and
deformable/world CCD limiters now share finite `[0, 1]` step-bound clamping,
safety-scale handling, and the strictly-interior CCD fraction used before
applying a line-search candidate. Variant-local owners still keep their
hit/limited payloads, candidate identities, and zero-step diagnostic counters.

The benchmark-packet utility slice promoted the first shared packet contract:
Google Benchmark row-name canonicalization, timing-unit conversion, median-row
lookup, and timing-field validation now live in
`scripts/benchmark_packet_utils.py`. The ABD comparison packet checker and the
Phase 5 GPU packet checker share those row-level rules while keeping their
packet-specific expected rows, metadata, speedup gates, and evidence flags in
their variant owners. The shared canonical row identity helper is also consumed
by the Phase 5 CUDA packet writer so packet generation and validation strip
Google Benchmark repeat/aggregate suffixes the same way.

The native-CCD primitive outcome accounting scout promoted only the outcome
accounting that is identical across rigid IPC and deformable CCD: hit/miss/
indeterminate counter updates, hit and indeterminate time-of-impact clamping to
`[0, 1]`, and zero-step counting. Limiting-payload ownership,
indeterminate-result policy, and line-search result structs remain
variant-local.

The native-CCD zero-step diagnostic follow-up keeps that shared accounting
contract honest for indeterminate primitive queries: if a native CCD backend can
only prove an indeterminate zero step, `detail/newton_barrier` now increments
the same zero-step counter used for zero-time hits. Rigid IPC and deformable CCD
still own their distinct limiting payloads and indeterminate-result policies.

The Phase 3 closeout routes deformable projected-Newton backtracking through
the shared Newton-barrier default scale and routes rigid IPC's option defaults
through the same shared scalar constants without moving projected-Newton
result/status terminology, line-search result payloads, solver diagnostics, or
additional benchmark-schema contracts out of their variant owners.

The implementation-roadmap Phase 2 closeout is now branch-local on
`simx/plan083-phase-2-shared-solver-contracts`: full-step line-search
feasibility, projected-Newton residual/tolerance diagnostics, lagged-friction
work diagnostics, and benchmark packet timing schema helpers are shared under
their internal owners with focused cross-variant tests. This completes the
Shared Solver Contracts phase; do not open more sub-item PRs for this phase.

The implementation-roadmap Phase 3 closeout is now branch-local on
`simx/plan083-phase3-articulation-constraints`:
`detail/newton_barrier/articulation_constraint.hpp` owns internal
point-connection, fixed-point, hinge-axis, cone-twist, sliding,
relative-sliding, distance, bounded-distance, sliding-range, and
rotation-range contracts. Focused `test_newton_barrier_primitives` coverage
checks residuals, finite-difference Jacobians/gradients, range feasibility, and
PSD Hessian approximations. Keep this as one phase-scoped PR targeting `main`.

The implementation-roadmap Phase 4 closeout is now branch-local on
`simx/plan083-phase4-restitution-bdf2`:
`detail/newton_barrier/restitution_damping.hpp` owns internal restitution
target, BDF-2 history/inertial/velocity-update, falling-box energy diagnostic,
and semi-implicit Rayleigh damping contracts. Focused
`test_newton_barrier_primitives` coverage checks restart/serialization,
BDF-2 velocity accuracy, timestep/Young's-modulus/barrier/gravity diagnostic
sweeps, PSD-projected damping, and hinge damping evidence. Existing rigid
stepping defaults remain unchanged.

The implementation-roadmap Phase 5 closeout is now branch-local on
`simx/plan083-phase5-mixed-domain-coupling`:
`detail/newton_barrier/mixed_domain_coupling.hpp` owns internal mixed-domain
surface adapters, deterministic primitive candidate generation, point-pair CCD
reduction, barrier/friction diagnostics, oracle-owner routing, and restart
candidate keys for rigid, deformable, affine, particle, rod, shell, and
codimensional domains. Focused `test_newton_barrier_primitives` coverage checks
all domain adapters, primitive-family candidates, finite barrier energy,
friction diagnostics, variant-owner preservation, conservative point CCD, and
deterministic restart keys. Runtime coupler promotion, py-demos, and GPU
packets remain future phases.

The implementation-roadmap Phase 6 closeout is now branch-local on
`simx/plan083-phase6-cpu-scenes-pydemos`: the py-demo catalog exposes
launchable planned PLAN-083 CPU corpus placeholders for mixed, constraint,
robot, and ABD categories, and
`docs/plans/083-unified-newton-barrier-multibody/cpu-scene-corpus.json` records
each paper/deck scene row's smoke command, long-horizon capture command,
benchmark/profile packet path, expected invariant, and current limitation.
`scripts/check_plan083_cpu_scene_corpus.py` plus focused Python tests guard that
the row map stays explicit without claiming runtime paper-scene reproduction.

The implementation-roadmap Phase 7 closeout is now branch-local on
`simx/plan083-phase7-gpu-parity`:
`docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json`
records the private GPU parity packet rows for contact stencils/candidate
filtering, CCD/line search, barrier/friction local kernels, PSD projection,
assembly/linear solve, and scene-level parity/speedup. The checked packet keeps
same-scene parity, tolerance, setup/transfer/readback timing, kernel/solve
timing, speedup, and no-public-API policies explicit while leaving rows without
landed kernels as planned/in-progress limitations instead of GPU speed claims.
Local validation on the Phase 7 branch passed the packet checker, focused
packet tests, `pixi run lint`, `pixi run build`, and
`pixi run -e cuda test-cuda` on the visible RTX 4080 Laptop GPU.

Implementation-roadmap Phases 3-8 landed together in PR #2960. The Phase 8
audit at
`docs/plans/083-unified-newton-barrier-multibody/completion-audit.md` records
that PLAN-083 is not complete because reduced in-progress manifest rows and
explicit CPU/GPU scene-packet limitations remain. The audit intentionally blocks retiring
`docs/dev_tasks/unified_newton_barrier_multibody/` until a maintainer decides
whether the remaining work stays active there or moves fully into durable plan
sidecars.

The follow-up PSD projection packet slice adds
`bm_plan083_gpu_psd_projection` plus
`scripts/write_plan083_gpu_psd_packet.py`. On 2026-06-09,
`pixi run -e cuda bm-plan083-gpu-psd-packet` measured the 4096-block 12x12 PSD
row with max error `2.4868995751603507e-14` and `4.451557656446166x` speedup
over the CPU reference. This moves only the local PSD projection row to
`measured`; contact candidates (point-triangle and edge-edge preassembled
stencils), CCD/line search, barrier/friction kernels, assembly/solve, and
reduced scene state-batch rollout now have in-progress private packets, while
full scene-level GPU `World::step` speedups remain unproven.

The follow-up barrier-force diagnostic slice adds
`NewtonBarrierPrimitives.BarrierForceCurveCapturesKappaSensitivity`, which
checks Fig. 17-style activation cutoff, kappa scaling, and near-contact slope
growth against the private Newton-barrier scalar kernel. This moves only
`unb-fig-17` to `in-progress`; runtime force plots and accepted default-kappa
policy remain future evidence.

The follow-up articulation manifest alignment moves `unb-fig-05`,
`unb-fig-06`, `unb-fig-07`, `unb-fig-08`, `unb-fig-12`, and `unb-fig-13` to
`in-progress` based on the landed Phase 3 private articulation diagnostics.
Those rows still do not claim runtime articulated scenes, cloth snapshots,
direct internal-contact speedups, or scaling packets.

The runtime-wiring follow-up is branch-local on
`feature/newton-barrier-runtime-wiring`: rigid IPC `World::step` consumes the
landed point-connection/fixed and hinge-axis contracts, exposes opt-in BDF-2
integration through the rigid IPC contact stage, routes deformable surfaces as
fixed obstacles through the mixed-domain surface adapter, and runs a reduced
hanging-bridge py-demo smoke scene through `World::step` plus headless capture.
This branch intentionally keeps paper-scale mixed-domain scenes, sparse
equality-constraint solving, complete CPU benchmark packets, and GPU parity
speed claims as remaining PLAN-083 work.

The CPU corpus evidence follow-up is branch-local on
`feature/newton-barrier-corpus-evidence`: the first packets cover only the
reduced lying-flat, hanging-bridge, pulley, umbrella, terrain vehicle, ragdoll,
nunchaku, windmill, Candy, and precession runtime smoke paths plus reduced
nunchaku scaling, timing-breakdown, and Table 2 setup/statistics packets with
`bm_plan083_cpu_scene_corpus` and
`scripts/write_plan083_cpu_scene_packet.py`. They validate finite reduced-scene
runtime stepping plus fixed/revolute equality residuals, reduced
hinge/contact/terrain/ragdoll/rolling/deformable diagnostics, and aggregate
wall-time counters, but they are not paper-scale rod/codimensional coupling,
lying-flat rigid-ring/deformable-torus/rod/ragdoll coupling, umbrella
cloth/sliding reproduction, affine packing, twisted shell, self-contact parity,
cone-twist range validation, 60-ragdoll scale, terrain navigation,
angular-velocity sweep validation, analytical pulley force comparison,
Bullet/reference comparison, Fig. 24 subphase timing, full Table 2 reproduction,
coupled-contact scaling evidence, or a completion claim.

The same branch adds a reduced affine point-triangle solved-state diagnostic
under `detail/affine_body_dynamics`: a private micro-solve combines inertial,
barrier, and orthogonality terms for one dynamic affine body against a static
triangle and adds a benchmark packet row. This is internal solved-state
evidence for `abd-alg-affine-body`; the packet checker now validates the
micro-solve residual/convergence counters. It is not an ABD runtime solver,
scene corpus row, or paper-scale ABD comparison.

The same branch now adds the sparse equality change-of-variable contract in
`detail/newton_barrier/change_of_variable.hpp`, with focused primitive tests for
rank detection, residual satisfaction, sparse free-coordinate basis structure,
and KKT solve equivalence, and routes the rigid IPC equality step through that
private contract. This moves `unb-alg-change-var` to `in-progress` but does not
claim paper-scale pulley/sliding scene reproduction.

The ABD runtime-evidence follow-up is branch-local on
`simx/plan083-phase6-abd-runtime-evidence`: the private ABD point-triangle
micro-solve now has a reduced runtime-step helper that builds an inertial target
from timestep, velocity, and gravity, then updates affine linear/affine
velocities from the solved state. The branch also adds a two-body affine
point-triangle pair runtime-step helper for dynamic affine/affine contact. The
reduced house-of-cards packet row for `abd-vs-rigid-cards` and the reduced
wrecking-ball packet row for `abd-vs-rigid-wreck` validate convergence,
active-barrier, objective-decrease, contact-distance, and velocity-update
counters. The same branch adds reduced chain-net packets for `abd-chain-8`,
`abd-chain-16`, and `abd-chain-96` using generated dynamic affine
point-triangle pair runtime steps. These packets are not card-stack,
wrecking-ball, or chain-net asset imports, rigid IPC timing comparisons,
paper-scale ABD solvers, GPU packets, or completion claims.

The remaining ABD manifest-packet follow-up is branch-local on
`simx/plan083-abd-remaining-packets`: reduced complex-geometry and
ABD/FEM coupled micro-solve packets now cover the last planned manifest rows
with generated affine pair runtime steps plus a deformable IPC smoke sidecar,
mixed candidate diagnostics, and affine/deformable contact micro-solve evidence
for the FEM row. These packets move the rows only to in-progress; they do not
claim paper-scale complex geometry, full runtime affine/FEM coupling, accepted
reference timings, or completion.

## Last Session Summary

Current slice: PR #2978 is the only active PLAN-083 follow-up PR. It contains
the former private GPU packet stack plus reduced ABD packet work, with atomic
commits for contact candidates, CCD/line-search, barrier/friction,
assembly/solve, scene state-batch parity, ABD complex geometry, ABD/FEM
coupling, and the latest CUDA CCD review fix. Preserve that this is runtime
smoke/correctness evidence, not paper-scale completion.

PLAN-083 was created for the unified Newton-barrier multibody solver family,
covering the supplied unified Newton barrier paper, the ABD deck, existing
deformable IPC work, existing rigid IPC work, CPU/GPU evidence, and py-demos
obligations. Phase 1 promoted the pure distance, barrier, and tangent-stencil
primitives into `detail/newton_barrier`; Phase 2 now has an internal
`detail/affine_body_dynamics` foundation with affine state/surface mapping,
four shared primitive barrier chain-rule rows, orthogonality energy, rigid
gradient/Hessian equivalence, affine primitive-family friction equivalence rows
through the shared tangent-displacement friction kernel,
`test_affine_body_dynamics`, and the first `bm_affine_body_dynamics` benchmark
packet. The active follow-up extends that packet with a reduced solved-state
point-triangle micro-solve diagnostic without promoting an ABD runtime solver.

The first Phase 3 contract slice was merged as PR #2936. It added a shared
fixed-size PSD projection helper, routed rigid IPC and ABD Hessian projection
through it, and added focused Newton-barrier primitive tests for eigenvalue
clamping and input symmetrization.

The line-search stats slice merged as PR #2937. It added
`detail/newton_barrier/line_search.hpp`, aliases rigid IPC and deformable CCD
option/stat types to the shared owner, and keeps line-search result semantics
variant-local.

The positive-step predicate slice merged as PR #2938. It promoted
`allowsPositiveLineSearchStep(stepBound, indeterminate)` into
`detail/newton_barrier`, routing rigid IPC and deformable CCD result methods
through it without moving hit/limited result ownership.

The benchmark-packet utility slice merged as PR #2940. It promoted
Google Benchmark row parsing into `scripts/benchmark_packet_utils.py`, routing
the ABD comparison packet checker plus the Phase 5 GPU packet checker through
the shared utility while keeping packet-specific metadata and gates in their
owners. The follow-up row-identity slice routes the Phase 5 CUDA packet writer
through the same canonical row identity helper and removes its duplicate parser.
Focused local validation passed `pixi run python -m pytest
tests/test_benchmark_packet_utils.py` and `pixi run lint`.

The line-search step-scale slice merged as PR #2943. It is intentionally
smaller than a line-search result or projected-Newton diagnostics merge:
current evidence shows rigid and deformable share finite step-bound scaling and
the strictly-interior CCD fraction, but not yet full line-search result
payloads, projected-Newton status terminology, or solver-loop diagnostics.
Focused local validation passed `pixi run lint`, the focused
`test_newton_barrier_primitives` / `test_rigid_ipc_barrier` / `test_world`
CTest entries, `pixi run build`, and `pixi run test-unit`.

The native-CCD primitive outcome accounting slice merged as PR #2945 after PR
#2943 landed. Focused local validation passed the
`test_newton_barrier_primitives`, `test_continuous_collision_step`, and
`test_rigid_ipc_barrier` build/CTest entries plus `pixi run lint`. Stronger
local validation also passed `pixi run build`, `pixi run test-unit`,
`pixi run test-all`, and `pixi run -e cuda test-all`. Both full gates passed;
the docs phase still emits the existing `dartpy._world_render_bridge` autodoc
warnings.

The PSD backend wrapper slice is on
`simx/shared-newton-barrier-psd-backend-wrapper`, merged as PR #2946 after
PR #2945 landed. Focused local validation passed
`test_deformable_psd_backend` and `test_world` build/CTest entries.

The sufficient-decrease policy slice is on
`simx/shared-newton-barrier-sufficient-decrease`, retargeted to `main` after
PR #2946 landed. Focused local validation passed `pixi run lint` plus the
`test_newton_barrier_primitives`, `test_rigid_ipc_barrier`, and `test_world`
build/CTest entries.

## Current Branch

`simx/plan083-gpu-contact-candidate-packet` - the single consolidated PLAN-083
follow-up PR for the remaining private packet work (#2978). This branch now
carries point-triangle and edge-edge contact-stencil filtering plus
brute-force all-pairs point-triangle and edge-edge candidate-mask packets plus
motion-aware swept-AABB point-triangle and edge-edge candidate-list packets,
compact runtime sweep-buffer endpoint-distance packets, winding-independent
endpoint-linear point-triangle CCD, edge-edge CCD/line-search packets, reduced
scene-owned runtime candidate-buffer packets, reduced scene-owned runtime sweep
broad-phase packets, scalar barrier/friction local kernels plus point-triangle
primitive barrier gradients and point-triangle/edge-edge tangent stencils plus
point-edge/point-point tangent stencils, point-triangle, point-point,
point-edge, and edge-edge primitive barrier-Hessian parity,
point-triangle/point-point/point-edge/edge-edge primitive barrier-Hessian PSD-projection
parity, reduced scene-owned point-triangle, point-edge, point-point, and
edge-edge barrier-Hessian runtime rows,
reduced assembly/solve parity, reduced scene state-batch parity, and reduced
ABD complex-geometry/FEM coupling evidence. Keep rows `in-progress` unless
their full row policy is satisfied: additional runtime contact rows, GPU
sweep-and-prune broad-phase construction, rigid curved trajectories, full
runtime scene filtering/line search, GPU `World::step` contact candidate
construction, equality-reduced/global sparse assembly and solving, paper-scale
assets, and accepted reference timings remain future evidence.

## Immediate Next Step

Resume only from `simx/plan083-gpu-contact-candidate-packet` / PR #2978. Keep
remaining PLAN-083 follow-up work on the same consolidated branch/PR instead of
reviving former stacked branches. The next contact-candidate packet gaps are
runtime scene filtering and speedup-gate work; the next barrier/friction packet
gaps are additional runtime contact rows, broader sparse Hessian assembly, and
speedup-gate work; the next assembly/solve gaps are
equality reduction, global sparse factorization, runtime scene rows, and
speedup-gate work. Do not mark these rows measured until the top-level speed
gate and runtime evidence are proven. Keep the dev-task folder active because
PLAN-083 acceptance criteria are still unmet. If the task later moves out of
this folder, get maintainer direction before deleting it and keep the remaining
planned manifest plus in-progress CPU/GPU/scene limitations in durable
sidecars.

## Context That Would Be Lost

- Rigid IPC now includes the shared Newton-barrier primitive owner directly for
  barrier and tangent math.
- Preserve old deformable-contact include paths as forwarding headers to avoid
  unnecessary conflicts with active PLAN-081 work.
- Keep benchmark binary names such as `bm_ipc_distance_kernels`,
  `bm_ipc_barrier_kernel`, `bm_ipc_tangent_stencil`, and
  `bm_rigid_ipc_solver`.
- The primitive contract has old/new alias parity tests and a rigid consumer
  test in `test_newton_barrier_primitives`.
- `detail/affine_body_dynamics` currently uses the affine map `x = a + A X`;
  its primitive Hessian is a pure `J^T H J` chain rule. The rigid Hessian oracle
  in `test_affine_body_dynamics` projects onto the rigid tangent space and adds
  the rotation-vector curvature correction before comparing to rigid IPC.
- `test_affine_body_dynamics` covers finite-difference vertex Jacobians,
  point-triangle/point-edge/edge-edge/point-point affine barrier derivatives,
  point-triangle/point-edge/edge-edge/point-point affine friction derivatives,
  orthogonality energy derivatives, and rigid value/gradient/Hessian
  equivalence for barrier and friction rows.
- The tangent-displacement friction kernel now lives under
  `detail/newton_barrier` so rigid IPC and ABD share the same Coulomb smoothing
  branch before reduced/affine chain rules.
- `detail/newton_barrier/line_search.hpp` owns the shared line-search
  option/stat types, positive/full-step predicates, the conservative native CCD
  option adapter, and the step-scale helpers. Do not move full line-search
  result structs there until rigid, deformable, ABD, or another variant prove
  identical result semantics.
- `detail/newton_barrier/restitution_damping.hpp` owns the first internal
  Phase 4 contracts for restitution targets, BDF-2 history and velocity
  updates, falling-box diagnostic sweeps, and PSD-projected Rayleigh damping.
  Do not wire it into runtime defaults until a separate promotion gate proves
  full solver parity.
- `detail/newton_barrier/mixed_domain_coupling.hpp` owns the first internal
  Phase 5 mixed-domain seam for shared surface adapters, primitive candidate
  keys, point-pair CCD reduction, and diagnostic ownership routing. Do not
  replace variant-local runtime contact buffers until scene-level mixed-domain
  stepping tests and py-demo evidence land.
- `docs/plans/083-unified-newton-barrier-multibody/cpu-scene-corpus.json` owns
  the Phase 6 row map for py-demo placeholders, smoke commands, long-horizon
  visual capture commands, benchmark/profile packet paths, invariants, and
  limitations. The placeholders are launchable catalog entries, not runtime
  paper-scene reproductions.
- `docs/plans/083-unified-newton-barrier-multibody/gpu-parity-packet.json` owns
  the Phase 7 private GPU row map. Its checked rows are evidence contracts for
  same-scene parity, timing, tolerance, speedup, and no-public-API policy; they
  are not GPU kernel or public backend promotion claims.
- `scripts/benchmark_packet_utils.py` owns the shared Google Benchmark row
  parsing utilities plus the per-step/subphase timing schema for packet
  validators and writers. Keep packet-specific metadata and go/no-go gates in
  each checker until more variants prove identical semantics.
- `bm_affine_body_dynamics` currently contains the first ABD benchmark packet:
  affine point-triangle barrier mapping, matched rigid IPC point-triangle oracle
  row, orthogonality energy, and the reduced point-triangle micro-solve
  residual counters.
- `scripts/check_abd_comparison_packet.py` is the reproducible packet checker
  for those rows; it validates micro-solve convergence counters but
  deliberately does not validate paper-scale ABD scene timings yet.
- The current packet is enough to proceed to Phase 3 shared-contract scouting;
  it does not require a two-body solved-state micro-solve first.
- Do not expose `detail/newton_barrier` through public headers or dartpy
  bindings.
- Do not expose ABD through public headers or dartpy bindings until runtime
  and benchmark evidence exists.
- Phase 1 validation passed: `pixi run lint`,
  `pixi run build-simulation-tests`,
  `pixi run test-simulation`, `pixi run check-api-boundaries`, and
  the four benchmark smokes listed in the dev-task README.
- Phase 2 validation so far passed: `pixi run build-simulation-tests`,
  `pixi run test-simulation` (64/64),
  `pixi run check-api-boundaries`, `pixi run check-docs-policy`,
  `pixi run lint`, and
  `pixi run bm bm_affine_body_dynamics -- --benchmark_min_time=0.05s`.
  The micro-packet slice adds `pixi run bm-abd-comparison-packet`.

## How To Resume

```bash
git status --short --branch
git switch simx/plan083-gpu-contact-candidate-packet
gh pr view 2978 --repo dartsim/dart --json headRefName,headRefOid,mergeStateStatus,reviewDecision,statusCheckRollup
sed -n '1,220p' docs/dev_tasks/unified_newton_barrier_multibody/README.md
sed -n '1,220p' docs/plans/083-unified-newton-barrier-multibody/implementation-roadmap.md
```

Then continue on #2978 only. Before any push, sync `main` into the PR branch,
verify the audit sidecars, and keep implementation slices tied to planned
CPU/GPU/scene rows. Do not describe planned GPU rows as speed claims or public
backend promotion.
