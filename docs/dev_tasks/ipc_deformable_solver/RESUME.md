# Resume: IPC Deformable Solver

## Current Reality (2026-07-04)

Live status is this folder's `README.md`, the durable PLAN-081 owner docs
(`docs/plans/081-deformable-implicit-barrier-solver/ipc-parity-roadmap.md`,
`ipc-paper-figure-showcase.md`, `ipc-paper-gap-audit.md`,
`ipc_scene_corpus_manifest.json`), `docs/plans/dashboard.md`, and the current
code. **All branch/PR handoff sections below (2026-05/06) are historical
archaeology** — their branch names, "current branch" language, and push
instructions are stale. In particular, PR #2821 (matrix-free CG + CG
diagnostics) merged long ago (`74338577982`); `main` is ~400 PRs ahead.

Completion-audit result (this session): the dev task is the active working
surface for **PLAN-081 (Status: Active, Horizon: Now)** and is **not
retire-able unilaterally** — the dashboard records that dev-task retirement for
incomplete IPC-family plans needs maintainer direction, and PLAN-081 is far from
parity (figure showcase: 24 planned / 3 in-progress / 6 landed /
3 reference-beaten; M7 scale/perf/GPU, the asset pipeline, and codim obstacles
remain). Continue in bounded, verifiable M7 slices.

Current deformable work keeps entering through the DART-owned deformable solver
families, shared Newton-barrier/VBD components, the built-in World schedule, and
facade-safe `World`/`DeformableBodyOptions`/diagnostics surfaces. Route any new
shared distance/barrier/CCD/Newton primitive through PLAN-083
(`docs/dev_tasks/unified_newton_barrier_multibody/`) before duplicating it here.

### Landed Slice: Fig-23 peak-contacts diagnostic — MERGED (PR #3257)

`DeformableSolverDiagnostics.maxActiveContactCount` (dartpy
`max_active_contact_count`) + internal `DeformableSolverStats.maxActiveContactCount`
— the IPC Fig. 23 "max contacts per step" axis — merged to `main` as
`1819b801228`. Captured as the delta of the existing
`selfContactBarrierActiveContacts` counter around the single outer-iteration
objective evaluation that passes it, so the solve is byte-identical (no hot-path
signature change). Surfaced on `BM_DeformableSelfContactBarrierStage` +
`ipc_deformable_cg_contact` demo; C++ peak-invariant + public-propagation
regressions. Changelog: no entry (family-level bullet covers it).

### Active Slice (2026-07-04): Fig-23 statistics packet (shape parity)

Branch `feature/ipc-deformable-fig23-statistics-packet` (off `main` after #3257
merged). Adds a machine-checkable **Fig-23-shaped statistics packet** that
distils the `bm_deformable_body` JSON into per-scene Fig-23 axes (per-step
Newton/CG effort, CG residual, assembled sparse-Hessian footprint, per-step wall
time, and the active-contact statistics — consuming the #3257
`max_active_contacts` axis) over DART-runnable scenes. **Pure-additive Python**,
zero C++/behavior change: `scripts/write_plan081_deformable_fig23_packet.py` +
`docs/plans/081-deformable-implicit-barrier-solver/fig23_deformable_statistics_corpus.json`
(manifest, `paper_scale: false`) + `tests/test_plan081_deformable_fig23_packet.py`
(7 pytest cases), reusing the plan091/`benchmark_packet_utils` pattern. The
packet output goes to gitignored `.benchmark_results/plan081/` (not committed).
Verified end-to-end against a real `bm_deformable_body` run (7 rows; the
direct-vs-CG crossover and matrix-free zero-footprint are visible). Scoped
honestly as shape parity, not paper parity: paper-scale scenes + the Table-1 CPU
reference comparison remain blocked on the M4 asset pipeline. Changelog decision:
**no entry** (internal evidence tooling, no user-facing API/behavior change).
Push + PR only after maintainer approval.

## Current State (2026-05-31) — PLAN-081 M1–M6 COMPLETE; M7 matrix-free CG in progress

This session drove the experimental deformable solver from "mass-spring scaffold"
to broad IPC paper-parity (Li et al. 2020). All work shipped as one-PR-per-slice
off `main` (milestone DART 7.0, `pixi run test-all` 6/6, admin-squash-merged, no
AI attribution). Net status by milestone:

- **M1 FEM elasticity — COMPLETE.** Stable neo-Hookean + fixed-corotational (FCR)
  tetrahedral kernels (`detail/deformable_elasticity/fem_tet_element.hpp`), both
  FD-validated, opt-in via `DeformableMaterialProperties.useFiniteElementElasticity`
  / `useFixedCorotationalElasticity`. FCR uses the **exact analytic Hessian**
  (rotation-gradient `(tr(S)I−S)w = axl(RᵀδF−δFᵀR)`), PSD-projected by the solver,
  Gauss-Newton fallback for inverted elements. ~7× faster FCR step (to neo-Hookean
  parity); benchmarked in wall-time **and** Newton iterations/step.
- **M2 obstacle barriers — COMPLETE.** Sphere + box + capsule static obstacles
  each get the clamped-log barrier force (energy + gradient + rank-1 radial
  Hessian) through the projected-Newton assembly.
- **M3 codim importers + capsule object — COMPLETE.** `.msh`/`.obj`/`.seg`/`.pt`
  importers (`io/gmsh_tet_mesh`, `io/obj_triangle_mesh`, `io/codim_mesh`); capsule
  (rod/wire) collision shape + obstacle barrier (the first codimensional object,
  barrier-only so cloth drapes freely).
- **M4 GMSH importer — COMPLETE** (`.msh` 2.x + 4.x).
- **M6 adaptive barrier stiffness — COMPLETE.** Opt-in
  `useAdaptiveBarrierStiffness`: per-step `κ = clamp((maxNodalMass/dt²)·d_hat², 25,
1e6)`; recovers the historical fixed κ=25 at unit mass, off is byte-identical.
- **M5 obstacle friction — COMPLETE.** Capsule friction works (barrier-only → free
  tangential slide). Sphere/box friction unblocked by the opt-in **barrier-only
  obstacle** mode (#2809): `DeformableObstaclePolicy::barrierOnly` maps to a
  `DeformableObstacleNoCcdTag` that `entt::exclude`s the obstacle from the
  surface-CCD collect view, so the clamped-log barrier alone prevents penetration
  while tangential sliding + friction survive (the CCD limiter no longer scales the
  whole step). The "right" CCD fix (limit only the normal approach, keep fast-motion
  CCD safety) remains a documented higher-risk follow-up, cf. #2732.
- **M7 scale + performance — four increments landed** (#2810/#2811/#2812/#2813)
  and a CG diagnostic follow-up in progress. (1) Opt-in **iterative
  conjugate-gradient projected-Newton linear solve**
  (`DeformableMaterialProperties.useIterativeLinearSolver`, #2810): reuses the
  sparse SPD Hessian assembly but solves with CG instead of `SimplicialLDLT`, so it
  never factorizes (memory ~O(nnz), gentler scaling than direct fill-in). Meshes
  above the direct node cap (20k) take CG automatically (ceiling raised to 1M
  nodes) instead of degrading to gradient descent; non-convergence falls back to
  steepest descent like the direct path. (2) **Incomplete-Cholesky preconditioner**
  (#2811): upgraded from diagonal (Jacobi); on stiff barrier contact it collapses
  the CG iteration count so the iterative path carries the solve within the cap
  (fallbacks drop below the direct solver's, fewer Newton iters/step). (3)
  **Chunky-3D scaling benchmark** (this PR): `BM_DeformableCube3d{Direct,Cg}Step`
  on a solid N^3 cube (wide Hessian bandwidth) makes the crossover measurable --
  direct 3D fill-in climbs super-linearly while IC-CG stays ~O(nnz); measured ~5x
  faster CG at ~4k nodes (11.3 vs 2.3 s/step). (4) Public iterative-solve
  diagnostics (#2813) expose whether a step used the CG path. The current
  branch extends that profiling surface with CG iteration/residual counters and
  an explicit matrix-free CG path with contact parity regressions against the
  direct sparse solve and sparse IC-CG. A CUDA backend exists for PSD projection
  and VBD (another track, #2781); automatic matrix-free selection for large
  meshes, an AMG preconditioner for the largest systems, on-device GPU
  assembly/solve, and the 688K-node Fig-22 run + Table-1 reference comparison
  remain.

Session PR train (all merged to `main`): #2787 FEM keystone, #2788 FEM twist,
#2789 FEM+ground, #2790 sphere barrier Hessian, #2791 box barrier, #2792/#2793
GMSH 2.x/4.x, #2794 FCR material, #2795 FCR benchmark+SVD-dedup, #2796 exact FCR
Hessian, #2797 solver diagnostics, #2798 FEM self-contact showcase, #2799
benchmark Newton-iters, #2800 `.obj` importer, #2802 `.seg`/`.pt` importers, #2804
capsule obstacle, #2805 adaptive stiffness, #2809 barrier-only obstacle (M5 done),
#2810 iterative-CG solve (M7 increment 1), #2811 incomplete-Cholesky
preconditioner (M7 increment 2), #2812 chunky-3D scaling benchmark (M7 increment
3), and #2813 public iterative-solve diagnostic (M7 increment 4). The IPC
Deformable (sx) py-demos category
now has ~21 scenes (latest: `ipc_deformable_cg_solver`, `ipc_deformable_cg_contact`).

### M7 Next (resume here)

M1–M6 + M5 are all complete; **M7 (scale + performance) is the only remaining
milestone.** Four increments landed (iterative CG solve #2810;
incomplete-Cholesky preconditioner #2811; chunky-3D scaling benchmark #2812;
public iterative-solve diagnostic #2813). The current branch is a small
profiling follow-up that adds CG iterations, residual estimates, and assembled
sparse-Hessian footprint counters, followed by an explicit matrix-free CG path
that applies local Hessian blocks directly with a block-Jacobi preconditioner.
The remaining M7 work, roughly in increasing-risk order:

1. **Matrix-free CG hardening.** The explicit
   `useMatrixFreeLinearSolver` path now skips sparse Hessian assembly and reports
   zero sparse-Hessian footprint, but it uses block-Jacobi rather than the sparse
   incomplete-Cholesky preconditioner. C++ ground-contact coverage now compares a
   contacting FEM cube across direct sparse, sparse IC-CG, and matrix-free CG;
   dartpy coverage compares direct and matrix-free contact settling. Next, harden
   it on larger contact-heavy meshes and decide when it can become the automatic
   path for very large meshes (Fig 22, 688K nodes).
2. **AMG / multigrid preconditioner** for the largest systems (beyond what
   incomplete-Cholesky handles).
3. **GPU assembly + solve.** Extend the existing CUDA PSD-projection backend
   (#2781) to the full deformable assembly + a GPU CG, beyond the current PSD
   offload.
4. **Profiling-grade benchmark + reference comparison.** Emit Fig-23-shaped
   per-scene statistics (avg/max contacts/step, Newton iters/step, peak memory,
   s/step) and the Table-1 CPU comparison vs the IPC reference. The
   `BM_DeformableCube3d{Direct,Cg}Step` chunky-3D benchmarks (this PR) already
   make the direct-vs-iterative crossover measurable; what remains is peak-memory
   tracking, the avg/max contacts-per-step axis on contact scenes, and the
   side-by-side comparison against the published IPC reference numbers.

### Conventions / gotchas (verified this session)

- One PR per slice off `main`; `pixi run test-all` must be 6/6; admin-squash-merge
  (NO `--delete-branch`); milestone DART 7.0; never add AI attribution.
- After `pixi run generate-stubs`, revert unrelated stubs unless the promoted
  DART 7 surface changed; keep the intended `simulation.pyi` diff.
- `pixi run check-api-boundaries` when a public dartpy binding changes (use
  `pixi run report-api-boundary-inventory` for the on-demand signal report).
- Adding a `CollisionShapeType` enum value breaks **every** switch on it under
  `-Werror=switch` — grep the **whole repo** (incl. `examples/`) for
  `case CollisionShapeType::Mesh`.
- C++ `DeformableBodyOptions` field is `surfaceTriangles` (camelCase); the dartpy
  alias is `surface_triangles`.
- `test-all`'s linting reflows `module.cpp` docstrings + `CHANGELOG.md` AFTER a
  `git add`; re-`git add` (or amend) before merging.
- The detailed live state also lives in the assistant memory
  `SESSION_HANDOFF.md`.

## Last Session Summary

The branch established a machine-checkable manifest for the audited upstream
`ipc-sim/IPC` scene corpus and added validation tooling for the 154 tracked
`.txt` scene paths. The manifest is a planning/data grounding artifact only; it
does not implement new solver, renderer, benchmark, or GUI behavior.

The follow-up mesh/material-state sub-slice adds optional surface and
tetrahedral topology, material validation, density-based lumped mass assembly
for tetrahedral bodies, boundary surface extraction, serialization, benchmark
counters, and GUI rendering from body-owned surface topology. It still steps
through the existing point-mass/spring path and must not be described as FEM,
mesh contact, or full IPC.

The scene/boundary/diagnostics sub-slice adds a contact-free upstream-style
scene text loader, a Gmsh 4.1 tetra-mesh subset importer, generated structural
spring replay edges, scripted DBC/NBC controls, binary restart continuity,
diagnostics JSON, replay/load benchmarks, and `experimental_deformable_gui
--deformable-scene` headless capture. It still ignores/reports contact and
friction directives and must not be described as IPC scene parity.

The primitive-distance kernel sub-slice starts PLAN-081 Phase 2 with internal
point-triangle and edge-edge squared-distance kernels, closest-feature
classification, gradients, the first solver-facing Hessian contract, IPC-style
edge-edge mollifier derivatives, feature-region regression tests, and
`bm_ipc_distance_kernels`.

The analytic-Hessian optimization sub-slice replaces the finite-difference
distance Hessian placeholder with feature-wise exact point-triangle and
edge-edge Hessian paths for vertex, edge, face, and interior closest features.
It is still scaffolding: tangent bases, solver-wired CCD line search, barrier
assembly, projected Newton, and friction are not implemented yet.

The barrier-kernel sub-slice adds internal IPC C2 clamped-log scalar barriers
over squared distances, raw analytic point-triangle and edge-edge barrier
gradients/Hessians through the distance kernels, and explicit-threshold
edge-edge mollifier product-rule derivatives. It is still scaffolding: barrier
stiffness adaptation, PSD projection, candidate-buffer assembly, solver-wired
CCD line search, projected Newton, and friction are not implemented yet.

The tangent-stencil sub-slice adds internal point-triangle, edge-edge,
point-edge, and point-point tangent bases, closest-point parameters, tangent
projection matrices, and tangent metric matrices for future IPC
contact/friction assembly. It is still scaffolding: friction energy,
friction gradients/Hessians, lagged friction convergence, solver-owned contact
caches, solver-wired CCD line search, projected Newton, and scene-level contact
behavior are not implemented yet.

The motion-aware candidate-culling sub-slice adds conservative start/end
swept-AABB point-triangle and edge-edge candidate assembly, preserving fast
crossings that static endpoint distance filters miss, and stores minimum
endpoint squared distance as representative metadata. It is still scaffolding:
the swept candidates are not wired into `World::step()`, solver-owned contact
buffers, barrier assembly, projected Newton, or friction.

The candidate-buffer reuse sub-slice adds reusable-output overloads for static
and motion-aware candidate builders. The overloads clear stale candidates and
stats while preserving candidate vector capacity, and the return-by-value
builders remain wrappers. It is still scaffolding: temporary sweep-item arrays
are rebuilt per call, and the buffers are not yet owned by `World::step()` or a
persistent IPC contact cache.

The per-body surface-contact CCD line-search sub-slice wires motion-aware
same-body surface candidates into `DeformableDynamicsStage` for conservative
line-search limiting. It adds primitive CCD status propagation so iteration
exhaustion is treated as uncertainty, uses per-body reusable contact candidate
scratch, disables the no-edge/no-ground fast path when surface contact topology
exists, shortens accepted trials before Armijo evaluation, and rejects zero-step
initial separation-band hits. For volumetric bodies, the point-triangle CCD
candidate path is filtered to boundary-surface nodes so interior tetrahedral
nodes do not over-limit surface contact. It is still scaffolding:
inter-body contact, deformable-rigid mesh contact, barrier assembly, projected
Newton, and friction are not implemented yet.

The sweep-scratch reuse sub-slice adds reusable internal point/triangle/edge
sweep-item arrays for static and motion-aware contact candidate sweep builders.
The world-stage same-body surface-contact line search owns this scratch per
body, alongside the reusable candidate set. This removes per-trial sweep-vector
allocation from the CCD line-search path while preserving the existing
return-by-value and reusable-candidate overloads as wrappers. It is still
scaffolding: inter-body/deformable-rigid mesh contact, barrier assembly,
projected Newton, and friction are not implemented yet.

The sweep-pair traversal sub-slice optimizes internal cross-set sweep traversal
used by experimental surface candidate generation and inter-body surface CCD
checks. Sorted right-hand-side AABBs whose maximum x coordinate is already
behind the current left-hand-side minimum x are skipped as a prefix instead of
rescanned. This is not a full sweep-and-prune active set: a long early
right-hand-side interval can still keep later expired items in the scan. This
preserves visitor semantics and leaves same-set edge-edge self traversal, exact
distance filters, topology filters, and public APIs unchanged.

The inter-body surface-contact CCD sub-slice adds stage-start deformable
surface snapshots and a cross-surface point-triangle/edge-edge CCD limiter
inside `DeformableDynamicsStage`. It covers moving-point versus
stationary-triangle, moving-triangle versus stationary-point, moving-edge
versus stationary-edge, and insertion-order regressions. It is still
scaffolding: other bodies are stationary obstacles for the current sequential
line search, not a coupled multi-body Newton solve, and deformable-rigid
contact, barrier forces, projected Newton, and friction are not implemented yet.

The static-ground-barrier CCD sub-slice adds a point-mass node sweep limiter
inside `DeformableDynamicsStage` for existing explicitly opted-in static rigid
ground barriers. It uses the current analytic box/sphere top-height barrier
semantics, shortens line-search trials before objective evaluation, covers
fast vertical sweeps, finite-footprint fly-through, narrow offset footprints,
sphere top surfaces, ordinary-static opt-out, and fixed-node skip regressions,
and reports explicit experimental C++ stage stats. It is still scaffolding: it is not
deformable-rigid mesh contact, rigid collision response, side-face collision,
moving obstacle contact, IPC barrier-force assembly, projected Newton, or
friction.

The static rigid surface CCD sub-slice adds an explicit
`DeformableObstaclePolicy::surfaceObstacle` opt-in for static box collision
shapes to become stationary surface obstacles for the deformable CCD line-search
limiter. `DeformableDynamicsStage` collects stage-start box transforms into
world-space 12-triangle snapshots with 12 physical box edges, then reuses the
primitive point-triangle and edge-edge CCD reducer against point-only and
surfaced deformables. The dartpy `RigidBody.deformable_obstacle_policy` property
mirrors the C++ opt-in. It is still scaffolding: ordinary untagged boxes,
non-static bodies, spheres, arbitrary meshes, moving obstacles, rigid collision
response, barrier/contact forces, projected Newton, friction, and IPC parity are
not implemented yet.

The candidate-set sub-slice adds deterministic unique surface-edge extraction,
internal point-triangle and edge-edge primitive candidate assembly,
incident/adjacent filtering, exact activation-distance filtering through the
primitive distance kernels, sweep-versus-brute-force regression tests, and
`bm_ipc_candidate_set`. It is still scaffolding: candidates are not wired into
`World::step()`, conservative CCD, barrier assembly, projected Newton, or
friction.

The CCD step-bound sub-slice adds conservative internal point-triangle and
edge-edge normalized step bounds through native primitive CCD, initial
separation-band handling, deterministic minimum aggregation over assembled
candidate sets, exact-CCD regression checks where available, sampled safety
checks before the returned bound, and `bm_ipc_continuous_collision_step`. It is
still scaffolding: the bounds are not wired into `World::step()`, motion-aware
candidate culling, barrier assembly, projected Newton, or friction.

## Current Branch

`feature/ipc-deformable-matrix-free-cg` - published as PR #2821 against `main`.
The hosted PR head is still `fc9f11a153a`, while the local branch has the
addressed Codex review fix, local handoff updates, and a clean merge of current
`origin/main` (`5b3faf623b3`). Push only after explicit approval, then resolve
the addressed Codex thread and request a fresh Codex review if approved. If time
passes before the push, fetch `origin/main` again and merge it into the branch
before pushing.

The branch continues on top of `feature/ipc-deformable-cg-iteration-diagnostics`
after #2810/#2811/#2812/#2813 were all squash-merged. The four older local
`feature/ipc-deformable-*` branches are patch-equivalent to current
`origin/main` (`git cherry -v origin/main <branch>` reports `-` for each), and
their remote refs have been pruned from `origin`. The diagnostics branch remains
local-only and is part of the active PR #2821 stack. Do not delete local branch
refs without explicit approval.

The base diagnostics slice is behavior-preserving M7 profiling work. It extends
the public deformable solver diagnostics beyond `projectedNewtonIterativeSolves`
with
`projectedNewtonIterativeIterations`, `projectedNewtonIterativeMaxError`,
`projectedNewtonHessianNonZeros`, and
`projectedNewtonHessianStorageBytes` (dartpy:
`projected_newton_iterative_iterations` /
`projected_newton_iterative_max_error` /
`projected_newton_hessian_nonzeros` /
`projected_newton_hessian_storage_bytes`) so CG-backed Newton steps report solve
effort, residual estimates, and the assembled sparse-matrix footprint, not just
path selection. The FEM-bar and chunky 3D cube benchmarks emit matching
`cg_iters_per_step`, `cg_max_error`, `hessian_nonzeros`, and
`hessian_storage_bytes` counters toward the Fig. 23 / Table 1 profiling surface.
The current matrix-free slice adds explicit
`DeformableMaterialProperties.useMatrixFreeLinearSolver` /
`use_matrix_free_linear_solver`, reports
`projectedNewtonMatrixFreeSolves` /
`projected_newton_matrix_free_solves`, and adds benchmark rows with
`matrix_free_solves_per_step` plus zero sparse-Hessian footprint counters. It is
covered on static-ground contact by a C++ FEM cube regression that matches
direct sparse, sparse IC-CG, and matrix-free CG equilibria while proving the
matrix-free path keeps zero sparse-Hessian footprint, plus a dartpy single-node
ground-contact regression that matches the direct solve. The local review-fix
commit bumps the simulation binary format to v9 and gates material
deserialization so v8 files default `useMatrixFreeLinearSolver` to false instead
of consuming the next byte.

Prior branch `feature/ipc-gpu-psd-perf-gate` - stacked on
`feature/ipc-gpu-psd-backend-injection` (#2759). GPU-vs-CPU PERF GATE +
threshold tuning. New cuda test GpuVsCpuPerfGateAtSolverScale: projects 12x12
barrier batches across {256,1024,4096,16384}, asserts GPU==CPU parity at every
scale (hard gate), logs per-call wall time (informational, NOT asserted ->
non-flaky). MEASURED on RTX 5000 Ada: 256 blocks ~0.4x (GPU slower), 1024 ~1.4x,
4096 ~4x, 16384 ~9x -> crossover ~1k blocks. So raised the cuda adapter's
kMinGpuBatchBlocks 64 -> 1024 (data-driven; small batches stay on CPU). Only
touches the cuda test + the cuda adapter threshold -> CPU/default build
unaffected; test-all for lint; 8/8 cuda tests pass. NOTE Phase-5 contract forbids
new cuda-named BENCHMARK files -> this is a TEST. Stack 22-deep
(#2738-#2746,#2748-#2760).
NEXT remaining (all riskier/deeper, best after Codex reviews the base): resident
GPU device buffers (avoid per-call cudaMalloc), matrix-free CG, adaptive barrier
stiffness (TRAJECTORY-CHANGING, no public kappa knob), rigid/codim obstacle
barrier forces (DISTURBS #2732). Blocked: codim-obstacle friction, corpus port.

Prior #2759 = live GPU PSD-backend injection. LIVE GPU PSD-BACKEND INJECTION
(user chose "full live wiring now" via AskUserQuestion). New core seam (NO CUDA
dep): `compute/deformable_psd_backend.{hpp,cpp}` -- `projectSymmetricBlocksToPsd`
dispatches to an injectable backend (default `projectSymmetricBlocksToPsdCpu`,
per-block Eigen self-adjoint eigensolve + clamp, BIT-IDENTICAL to the old inline
projectSymmetricToPsd which I removed). world_step_stage assembly RESTRUCTURED:
the spring (6x6) and barrier (12x12, PT+EE) Hessian blocks are now COLLECTED into
a packed row-major buffer, batch-projected via the seam, then scattered (was
inline per-block). CPU path bit-identical -> 66 deformable tests + full test-all
6/6 (incl check-no-gpu-runtime-dependencies: core stays GPU-free). CUDA sidecar
(deformable_psd_projection_cuda.{cuh,cpp}) adds installCudaDeformablePsdBackend()/
restoreDefaultDeformablePsdBackend(): an adapter that offloads batches >=64 blocks
to projectSymmetricBlocksToPsdCuda and defers smaller/no-device to CPU (so the
offload never changes results). GPU validated on RTX 5000 via pixi -e cuda:
test BackendInjectionRoutesThroughCoreSeam (install -> core seam -> GPU matches
CPU <1e-9; restore -> CPU). 7/7 cuda tests pass. Stack 21-deep
(#2738-#2746,#2748-#2759).
GOTCHA: `pixi run -e cuda test-cuda`'s build step does NOT rebuild the deformable
cuda TEST target (only the lib/sidecar + rigid bench), so it ran a STALE binary;
had to `cmake --build build/cuda/cpp/Release --target test_deformable_psd_projection_cuda`
explicitly. Eigen ternary/RowMajor: packed blocks via Eigen::Map<...,RowMajor>
(symmetric so row/col-major identical).
NEXT remaining Phase 3: resident GPU solve path (current backend round-trips
host<->device per batch -> persistent device buffers + formal GPU-vs-CPU perf
gate), matrix-free CG, adaptive barrier stiffness, rigid/codim barrier forces
(DISTURBS #2732). Blocked: codim-obstacle friction, corpus port (assets).

Prior #2758 = converged-ness step-norm diagnostic
(DeformableSolverStats.finalStepInfinityNorm; diagnostic-only). Adds a CONVERGED-NESS
step-norm diagnostic: DeformableSolverStats.finalStepInfinityNorm = largest last
accepted per-node step (infinity norm) across the step's bodies, captured in the
line-search accept block (max|candidate-next| over free nodes, before the swap),
folded across bodies after the outer loop. BEHAVIOR-PRESERVING (diagnostic only;
solve unchanged). KEY FINDING while investigating the #2745 barrier-stall: the
320-node grid-on-ground UNIT scenario actually CONVERGES FULLY
(finalGradientResidualNorm ~3e-13 after 5 steps) -- the ~868/~99 high-residual
figures were BENCHMARK transients (bm_deformable_body), NOT this settled case.
So this slice is honestly a DIAGNOSTIC (the right convergence companion for
stiff barriers where the near-singular barrier Hessian inflates the gradient
norm), NOT a stall fix -- I did not reproduce a genuine stall in a unit test.
Regression StiffGroundBarrierSettlesByStepNorm (early step >0 while settling,
shrinks <1e-4 < early at equilibrium). 66 deformable tests, test-all expected
6/6. Stack 20-deep (#2738-#2746,#2748-#2758).
NEXT: if pursuing real barrier-stall robustness, reproduce the high residual via
the BENCHMARK scenarios first (bm_deformable_body grid/drape), then consider an
IPC-standard Newton-decrement convergence criterion or CCD-filtered line search
-- but those change trajectories/iteration counts (risk on the unreviewed stack).
Other remaining: live GPU-backend injection, adaptive stiffness, rigid/codim
barrier forces (disturbs #2732). Blocked: codim-obstacle friction, corpus port.

Prior #2757 = scene loader + diagnostics bindings (FINAL Phase-8 increment): dartpy
SCENE LOADER + DIAGNOSTICS. m.def free functions load_deformable_scene(world,
scene_path, options) + collect_deformable_scene_diagnostics(world), and classes
DeformableSceneLoadOptions (asset_root[path], body_name_prefix,
add_structural_springs, structural_spring_stiffness, damping,
ignore_contact_directives), DeformableSceneInfo (duration, time_step,
gravity_enabled, bodies, warnings), DeformableSceneBodyInfo (name, body, *counts),
DeformableSceneDiagnostics (frame, time, *counts, total_mass, max_displacement,
min_z, max_z). Needed `#include <nanobind/stl/filesystem.h>` for the path<->str
caster + the io header. io types are `sim::io::...` (sim=dart::simulation::
experimental). Python test test_experimental_deformable_scene_loader_python_api
writes a single-tet Gmsh msh + scene.txt into tmp_path, loads it, checks counts
(4 nodes, 1 tet, 4 derived surface tris) + total_mass==1 (vol 1/6 \* density 6).
Binary restart save/load (std::iostream) DEFERRED (needs a Python bytes API).
PHASE 8 NOW COMPLETE -> README Phase 8 marked [x]. Stack 19-deep
(#2738-#2746,#2748-#2757).
NEXT (riskier core items, now the main remaining work; sequence carefully on the
unreviewed stack): barrier-stall convergence robustness (the #2745 high-residual
finding), live GPU-backend injection, adaptive barrier stiffness, rigid/codim
obstacle barrier forces (disturbs #2732). Blocked: codim-obstacle friction,
upstream corpus port (assets).

Prior #2756 = DBC/NBC bindings (DeformableDirichletBoundaryCondition +
DeformableNeumannBoundaryCondition classes + options
dirichlet_boundary_conditions/neumann_boundary_conditions; DBC node follows
linear_velocity\*dt, NBC applies per-node acceleration; Python regression).

Prior #2755 = deformable surface/tetra topology bindings (DeformableSurfaceTriangle

- DeformableTetrahedron classes, options surface*triangles/tetrahedra, body
  accessors surface_triangle_count/surface_triangle, tetrahedron_count/tetrahedron/
  tetrahedron_rest_volume, node_mass; regen stubs; unit-tetra test). KEY dartpy
  facts: NO get*\_/set\_\_ names allowed;
  binding changes need `pixi run generate-stubs` AND
  `pixi run check-api-boundaries` (on-demand report via
  `pixi run report-api-boundary-inventory`).

Prior #2754 = friction diagnostics: DeformableSolverStats gains
`frictionDissipation` (sum mu*lambda*f1(y)*y over active friction contacts at the
converged iterate = force*slip, ramped to 0 at rest) + `activeFrictionContacts`
(count of ground+self-contact contacts w/ nonzero lagged normal force). New free
fn `accumulateFrictionDiagnostics` mirrors the two friction energies' slip
measures (ground u_T=(I-n n^T)(x-start); self-contact projection\*displacement),
called ONCE after the outer loop (not the line-search hot path). Both 0 when
mu==0. DeformableSolverStats is C++-ONLY (NOT bound to dartpy) -> no stub regen.
Regression FrictionDiagnosticsReportSlidingDissipation. With non-flat normals +
diagnostics done, Phase 4 is COMPLETE except codim-obstacle friction (BLOCKED on
the codim-obstacle barrier, a Phase-3 item).

Prior #2753 = non-flat ground-normal friction (static-ground
friction follows the TRUE GEOMETRIC ground normal instead of a hardcoded xy
tangent plane. KEY: the static-ground barrier is a vertical height field (force
along +z) and stays that way; only the FRICTION term gains a per-node lagged
normal. `boxContactAt`/`staticGroundContactAt` now return a `StaticGroundContact
{top, normal}` (radial normal for a sphere; the +z-exit-face normal of the
ray-march for a rotated/tilted box); `staticGroundTopAt` delegates to it so the
6 CCD callers are unchanged. `computeStaticGroundNormalForces` outputs a
parallel `normalDirection`; `GroundFrictionInputs` gains `laggedNormalDirection`;
`addGroundFrictionEnergy` and the Newton ground-friction Hessian project against
P = I - n n^T (energy/grad use u_T = (I - n n^T)(x - x_start); the Hessian is a
PSD 3x3 tangent block scale*[ (f1/y)(P - T T^T) + f1' T T^T ], -> scale*(2/eps)P
as y->0). For flat ground n = +z, P = diag(1,1,0) -> reduces EXACTLY to the old
xy 2x2 block, so flat-ground friction tests are bit-identical. Regression
GroundFrictionFollowsTiltedSlopeNormal: a node dropped straight down (-z) onto a
45-deg tilted box deflects DOWN-SLOPE (+x ~= 0.017) because tilt-aware friction
couples normal/tangential, whereas the frictionless control (and any xy-only
tangent model) stays on x = 0. 64 deformable tests pass, test-all 6/6.
CAVEAT/LESSON: the failure-then-fix flipped my predicted -x deflection to +x --
the vertical height-field barrier exerts no x-force, so all x-motion comes from
the tilted friction coupling; sign is +x (down-slope), validated empirically.
ALSO: `cmake --build ... 2>&1 | tail` masks ninja's exit code (pipe returns
tail's 0); redirect to a file and check $? separately. Eigen ternary needs both
branches wrapped in Eigen::Vector3d (lazy-expr vs BasisReturnType type mismatch).
NOTE: codimensional-obstacle friction (the next nominal Phase-4 item) is BLOCKED
on the codim-obstacle barrier (a remaining Phase-3 item) -- no barrier to lag
against -- so it is skipped; friction diagnostics is the next executable item.
friction to follow the TRUE GEOMETRIC ground normal instead of a hardcoded xy
tangent plane. KEY: the static-ground barrier is a vertical height field (force
along +z) and stays that way; only the FRICTION term gains a per-node lagged
normal. `boxContactAt`/`staticGroundContactAt` now return a `StaticGroundContact
{top, normal}` (radial normal for a sphere; the +z-exit-face normal of the
ray-march for a rotated/tilted box); `staticGroundTopAt` delegates to it so the
6 CCD callers are unchanged. `computeStaticGroundNormalForces` outputs a
parallel `normalDirection`; `GroundFrictionInputs` gains `laggedNormalDirection`;
`addGroundFrictionEnergy` and the Newton ground-friction Hessian project against
P = I - n n^T (energy/grad use u_T = (I - n n^T)(x - x_start); the Hessian is a
PSD 3x3 tangent block scale*[ (f1/y)(P - T T^T) + f1' T T^T ], -> scale*(2/eps)P
as y->0). For flat ground n = +z, P = diag(1,1,0) -> reduces EXACTLY to the old
xy 2x2 block, so flat-ground friction tests are bit-identical. Regression
GroundFrictionFollowsTiltedSlopeNormal: a node dropped straight down (-z) onto a
45-deg tilted box deflects DOWN-SLOPE (+x ~= 0.017) because tilt-aware friction
couples normal/tangential, whereas the frictionless control (and any xy-only
tangent model) stays on x = 0. 64 deformable tests pass, test-all 6/6.
CAVEAT/LESSON: the failure-then-fix flipped my predicted -x deflection to +x --
the vertical height-field barrier exerts no x-force, so all x-motion comes from
the tilted friction coupling; sign is +x (down-slope), validated empirically.
ALSO: `cmake --build ... 2>&1 | tail` masks ninja's exit code (pipe returns
tail's 0); redirect to a file and check $? separately. Eigen ternary needs both
branches wrapped in Eigen::Vector3d (lazy-expr vs BasisReturnType type mismatch).
NOTE: codimensional-obstacle friction (the next nominal Phase-4 item) is BLOCKED
on the codim-obstacle barrier (a remaining Phase-3 item) -- no barrier to lag
against -- so it is skipped; friction diagnostics is the next executable item.

Prior #2752 = edge-edge self-contact friction (force + Hessian; generic over the
4-node SelfContactFrictionContact, only buildSelfContactFrictionContacts extended
to edgeEdgeCandidates). #2751 = self-contact friction Hessian (PT). #2750 = Python facade (dartpy
deformable bindings; forbids get*\*/set*\* names; regenerate stubs; verify with
`pixi run check-api-boundaries`). #2749 = scene-replay harness.

Prior #2749 = scene-replay harness; #2748 = self-contact friction; #2746 = ground
friction. NOTE (from the #2745
diagnostic): the stiff barrier-contact benchmarks
(grid-on-ground, drape) settle feasibly/non-penetrating but do NOT converge to
the tight gradient tolerance (finalGradientResidualNorm ~99-868) -- a pre-existing
projected-Newton + line-search stall on very stiff barriers, present without
friction (mu=0 == mu=0.5 residual). Not a friction bug; a known IPC-class
limitation worth a future continuation/line-search-robustness slice.

Prior stacked branches, all open + awaiting Codex review (Codex usage-limited
until ~Sat 2AM; user is batching review): #2738 (moving rigid CCD) <- #2739
(self-contact barrier) <- #2740 (projected Newton, dense) <- #2741 (sparse
Cholesky) <- #2742 (drape demo) <- #2743 (GPU PSD primitive) <- #2744 (symbolic
reuse) <- #2745 (convergence diagnostic) <- #2746 (ground friction) <- #2748
(self-contact friction) <- #2749 (scene-replay harness) <- #2750 (Python facade)
<- #2751 (self-contact friction Hessian) <- #2752 (edge-edge self-contact
friction) <- #2753 (non-flat ground-normal friction) <- #2754 (friction
diagnostics) <- #2755 (deformable surface/tetra topology bindings) <- #2756
(deformable DBC/NBC bindings) <- #2757 (deformable scene-loader + diagnostics
bindings) <- #2758 (converged-ness step-norm diagnostic) <- #2759 (live GPU
PSD-backend injection). (PR #2747 is another author's.) <- #2760 (GPU-vs-CPU perf gate + adapter threshold tuning).

## Immediate Next Step

**Historical (superseded):** PR #2821 (matrix-free CG + CG diagnostics) merged
long ago (`74338577982`). The instructions below it about pushing that branch are
stale — ignore them.

**Current (2026-07-04):** PR #3257 (peak-contacts diagnostic) **merged** to
`main` (`1819b801228`). The follow-on Fig-23 **statistics packet** slice is
staged on `feature/ipc-deformable-fig23-statistics-packet` (see the "Active
Slice" block); verify (`pixi run check-lint-py`, the packet pytest, an
end-to-end writer run), then push + PR after approval; never reply inline to a
`[bot]` comment.

After that, continue M7 in bounded performance slices: harden matrix-free CG on
larger contact-heavy meshes and decide the automatic large-mesh selection
policy; build out the rest of the Fig-23 statistics harness (avg contacts/step
aggregation, peak-memory tracking, then the Table-1 CPU reference comparison);
then AMG / multigrid preconditioning, on-device GPU assembly + solve, and the
688K-node Fig. 22 scale run. Codimensional-obstacle friction remains blocked on
codimensional-obstacle barrier support. Dev-task retirement stays maintainer-
gated while PLAN-081 is incomplete.

## Context That Would Be Lost

- The upstream IPC audit is pinned to commit
  `573d2c7e04104d3f9baf526bdaee7745891a571a`.
- Use `git ls-tree` to enumerate scene paths; `find -type f` misses the 10 SQP
  benchmark symlink aliases.
- The durable row-level source of truth is
  `docs/plans/081-deformable-implicit-barrier-solver/ipc_scene_corpus_manifest.json`.
- The validator must keep zero unclassified family or target-type rows before
  any future IPC parity claim.

## How To Resume

Current slice (2026-07-04) — Fig-23 statistics packet gates (Python-only, no C++ rebuild):

```bash
git checkout feature/ipc-deformable-fig23-statistics-packet
git status && git log -3 --oneline
pixi run python -m pytest tests/test_plan081_deformable_fig23_packet.py -q
pixi run check-lint-py
# End-to-end against a real benchmark run (proves the manifest matches real counters):
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_format=json --benchmark_min_time=0.01s \
  --benchmark_filter='(BM_DeformableFemBarStep/24|BM_DeformableFcrBarStep/24|BM_DeformableCgBarStep/24|BM_DeformableMatrixFreeCgBarStep/24|BM_DeformableCube3dDirectStep/8|BM_DeformableCube3dCgStep/8|BM_DeformableSelfContactBarrierStage/8)$' > /tmp/def.json
pixi run python scripts/write_plan081_deformable_fig23_packet.py --benchmark-json /tmp/def.json
pixi run lint
```

Prior merged slice (#3257) — Fig-23 peak-contacts diagnostic gates (archived):

```bash
# git checkout feature/ipc-deformable-max-contacts-diagnostic   # merged as 1819b801228
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.SelfContactBarrier*'
./build/default/cpp/Release/bin/test_world --gtest_filter='World.DeformableSelfContactFriction*'
```

Historical (#2821 handoff — superseded, run only for archaeology on that slice):

```bash
git checkout feature/ipc-deformable-matrix-free-cg
git status && git log -3 --oneline
git fetch origin main
git merge --no-ff origin/main
cmake --build build/default/cpp/Release --target test_deformable_body dartpy
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.SerializationLoadsLegacyV8Material:DeformableBody.SerializationPreservesMeshTopologyAndMaterial:DeformableBody.MatrixFreeLinearSolverMatchesSparseSolversOnGroundContact:DeformableBody.MatrixFreeLinearSolverMatchesSparseDirectSolve'
PYTHONPATH=build/default/cpp/Release/python ./.pixi/envs/default/bin/python -m pytest python/tests/unit/simulation/test_experimental_world.py -k 'matrix_free_solver'
pixi run check-api-boundaries
pixi run lint

# Stop here for the current PR #2821 handoff. The remaining commands are
# historical validation references for older IPC slices; run them only when
# editing those older areas.
cmake --build build/default/cpp/Release --target test_primitive_distance bm_ipc_distance_kernels
ctest --test-dir build/default/cpp/Release -R '^test_primitive_distance$' --output-on-failure
./build/default/cpp/Release/bin/bm_ipc_distance_kernels --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
cmake --build build/default/cpp/Release --target test_contact_candidate_set bm_ipc_candidate_set
./build/default/cpp/Release/bin/test_contact_candidate_set
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcCandidateSet'
cmake --build build/default/cpp/Release --target test_continuous_collision_step bm_ipc_continuous_collision_step
./build/default/cpp/Release/bin/test_continuous_collision_step
./build/default/cpp/Release/bin/bm_ipc_continuous_collision_step --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
cmake --build build/default/cpp/Release --target test_barrier_kernel bm_ipc_barrier_kernel
./build/default/cpp/Release/bin/test_barrier_kernel
./build/default/cpp/Release/bin/bm_ipc_barrier_kernel --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
cmake --build build/default/cpp/Release --target test_tangent_stencil bm_ipc_tangent_stencil
./build/default/cpp/Release/bin/test_tangent_stencil
./build/default/cpp/Release/bin/bm_ipc_tangent_stencil --benchmark_min_time=0.05s --benchmark_filter='BM_Ipc'
cmake --build build/default/cpp/Release --target test_contact_candidate_set test_continuous_collision_step bm_ipc_motion_aware_candidate_set
./build/default/cpp/Release/bin/test_contact_candidate_set
ctest --test-dir build/default/cpp/Release -R '^(test_contact_candidate_set|test_continuous_collision_step)$' --output-on-failure
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcMotionAwareCandidateSet'
cmake --build build/default/cpp/Release --target test_contact_candidate_set test_continuous_collision_step bm_ipc_candidate_set bm_ipc_motion_aware_candidate_set
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcCandidateSet'
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.05s --benchmark_filter='BM_IpcMotionAwareCandidateSet'
cmake --build build/default/cpp/Release --target test_primitive_ccd test_continuous_collision_step test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_primitive_ccd --gtest_filter='PointTriangleCcd.*'
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.SurfaceContactCcd*:DeformableBody.SurfaceFreeParticlesKeepFastPath:DeformableBody.SurfaceContactScratchIsPerBody:DeformableBody.StageMetadataUsesDeformableDomain'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableSurfaceContactStage'
cmake --build build/default/cpp/Release --target test_contact_candidate_set bm_ipc_candidate_set bm_ipc_motion_aware_candidate_set
./build/default/cpp/Release/bin/test_contact_candidate_set --gtest_filter='IpcContactCandidateSet.*Reusable*:IpcContactCandidateSet.*SweepScratch*'
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.03s --benchmark_filter='BM_IpcCandidateSet(Reusable|Scratch)?Sweep'
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.03s --benchmark_filter='BM_IpcMotionAwareCandidateSet(Reusable|Scratch)?Sweep'
cmake --build build/default/cpp/Release --target test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.InterBodySurfaceContactCcd*:DeformableBody.SurfaceContactCcd*'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableInterBodySurfaceContactStage'
cmake --build build/default/cpp/Release --target test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.StaticGroundBarrier*:DeformableBody.ActiveStaticGroundContactAllowsTangentialMotion:DeformableBody.ActiveDirichletNodesDoNotBlockGroundBarrierSolve:DeformableBody.StaticCollisionRequiresGroundBarrierOptIn'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableStaticGroundBarrierCcdStage'
cmake --build build/default/cpp/Release --target test_contact_candidate_set test_continuous_collision_step test_deformable_body bm_ipc_candidate_set bm_ipc_motion_aware_candidate_set bm_deformable_body
./build/default/cpp/Release/bin/test_contact_candidate_set --gtest_filter='IpcContactCandidateSet.VisitSweepPairsMatchesNaiveReference:IpcContactCandidateSet.*Sweep*:IpcContactCandidateSet.*Reusable*:IpcContactCandidateSet.MotionAware*'
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.SurfaceContactCcd*:DeformableBody.InterBodySurfaceContactCcd*:DeformableBody.SurfaceFreeParticlesKeepFastPath:DeformableBody.SurfaceContactScratchIsPerBody:DeformableBody.StageMetadataUsesDeformableDomain'
./build/default/cpp/Release/bin/bm_ipc_candidate_set --benchmark_min_time=0.03s --benchmark_filter='BM_IpcCandidateSet(CrossSweepExpiredPrefix|(Scratch|Reusable)?Sweep)'
./build/default/cpp/Release/bin/bm_ipc_motion_aware_candidate_set --benchmark_min_time=0.03s --benchmark_filter='BM_IpcMotionAwareCandidateSet(Scratch|Reusable)?Sweep'
cmake --build build/default/cpp/Release --target test_deformable_body test_serialization bm_deformable_body dartpy
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.StaticRigidSurfaceCcd*:DeformableBody.SurfaceFreeParticlesKeepFastPath:DeformableBody.StaticGroundBarrier*:DeformableBody.InterBodySurfaceContactCcd*'
./build/default/cpp/Release/bin/test_serialization --gtest_filter='Serialization.PreservesRigidBodyCollisionComponents'
PYTHONPATH=build/default/cpp/Release/python ./.pixi/envs/default/bin/python -m pytest python/tests/unit/simulation/test_experimental_world.py -k 'collision_query'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableStaticRigidSurfaceCcdStage'
cmake --build build/default/cpp/Release --target test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.MovingRigidSurfaceCcd*:DeformableBody.MovingAndStaticRigidSurfaceCcdCollectorsAreDisjoint'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableMovingRigidSurfaceCcdStage'
cmake --build build/default/cpp/Release --target test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.SelfContactBarrier*'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.03s --benchmark_filter='BM_DeformableSelfContactBarrierStage'
cmake --build build/default/cpp/Release --target test_deformable_body bm_deformable_body
./build/default/cpp/Release/bin/test_deformable_body --gtest_filter='DeformableBody.*ProjectedNewton*:DeformableBody.SparseProjectedNewtonScalesBeyondDenseCap:DeformableBody.FixedSpringMatchesAnalyticImplicitEulerStep'
./build/default/cpp/Release/bin/bm_deformable_body --benchmark_min_time=0.02s --benchmark_filter='BM_DeformableGridStage'
```

Historical note: use older `feature/ipc-*` branches only for archaeology or
targeted regression work on their slice; the current handoff branch is
`feature/ipc-deformable-matrix-free-cg`.
