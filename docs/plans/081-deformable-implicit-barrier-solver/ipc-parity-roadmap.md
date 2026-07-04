# IPC Paper-Parity Roadmap

Authoritative execution roadmap for taking the DART 7 deformable
solver from its current mass-spring contact scaffold to faithful reproduction
of the IPC paper (Li et al. 2020) experiments, with correctness tests and
performance benchmarks that match or beat the reference, on **both CPU and
GPU**. Companion to [`ipc-paper-figure-showcase.md`](ipc-paper-figure-showcase.md)
(figure-by-figure targets), [`ipc-paper-gap-audit.md`](ipc-paper-gap-audit.md)
(Algorithm 1 / kernel matrix), and
[`ipc_scene_corpus_manifest.json`](ipc_scene_corpus_manifest.json) (154 upstream
scenes).

## Ground truth (audited from the paper + current code, 2026-05-30)

- **Every IPC simulation scene is FEM-tetrahedral.** Figs 1, 2, 4, 5, 7, 8,
  13–22 and the Fig 10/11/12 unit tests are all volumetric tetrahedral FE
  bodies with a **neo-Hookean** (default) or **fixed-corotational (FCR)**
  material, stepped with implicit Euler (implicit Newmark only for the
  high-speed Fig 19 / golf-ball). Figs 3/6/9 are method diagrams, not scenes.
- **The original IPC is CPU-only — the paper reports no GPU numbers anywhere.**
  Timings come from multi-threaded CPU machines (TBB assembly, CHOLMOD direct
  solve, AMGCL only for the 688K-node Fig 22). The CPU baselines to beat are
  **Fig 23** (16-scene per-step statistics) and **Table 1** (h-sweep on the
  twisted-rods scene). Consequently the GPU story is **net-new acceleration**:
  there is no published GPU baseline to match — the win is (a) match/undercut
  IPC's per-step CPU time at equal accuracy, then (b) add GPU as a bonus.
- **DART today is mass-spring, not FEM.** Elastic energy is per-edge
  `0.5·k·(L−L0)²`; tets are inert scaffolding (lumped mass + surface
  extraction only). `DeformableMaterial.youngsModulus/poissonRatio` exist but
  are unused. On top of the spring net, real IPC machinery is landed: C2
  clamped-log barrier (self-contact PT+EE, ground, sphere obstacle), sparse
  projected Newton with batched PSD projection (+ optional CUDA backend),
  lagged smoothed Coulomb friction (ground + self-contact), conservative
  PT/EE CCD limiters, and active-set sweep-and-prune broad phase.
- **153/154 corpus scenes need FEM** _and_ upstream meshes; 0 meshes are
  vendored and there is no `.msh`/`.obj`/`.seg`/`.pt` importer. So essentially
  no paper scene is reproducible today, and **every showcase figure row is
  still `planned`.**

## Critical path (capability-ordered)

The milestones are ordered by dependency. Each ships as one PR off `main`
(milestone DART 7.0) with **kernel + correctness tests + a runnable example +
a benchmark**, per the standing slice contract.

### M1 — FEM tetrahedral elasticity (the keystone) — LANDED

Status: the stable neo-Hookean kernel
(`detail/deformable_elasticity/fem_tet_element.hpp`) is implemented and
finite-difference-validated; the solver wires it in as an opt-in elasticity
model (`DeformableMaterialProperties.useFiniteElementElasticity`, also exposed to
`dartpy`), assembling each tet's PSD-projected 12x12 Hessian through the existing
batched seam. Ships with solver regressions, a `BM_DeformableFemBarStep`
benchmark, and a `Deformable FEM Bar (IPC)` py-demos cantilever. The mass-spring
path is byte-identical when the flag is off. Each tet's rest shape is cached in
the per-entity scratch (computed once, reused every step).

Follow-ups within M1 — **FCR LANDED:** the fixed-corotational material variant
(`psi = mu||F - R||^2 + (lambda/2)(J-1)^2`, exact first Piola + positive-definite
Gauss-Newton element Hessian, with `R` from the polar decomposition) is wired in
as a second opt-in material behind
`DeformableMaterialProperties.useFixedCorotationalElasticity` (dartpy
`use_fixed_corotational_elasticity`), dispatched through a shared per-element seam
so neo-Hookean stays the default. It ships kernel
FD/rotation-invariance tests, solver regressions (FCR tet stationary at rest;
restores a perturbed node toward rest), and a `Deformable FCR Twist (IPC)`
py-demos showcase toward Fig. 4 / Fig. 14.

The exact analytic FCR Hessian has since **LANDED** in place of the Gauss-Newton
approximation: `2*mu*(I9 - dR/dF) + lambda*(g*g^T + (J-1)*d^2J/dF^2)`, with the
polar-rotation gradient `dR/dF` solved from the corotational identity
`(tr(S)I - S) w = axl(R^T dF - dF^T R)` (FD-validated; indefinite, so the solver
PSD-projects it; inverted elements fall back to Gauss-Newton). The exact Newton
curvature cut the `BM_DeformableFcrBarStep` per-step time ~7x, to near
stable-neo-Hookean parity at equal mesh resolution. M1 FEM (both materials, exact
Newton curvature, benchmarked) is now fully realized.

FEM **self-contact** is also now demonstrated end-to-end: a `Deformable FEM
Self-Contact (IPC)` showcase compresses a slender FEM beam end-to-end until it
buckles and folds onto itself, and the always-on clamped-log self-contact barrier
holds the folding surface at a strictly positive separation (verified via the new
`World` solver diagnostics: the barrier activates, every active contact stays at
a positive distance, and the solve stays finite). This couples the FEM keystone
with the IPC self-collision machinery on a volumetric body.

Add a per-tetrahedron strain-energy term producing per-element energy,
12-vector gradient, and 12×12 Hessian, reusing the existing PSD-projection
batch seam and sparse projected-Newton assembly. Material from the existing
`youngsModulus`/`poissonRatio` (→ Lamé μ, λ). First model: **stable
neo-Hookean** (Smith et al. 2018) — inversion-safe (finite energy for all F,
including J ≤ 0), C∞, and the standard in modern IPC toolkits; FCR is a
follow-up material variant. Rest data (`restPositions`,
`tetrahedronRestVolumes`) already exists.

- Tests: finite-difference gradient/Hessian consistency; rest-state zero
  energy + zero force; monotone energy under stretch; inversion-robustness
  (finite energy + force for an inverted element); determinism.
- Example: a twisting / squashing FEM bar in py-demos (toward Fig 4 / Fig 14).
- Benchmark: per-step FEM assembly + solve cost at a few mesh sizes.
- Unblocks: every volumetric figure becomes _physically_ possible (Figs 4, 12,
  13, 14, 15, 22, Table 1), pending obstacles/friction/assets per below.

### M2 — Obstacle barrier completeness (analytic + box) + codim wiring

Progress: the **sphere obstacle barrier Hessian** and a full **box obstacle
barrier** (energy + gradient + projected-Newton Hessian, via local-frame clamp
for face/edge/corner) have landed, so sphere- and box-obstacle contact are now
first-class Newton terms; FEM slabs draping over a sphere and over a box are
shown in py-demos. Remaining M2:

Generalize obstacle contact into a real clamped-log **force** everywhere: an
analytic half-space (plane) collision object, a box-obstacle barrier force
(reuse PT distance kernels against the box's triangulated faces), the
projected-Newton Hessian for the sphere/box obstacle barriers, and wire the
already-implemented point-edge / point-point distance kernels into the active
set so codimensional contact has barrier coverage. Unblocks the plane-drop
unit families and is a prerequisite for codim objects.

### M3 — Codimensional collision objects (triangle / edge / vertex)

Static (then moving) codimensional obstacles with barrier + CCD, plus the
`.obj` / `.seg` / `.pt` importers. Unblocks Fig 2 (knives, points), Fig 17
(pin-cushions), Fig 18 (codim rollers), and the 9-scene codim-unit family.

Progress: the **codimensional importer set is complete** — `.obj` (triangles →
cloth, `io::loadObjTriangleMesh*`), `.seg` (segments → strand,
`io::loadSegLineMesh*`), and `.pt` (points → particles, `io::loadPointSet*`), all
exposed to dartpy (`load_obj_triangle_mesh` / `load_seg_line_mesh` /
`load_point_set`) with `build_cloth_from_obj` / `build_strand_from_seg` /
`build_particles_from_pt` helpers and draped-cloth / hanging-strand /
falling-particles showcases.

The first codimensional collision **object** has also LANDED: a `Capsule`
(rod/wire) collision shape + a static **capsule obstacle barrier**
(`CollisionShape::makeCapsule`, dartpy `CollisionShape.capsule`). A static capsule
opted in as a deformable obstacle exerts the same clamped-log barrier (energy +
gradient + rank-1 radial Hessian) as the sphere/box obstacles, with distance +
normal from the analytic point-to-segment closest point; it is barrier-only (no
surface CCD), so a connected sheet drapes over it freely. Ships a radial-repulsion
regression, an intersection-free draped-cloth regression, and a `Deformable Cloth
over Capsule Rod (IPC)` showcase (toward Fig. 18 codimensional rollers). Remaining
M3: the point-cloud / single-triangle codim obstacles (the point obstacle is the
sphere with radius -> 0; a thin box approximates a single triangle today), and a
true codim-vs-codim CCD if needed for fast codim rollers.

### M4 — Upstream asset pipeline + `.msh` (GMSH) importer

Progress: the **GMSH `.msh` tetrahedral-mesh importer** has landed
(`io::loadGmshTetMesh*`, ASCII format 2.x; exposed to dartpy as
`load_gmsh_tet_mesh`), with a py-demos scene loading a bundled tet-bar mesh into
an FEM cantilever. Remaining M4:

A fetch-into-fixtures workflow for the upstream meshes pinned at
`573d2c7e0…` (no vendoring, no runtime dependency), plus a GMSH `.msh`
tet-mesh importer in `dart/io`. Port the contact-free tutorial scenes
(`2cubesFall` DBC/NBC) first as regression fixtures, then the contact scenes.

### M5 — Mesh-vs-obstacle friction completeness — capsule LANDED

Extend lagged smoothed Coulomb friction beyond the static-ground coefficient to
mesh-vs-obstacle and codim contacts. Unblocks the friction figures: Fig 5 (card
house), Fig 7 (cement arch), Fig 8 (ball-on-roller), Fig 16 (Armadillo roller),
Fig 20 (stick-slip rod).

Progress: **capsule (rod) obstacle friction has landed.** A node contacting a
static capsule feeds the capsule barrier's radial normal force/direction into the
existing lagged Coulomb friction term (dominant per-node contact wins), so a
deformable sliding along a rod decelerates with `frictionCoefficient`. This works
because the capsule obstacle is barrier-only (no over-limiting surface CCD), so
the tangential slide is unconstrained — directly demonstrating mesh-vs-obstacle
friction (toward Fig 8 / Fig 20 roller themes). Ships C++ + dartpy regressions and
a `Deformable Friction on Capsule Rod (IPC)` showcase.

**Sphere/box obstacle friction has also LANDED**, via an opt-in **barrier-only
obstacle** mode (`RigidBody::setDeformableObstaclePolicy` with
`DeformableObstaclePolicy::barrierOnly`, dartpy
`RigidBody.deformable_obstacle_policy.barrier_only`): the obstacle keeps its
clamped-log barrier but is `entt::exclude`-d from the surface-CCD collect, so the
deformable slides tangentially (the barrier alone prevents penetration for the
quasi-static contact) and the sphere/box obstacle normal forces feed the lagged
Coulomb friction term. A strip shoved across a barrier-only box plate is
decelerated by friction
(`Deformable Friction on Box Plate (IPC)` showcase + C++/dartpy regressions);
existing CCD-obstacle scenes are unchanged. **Remaining M5 (lower priority):** the
"right" fix that keeps the conservative CCD _and_ allows tangential sliding —
making the surface CCD limit only the _normal_ approach component (not the whole
step) — so friction works without giving up the fast-motion CCD safety net. That
is a focused, higher-risk change to the CCD contracts (cf. #2732); the barrier-only
mode is the safe, shipped path for quasi-static obstacle-friction scenes.

### M6 — Adaptive barrier stiffness (κ homotopy) — LANDED

Opt-in adaptive barrier stiffness has landed
(`DeformableMaterialProperties.useAdaptiveBarrierStiffness`, dartpy
`use_adaptive_barrier_stiffness`). Per step, the ground/obstacle barrier κ is set
from the mass/time-step force balance `κ = clamp((maxNodalMass/dt²)·d_hat², 25,
1e6)` — the barrier curvature `~κ/d_hat²` balances the inertial stiffness
`mass/dt²`, keeping the contact equally conditioned across mass/stiffness ratios
(toward Fig 12). For unit mass at dt=1/250, d_hat=2e-2 this is exactly the
historical fixed κ=25, so the adaptive form generalizes the constant; it is
floored at the default (never softer) and capped. Off (the default) is
byte-identical. A regression checks a heavy node settles measurably higher (the
stiffer adaptive barrier balances gravity farther from the surface,
mass-independently) than with fixed κ. Remaining M6 follow-up: in-Newton κ
homotopy (increase κ mid-solve if the min distance drops), and applying the
adaptive κ to the self-contact barrier (which today carries its own stiffness).

### M7 — Scale + performance (CPU then GPU) until beating the reference — iterative CG solve LANDED

Matrix-free / AMGCL-class iterative linear solve for very large systems (Fig
22, 688K nodes), on-device GPU assembly + solve beyond the current PSD offload,
and a profiling-grade benchmark harness that emits Fig-23-shaped statistics
(avg/max contacts per step, Newton iterations per step, peak memory, s/step)
per scene. Acceptance: match or undercut IPC's per-step CPU time at equal
accuracy on shared scenes (Table 1, Fig 23), then report GPU acceleration as
net-new.

**First increment landed:** an opt-in iterative conjugate-gradient
projected-Newton linear solve
(`DeformableMaterialProperties.useIterativeLinearSolver` / dartpy
`use_iterative_linear_solver`). It reuses the existing sparse SPD Hessian
assembly but solves with CG instead of `SimplicialLDLT`, so it never factorizes
— memory stays near O(nnz) and per-step cost grows more gently than direct
fill-in as the mesh chunks up. The inertia floor + PSD-projected element blocks
guarantee convergence; a non-converged/non-finite solve falls back to
steepest descent exactly as the direct path does on an indefinite
factorization. Systems above the retained dense-direct cap
(`kProjectedNewtonDenseDirectDofCap` = 128 DoF, ~42 nodes) now take the iterative
path automatically (effective ceiling 1M nodes) instead of degrading to gradient
descent. (Current architecture note: the built-in DART 7 World step keeps a dense
LDLT below that cap and sparse Jacobi-preconditioned CG above it; the Eigen
sparse-direct `SimplicialLDLT` factorization is intentionally kept out of the
allocation-safe simulation loop. The historical 20k figure was the earlier
sparse-direct node cap, since superseded.) Verified by a regression in which a
dropped FEM
cube settles identically under both solvers while taking mutually exclusive
solve paths (CG run never factorizes), with a `cg_solver` py-demo and a
`BM_DeformableCgBarStep` benchmark mirroring the direct FEM-bar benchmark for
per-step scaling comparison.

**Second increment landed:** the CG preconditioner was strengthened from
diagonal (Jacobi) to **incomplete-Cholesky**. Stiff barrier contact makes the
Hessian ill-conditioned, where Jacobi-CG stalls against its iteration cap and
falls back to steepest descent; the incomplete-Cholesky preconditioner (a sparse
approximate factorization that drops fill, so the solve still stays near O(nnz))
collapses the CG iteration count so the iterative path carries stiff contact
within the cap. On a settling stiff FEM contact scene this cuts the iterative
solver's fallbacks below the direct solver's and reduces Newton iterations per
step. Verified by a regression in which a stiff (E = 1e6) FEM cube settles on a
ground barrier through the iterative path with accepted CG steps dominating
fallbacks and the same equilibrium as the direct solve, plus a `cg_contact`
py-demo.

**Third increment landed:** a chunky 3D FEM-cube direct-vs-iterative scaling
benchmark (`BM_DeformableCube3dDirectStep` / `BM_DeformableCube3dCgStep`) that
makes the crossover measurable on the mesh shape where it bites. On a solid N^3
cube the direct solve's 3D fill-in makes its per-step time climb super-linearly
while the incomplete-Cholesky CG stays near O(nnz); on the development machine
the per-step cost crosses over near ~1k nodes and the iterative solve runs ~5x
faster by ~4k nodes (measured: 11.3 s/step direct vs 2.3 s/step iterative at
4096 nodes). The thin `BM_DeformableFemBarStep` beam (2x2 cross-section) cannot
show this -- its bandwidth is too small -- which is why the chunky fixture was
added. Guarded by a regression that the chunky 3D cube sags to the same
equilibrium under both solvers. This is the first concrete, measured rung of the
Fig-23 / Table-1 per-step-scaling axis.

**Diagnostic follow-up in progress:** the iterative solver's public diagnostics
now extend beyond the CG solve count with total CG iterations, the maximum
Eigen-reported residual estimate for successful iterative Newton linear solves,
and the assembled compressed sparse Hessian's nonzero count and storage estimate.
The FEM-bar and chunky 3D cube benchmarks report the same `cg_iters_per_step`,
`cg_max_error`, `hessian_nonzeros`, and `hessian_storage_bytes` counters.

**Matrix-free follow-up in progress:** an explicit
`useMatrixFreeLinearSolver` path applies local Hessian blocks directly with a
block-Jacobi preconditioner and reports `projectedNewtonMatrixFreeSolves`.
Benchmark rows expose `matrix_free_solves_per_step` plus zero sparse-Hessian
footprint counters. It is now covered by ground-contact parity regressions: C++
compares direct sparse, sparse IC-CG, and matrix-free CG on the same contacting
FEM cube, while dartpy compares direct and matrix-free contact settling.
Remaining hardening: larger contact-heavy meshes and deciding when matrix-free
CG becomes the automatic very-large-mesh path.

**Fig-23 statistics harness — shape-parity scaffold landed:** the deformable
solver diagnostics now include the peak active-contact count per step
(`maxActiveContactCount`, the Fig-23 max-contacts axis), and a machine-checkable
statistics packet distils the `bm_deformable_body` JSON into the Fig-23-shaped
per-scene axes (per-step Newton/CG effort, CG residual, assembled sparse-Hessian
footprint, per-step wall time, and the active-contact statistics) over the
DART-runnable scenes: `scripts/write_plan081_deformable_fig23_packet.py` +
[`fig23_deformable_statistics_corpus.json`](fig23_deformable_statistics_corpus.json),
validated by `python/tests/unit/test_write_plan081_deformable_fig23_packet.py`.
It is explicitly
`paper_scale: false` (shape parity, not paper parity).

Remaining M7 work: AMG / multigrid preconditioning for the largest systems,
on-device GPU assembly + solve beyond the current PSD offload, the 688K-node
Fig-22 scale run, and the **paper-scale** Fig-23 statistics plus the Table-1 CPU
comparison against the published IPC reference numbers (both blocked on the M4
upstream asset pipeline).

## Honest status

This is a multi-PR program. No paper figure is reproducible until at least M1
(FEM) lands, and the contact-heavy figures additionally need M2/M3/M5 plus the
M4 assets. The mass-spring showcases already in py-demos (`IPC Deformable (sx)`)
evoke the paper's _themes_ but are explicitly **not** figure reproductions and
do not count toward parity. Progress is tracked by promoting figure rows in
[`ipc-paper-figure-showcase.md`](ipc-paper-figure-showcase.md) from `planned`
→ `in-progress` → `landed` → `reference-beaten`.
