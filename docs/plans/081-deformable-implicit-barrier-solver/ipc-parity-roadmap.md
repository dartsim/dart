# IPC Paper-Parity Roadmap

Authoritative execution roadmap for taking DART's experimental deformable
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
- **153/154 corpus scenes need FEM** *and* upstream meshes; 0 meshes are
  vendored and there is no `.msh`/`.obj`/`.seg`/`.pt` importer. So essentially
  no paper scene is reproducible today, and **every showcase figure row is
  still `planned`.**

## Critical path (capability-ordered)

The milestones are ordered by dependency. Each ships as one PR off `main`
(milestone DART 7.0) with **kernel + correctness tests + a runnable example +
a benchmark**, per the standing slice contract.

### M1 — FEM tetrahedral elasticity (the keystone) — IN PROGRESS
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
- Unblocks: every volumetric figure becomes *physically* possible (Figs 4, 12,
  13, 14, 15, 22, Table 1), pending obstacles/friction/assets per below.

### M2 — Obstacle barrier completeness (analytic + box) + codim wiring
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

### M4 — Upstream asset pipeline + `.msh` (GMSH) importer
A fetch-into-fixtures workflow for the upstream meshes pinned at
`573d2c7e0…` (no vendoring, no runtime dependency), plus a GMSH `.msh`
tet-mesh importer in `dart/io`. Port the contact-free tutorial scenes
(`2cubesFall` DBC/NBC) first as regression fixtures, then the contact scenes.

### M5 — Mesh-vs-obstacle friction completeness
Extend lagged smoothed Coulomb friction beyond the static-ground coefficient to
mesh-vs-obstacle and codim contacts. Unblocks the friction figures: Fig 5 (card
house), Fig 7 (cement arch), Fig 8 (ball-on-roller), Fig 16 (Armadillo roller),
Fig 20 (stick-slip rod).

### M6 — Adaptive barrier stiffness (κ homotopy)
IPC stiffness adaptation with a minimum-stiffness scale and force-balance
update; additive (defaults to today's fixed κ). Unblocks Fig 12 (large
mass/stiffness ratios) robustness.

### M7 — Scale + performance (CPU then GPU) until beating the reference
Matrix-free / AMGCL-class iterative linear solve for very large systems (Fig
22, 688K nodes), on-device GPU assembly + solve beyond the current PSD offload,
and a profiling-grade benchmark harness that emits Fig-23-shaped statistics
(avg/max contacts per step, Newton iterations per step, peak memory, s/step)
per scene. Acceptance: match or undercut IPC's per-step CPU time at equal
accuracy on shared scenes (Table 1, Fig 23), then report GPU acceleration as
net-new.

## Honest status

This is a multi-PR program. No paper figure is reproducible until at least M1
(FEM) lands, and the contact-heavy figures additionally need M2/M3/M5 plus the
M4 assets. The mass-spring showcases already in py-demos (`IPC Deformable (sx)`)
evoke the paper's *themes* but are explicitly **not** figure reproductions and
do not count toward parity. Progress is tracked by promoting figure rows in
[`ipc-paper-figure-showcase.md`](ipc-paper-figure-showcase.md) from `planned`
→ `in-progress` → `landed` → `reference-beaten`.
