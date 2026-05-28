# VBD Paper / Reference Gap Audit

This audit grounds PLAN-082 against the Vertex Block Descent paper and its
reference implementations. It records what the method requires, what the DART
experimental world already provides, and what each phase must add. It is a
planning artifact; it does not itself claim implemented behavior.

## Sources

- Chen, A. H., Liu, T., Yang, Y., Kim, T., et al. "Vertex Block Descent."
  _ACM Transactions on Graphics (SIGGRAPH 2024)_.
  Paper PDF: <https://graphics.cs.utah.edu/research/projects/vbd/vbd-siggraph2024.pdf>
  Project page: <https://ankachan.github.io/Projects/VertexBlockDescent/index.html>
  Talk: <https://www.youtube.com/watch?v=2HCgKfKy3W8>
- Reference implementations:
  - `Gaia`: <https://github.com/AnkaChan/Gaia> (full research framework)
  - `TinyVBD`: <https://github.com/AnkaChan/TinyVBD> (minimal reference)

The formulas and numbers below were verified against the arXiv full text
(arXiv:2403.06321), the project page, and the `Gaia`/`TinyVBD` reference source.
Code references are to those repositories' `main` branch.

## Verified Reference Details (load-bearing)

- **Time integration is plain backward Euler (BDF1)**; the paper gets accuracy
  from substepping, not BDF2. Reproductions should expose substeps, not a BDF2
  option.
- **The reference does not PSD-project the element Hessian by default.** Gaia's
  Neo-Hookean path has `#define PSD_FILTERING` immediately `#undef`-ed; the
  SVD eigenvalue-clamp path is compiled out. Robustness instead comes from (a)
  the always-SPD inertia term `(m/h^2) I`, and (b) a stabilized 3x3 SPD solve
  (`CuMatrix::solve3x3_psd_stable`). DART's `solveVertexBlock` uses an SPD LDLT
  with a positive-definite check and a zero-step fallback, plus optional
  diagonal regularization, which is the same robustness idea. DART additionally
  offers an opt-in transverse-term clamp for the mass-spring block; because the
  converged stationary point depends only on the force (not the Hessian), the
  clamp changes only the descent path, not the fixed point.
- **Stable Neo-Hookean offset is `a = 1 + mu/lambda`** with energy density
  `Psi = mu * 0.5 (||F||_F^2 - 3) + lambda * 0.5 (det F - a)^2` and per-tet
  energy `E = A * Psi` (`A` = rest volume). Getting `a` wrong moves the rest
  state off the energy minimum.
- **Rayleigh damping is built from the elastic Hessian and divided by `h`:**
  `H_damp = (k_d / h) d^2E/dx_i^2`, with damping force
  `-(k_d/h) d^2E/dx_i^2 (x_i - x_i^t)`. Two coefficients (hydrostatic and
  deviatoric), paper range ~1e-6..1e-7.
- **Adaptive initialization** (TinyVBD `forwardStep`):
  `a_tilde = clamp((a_prev . g_hat), 0, ||a_ext||)`,
  `x_init = x^t + h v^t + h^2 g_hat a_tilde`, where `a_prev = (v^t - v^{t-1})/h`.
  First step (no previous velocity) falls back to `x_init = y`.
- **Chebyshev semi-iterative acceleration** (applied across iterations):
  `omega_1 = 1`, `omega_2 = 2/(2 - rho^2)`,
  `omega_n = 4/(4 - rho^2 omega_{n-1})`,
  `x^(n) = omega_n (x_bar^(n) - x^(n-2)) + x^(n-2)`, applied only when
  `omega > 1`. `rho` in `(0,1)` (TinyVBD default 0.5). No Anderson acceleration.
- **Vertex coloring (not element coloring)**: 3-8 colors for vertex graphs vs
  7-76 for element/dual graphs; Gauss-Seidel across colors, parallel within a
  color, using the latest positions from earlier colors.
- **Contact is penalty-based** (`E_c = 0.5 k_c d^2`, `d = max(0, gap)`), not an
  IPC log-barrier, so VBD is not penetration-free. Friction is IPC-style lagged
  with the `f1` smoothing on tangential speed and a Gauss-Newton Hessian.

Cross-check any further constant against the reference source before relying on
it.

## Method Summary

VBD is a **block coordinate descent** minimizer for the variational form of
implicit (backward) Euler time integration of elastodynamics. Each timestep it
minimizes

```
G(x) = (1 / (2 h^2)) (x - y)^T M (x - y) + E(x)
```

where `h` is the timestep, `M` is the (lumped) mass matrix, `E(x)` is the total
elastic (and contact) potential, and the inertial target is

```
y = x_t + h v_t + h^2 M^{-1} f_ext
```

(`f_ext` includes gravity). Minimizing `G` is equivalent to one implicit-Euler
step; the minimizer `x*` gives the new positions and `v_{t+1} = (x* - x_t)/h`.

Instead of a global Newton/PCG solve, VBD sweeps **one vertex block at a time**.
For vertex `i` (3 DOFs), holding all other vertices fixed, it takes one
regularized Newton step on `G` restricted to `x_i`:

```
f_i = -dG/dx_i = -(m_i / h^2)(x_i - y_i) - sum_{e in N(i)} dE_e/dx_i
H_i =  d^2G/dx_i^2 = (m_i / h^2) I_3 + sum_{e in N(i)} d^2E_e/dx_i^2
x_i <- x_i + H_i^{-1} f_i
```

where `N(i)` is the set of elements (springs / tets / contacts) incident to
vertex `i`. `H_i` is a single symmetric 3x3 matrix, so each block solve is a
fixed-size SPD solve — no global sparse system. Robustness comes from keeping
`H_i` SPD: the inertia term `(m_i/h^2) I_3` is always SPD, and each element block
is PSD-projected (e.g. clamping indefinite geometric-stiffness terms) so the sum
is SPD.

### Parallelization by graph coloring

Vertices that do not share an element can be updated independently. VBD
partitions vertices into color classes (greedy graph coloring over the
vertex–vertex adjacency induced by elements) and, within a color, updates all
vertices in parallel (Jacobi); colors are processed sequentially (Gauss-Seidel).
This is the key to GPU throughput. Several sweeps over all colors are performed
per timestep.

### Acceleration and initialization

VBD reaches good accuracy in few iterations via (a) an **adaptive initial
guess** that blends the inertial prediction with the previous step's
acceleration, and (b) an optional **Chebyshev semi-iterative acceleration** of
the block-descent sweeps. The exact blend rule and Chebyshev parameters are to
be transcribed from the paper during Phase 5.

## Component-by-Component Gap

| VBD component                                             | DART experimental world today                                                                                            | Phase that closes the gap         |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ | --------------------------------- |
| Variational implicit-Euler objective `G(x)`               | Present: `evaluateDeformableObjective` already assembles inertia + spring (+ barrier) energy/gradient                    | Phase 1 reuses the same objective |
| Inertial target `y` with gravity/damping/external accel   | Present: `world_step_stage.cpp` inertial-target setup (`dampingScale`, `gravityStep`, `externalAccelerations`)           | Phase 1/3 reuse                   |
| Per-vertex force `f_i` (inertia + spring)                 | Partial: global gradient assembled, not per-vertex blocks                                                                | Phase 1                           |
| Per-vertex SPD Hessian `H_i` (inertia + spring)           | Missing: no per-vertex Hessian assembly                                                                                  | Phase 1                           |
| PD projection of element Hessian blocks                   | Missing                                                                                                                  | Phase 1 (spring), Phase 4 (FEM)   |
| 3x3 block Newton step                                     | Missing                                                                                                                  | Phase 1                           |
| Vertex graph coloring                                     | Missing                                                                                                                  | Phase 2                           |
| Block-descent sweep driver                                | Missing (current solver is global gradient descent)                                                                      | Phase 3                           |
| Stable Neo-Hookean tetra energy + per-vertex contribution | Missing: `DeformableMaterial` stores `youngsModulus`/`poissonRatio` but no FEM energy is evaluated                       | Phase 4                           |
| Adaptive initialization + Chebyshev acceleration          | Missing                                                                                                                  | Phase 5                           |
| VBD damping model                                         | Partial: spring `damping` enters the inertial target; no element-level VBD damping                                       | Phase 5                           |
| Selectable VBD solver path in World step                  | Missing: only one deformable stage exists                                                                                | Phase 6                           |
| Vertex-based contact + friction                           | Partial: `deformable_contact` distance/barrier/CCD/tangent kernels exist (IPC slices); not wired into a VBD vertex block | Phase 7                           |
| CPU parallel color sweeps + SoA                           | Missing                                                                                                                  | Phase 8                           |
| GPU (CUDA) VBD backend                                    | Missing (experimental CUDA compute seam exists under `compute/cuda`)                                                     | Phase 9                           |
| Example/scene corpus + visual evidence                    | Missing for VBD                                                                                                          | Phase 10                          |

## Elastic Energy Targets

- **Mass-spring** (Phase 1): `E_e = (k/2)(l - L)^2`, `l = ||x_b - x_a||`,
  `n = (x_b - x_a)/l`. Per-vertex gradient `dE/dx_a = -k(l-L) n` (already in
  `evaluateDeformableObjective`). Per-vertex Hessian block
  `H_aa = k[ n n^T + (1 - L/l)(I - n n^T) ]`, transverse factor clamped to
  `>= 0` for the SPD local Hessian.
- **Stable Neo-Hookean** (Phase 4): the Smith et al. 2018 energy density used by
  the reference, with Lame parameters derived from
  `DeformableMaterial.youngsModulus` / `poissonRatio`, plus the per-tet,
  per-vertex force and PD-projected Hessian block. Exact constants transcribed
  during Phase 4.

## Reference Performance Targets ("beat the reference / paper")

Paper Table 1 / figures (GPU = NVIDIA RTX 4090 unless noted) are the numbers to
beat:

| Scene               | Verts | Tets  | h     | Substeps | Iters | Time/frame (avg/max) |
| ------------------- | ----- | ----- | ----- | -------- | ----- | -------------------- |
| Twisting thin beams | 97K   | 266K  | 1/300 | —        | 100   | 60 / 78 ms           |
| Squishy ball drops  | 230K  | 700K  | 1/120 | —        | 120   | 15 / 17 ms           |
| 216 squishy balls   | 48M   | 151M  | 1/240 | 4        | 40    | 3.6 / 3.9 s          |
| 10,368 models       | 36M   | 124M  | 1/120 | 2        | 60    | 4.2 / 4.7 s          |
| Tearing cloth (CPU) | 2,500 | 4,800 | —     | —        | —     | 11.2 / 11.5 ms       |

Headline comparisons: ~10x faster than XPBD at matched settings and stable on
high mass ratios (1:2000) where XPBD fails; far cheaper per iteration than
Projected Newton. Limitation: information propagates only `n_colors` graph-hops
per iteration, so very stiff global problems converge slower than Newton, and
high stiffness ratios (~1:10,000) converge slowly.

PLAN-082 gates, to be filled with controlled benchmark JSON:

- Iterations-to-converged-energy vs the existing DART gradient-descent solver on
  matched mass-spring scenes (correctness + efficiency, Phase 3). The first
  reproduction target is the `TinyVBD` tilted-strand default scene: 20 verts,
  spring stiffness `1e8`, mass ratio 1:1000 (last vertex mass 1000), skip-spring
  stiffness 100, `h = 1/60`, 100 iterations, gravity `(0, -10, 0)`.
- CPU per-step wall time on matched scenes vs `TinyVBD`/`Gaia` CPU (Phase 8).
- GPU per-step wall time / frame rate on the paper's high-resolution scenes vs
  `Gaia` GPU and the paper's reported numbers (Phase 9).

Until those phases land with controlled benchmark JSON, no performance-parity
claim is made.

## DART-Specific Constraints

- Public API stays backend-neutral and DART-owned (no `vbd`/`Gaia` vocabulary in
  user-facing solver selectors until a deliberate API slice).
- No runtime/vendored dependency on the reference repos.
- Each slice keeps `pixi run lint`, the focused build, focused tests, and
  `pixi run check-api-boundaries` green, and records benchmark smoke separately
  from controlled performance claims.
