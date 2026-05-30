# PLAN-104: Vertex Block Descent Solver

- Operating state: `PLAN-104` in [`dashboard.md`](dashboard.md)
- Outcome: the experimental `World` can step deformable bodies with a
  DART-owned Vertex Block Descent (VBD) solver — a per-vertex, graph-colored
  block coordinate descent on the variational implicit-Euler objective — that
  matches the reference implementations for correctness and beats the reference
  and/or paper performance numbers on CPU and GPU, without exposing solver
  registries, backend/project names, ECS storage, or execution resources
  through the public facade.
- Current evidence:
  - PR #2705 provides the rigid-body solver surface, staged `WorldStepPipeline`,
    and the multi-solver architecture this plan extends.
  - PR #2711 added experimental deformable body dynamics: the public
    `DeformableBody` handle, the `comps::Deformable*` ECS components, the
    inertial-target/gravity/damping setup, and `DeformableDynamicsStage`, which
    minimizes the variational implicit-Euler objective with a global
    mass-preconditioned gradient descent plus Armijo line search.
  - PLAN-081's IPC slices added the `detail/deformable_contact` distance,
    barrier, CCD, and tangent kernels that the VBD contact phase will reuse.
  - `docs/design/simulation_solver_architecture.md` defines the solver/coupler
    direction, domain-driven assignment, and model/state/control/contact
    separation that VBD plugs into as a second deformable solver.

## Owner Docs

- Architecture rationale:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
- Public facade:
  [`../design/simulation_experimental_cpp_api.md`](../design/simulation_experimental_cpp_api.md),
  [`../design/simulation_experimental_python_api.md`](../design/simulation_experimental_python_api.md)
- Research references: [`../readthedocs/papers.md`](../readthedocs/papers.md)
  (`chen-2024-vbd`, `tinyvbd`, `gaia`).
- VBD paper/reference gap audit:
  [`../dev_tasks/vbd_deformable_solver/vbd-paper-gap-audit.md`](../dev_tasks/vbd_deformable_solver/vbd-paper-gap-audit.md)
  owns the method scope, the component-by-component gap, the elastic-energy
  targets, and the reference performance numbers to beat.
- Implementation tracking:
  [`../dev_tasks/vbd_deformable_solver/`](../dev_tasks/vbd_deformable_solver/)
  owns the multi-session slice status and resume prompt; promote durable output
  and delete the folder in the completing PR.

## Relationship To PLAN-081

PLAN-081 (IPC) and PLAN-104 (VBD) both target robust deformable dynamics on the
same experimental deformable ECS components and the same variational
implicit-Euler objective. They differ in the inner solver: PLAN-081 pursues an
implicit-barrier projected-Newton-style method; PLAN-104 pursues per-vertex
block coordinate descent. VBD reuses the PLAN-081 `deformable_contact` kernels
for contact/friction rather than duplicating them. The public deformable stage
stays algorithm-neutral; which inner solver runs is an internal/explicit-opt-in
decision, not a leaked solver registry.

## Workstreams

1. **Per-vertex block kernels** — Internal `detail/deformable_vbd` kernels that,
   for a vertex and its incident springs/tets/contacts, accumulate the local
   force `f_i = -dG/dx_i` and a symmetric positive-definite 3x3 Hessian `H_i`,
   then take one block Newton step `x_i += H_i^{-1} f_i`. Mass-spring first,
   FEM later. The inertia term and PD-projected element blocks keep `H_i` SPD.
2. **Graph coloring** — Greedy vertex coloring over the element-induced
   vertex–vertex adjacency so same-color vertices share no element and can be
   updated in parallel. Deterministic, with conflict-free verification.
3. **Block-descent driver** — A single-body sweep that, per VBD iteration,
   visits colors sequentially and updates each color's vertices, reducing `G`.
   Validated against the existing gradient-descent solver on mass-spring scenes.
4. **FEM hyperelasticity** — Stable Neo-Hookean tetrahedral energy with Lame
   parameters from `DeformableMaterial`, contributing per-vertex PD Hessian
   blocks to the block solve.
5. **Acceleration + damping** — Adaptive inertial/previous-step initialization,
   optional Chebyshev semi-iterative acceleration, and the VBD element damping
   model.
6. **Solver wiring** — Make VBD a DART-owned, selectable inner solver for the
   deformable stage with integration tests; keep the public surface neutral.
7. **Contact + friction** — Vertex-based contact and friction reusing the
   `deformable_contact` distance/barrier/CCD/tangent kernels.
8. **CPU performance** — SoA layout and multithreaded color sweeps; benchmarks
   versus the reference CPU numbers.
9. **GPU performance** — A CUDA VBD backend behind the experimental compute
   boundary; benchmarks versus the reference/paper GPU numbers.
10. **Corpus + visual evidence** — DART-native examples, tests, benchmarks,
    profiling JSON, and headless Filament captures reproducing the paper scenes.

## Acceptance Criteria

VBD-parity progress is not complete until the implementation:

- distinguishes each internal kernel slice from a wired, scene-level VBD solver
  in its verification language;
- keeps VBD naming backend-neutral and DART-owned in public signatures/docs;
- proves per-vertex force/Hessian correctness with finite-difference tests, PD
  Hessian guarantees, graph-coloring conflict-freedom, block-descent energy
  decrease, and convergence parity against the existing solver and the
  reference implementations on matched scenes;
- adds FEM hyperelasticity, acceleration, damping, contact, and friction with
  focused tests;
- records benchmark/profiling JSON for kernels, the solver, and scenes on both
  CPU and GPU, and demonstrates beating the reference and/or paper numbers
  before any performance-parity claim;
- verifies long-horizon headless Filament captures for GUI examples; and
- keeps `pixi run lint`, `pixi run build`, focused C++ tests, and
  `check-api-boundaries` green for every slice.
