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
  (`chen-2024-vbd`, `tinyvbd`, `gaia`, `ogc-2025`, `avbd-2025`).
- VBD paper/reference gap audit:
  [`104-vertex-block-descent-solver/vbd-paper-gap-audit.md`](104-vertex-block-descent-solver/vbd-paper-gap-audit.md)
  owns the method scope, the component-by-component gap, the elastic-energy
  targets, and the reference performance numbers to beat.
- OGC paper/source gap audit:
  [`104-vertex-block-descent-solver/ogc-gap-audit.md`](104-vertex-block-descent-solver/ogc-gap-audit.md)
  owns the Offset Geometric Contact research and implementation sequence for
  codimensional VBD contact.
- AVBD paper/reference gap audit:
  [`104-vertex-block-descent-solver/avbd-paper-gap-audit.md`](104-vertex-block-descent-solver/avbd-paper-gap-audit.md)
  owns the augmented-Lagrangian hard-constraint extension, demo corpus,
  CPU/GPU parity target, and paper/reference performance numbers to beat.

## Current Implementation Evidence

The temporary `docs/dev_tasks/vbd_deformable_solver/` tracker was retired after
the post-merge self-collision follow-up slice because the durable method audit
and remaining work now live here. PR #2781 landed the DART-owned VBD CPU+CUDA
solver path: per-vertex block kernels, graph coloring, colored Gauss-Seidel
block descent, Stable Neo-Hookean tetrahedra, Chebyshev/Rayleigh acceleration,
World solver selection, static ground contact + Coulomb friction, CPU baseline
benchmarks, CUDA mass-spring/tetrahedral rollouts, and the first GUI showcases.
PR #2801 extends that landed path so the World VBD solver honors shared FEM
tetrahedral material kernels, handles static sphere/box obstacle barriers,
adds lagged VT/EE surface self-collision penalties, and provides the TinyVBD
tilted-strand plus contact showcase py-demos.

Remaining durable work is deliberately narrower than the retired task tracker:
self-contact tangential friction, committed benchmark/profiling JSON, paper
tetrahedral scene reproduction, OGC source/code audit plus CPU proof-of-contact,
Phase 8 SoA plus Gaia CPU comparison, and same-GPU RTX-4090 Table 1
reproduction.

Maintainer direction now extends this same solver-family plan to Augmented VBD
(`avbd-2025`). AVBD is not a replacement for the VBD foundation above; it is
the hard-constraint, friction, rigid/articulated, finite-stiffness-ramp, and
CPU/GPU performance continuation of the VBD path. The active multi-session
implementation tracker is
[`../dev_tasks/avbd_solver/`](../dev_tasks/avbd_solver/). The first local
implementation slice adds a tested internal scalar-row utility for AVBD
regularization, warm starting, hard-row clamping, dual updates, and
finite-stiffness ramping. Follow-on local slices add deterministic scalar-row
keys/inventory for warm-started `lambda`/`k` state, standalone CPU
half-space contact-normal, hard point-attachment, and finite-stiffness spring
row drivers, plus a standalone finite-stiffness tetrahedral material row
driver, a bounded half-space friction-tangent row primitive, and self-contact
normal rows for point-triangle / edge-edge primitive directions and AVBD
hard-row stamping. Self-contact friction tangent rows reuse lagged
point-triangle / edge-edge tangent stencils in the combined mass-spring row
driver, with supported World generation for serial self-contact scenes plus
pairwise static/dynamic switching and circular-cone projection.
The supported mass-spring World envelope carries
contact-normal, friction-tangent, self-contact-normal, attachment, and spring
finite-stiffness families in one serial AVBD row solve. The supported
frictionless pure-tetrahedral World envelope now carries finite-stiffness
material rows with a dimensionless Lamé multiplier and separate tet-row
diagnostics, while still using the existing lagged VBD self-contact penalty
when hard self-contact rows are not requested. That pure-tet envelope can now
also combine finite-stiffness material rows with AVBD self-contact normal rows
and matching bounded self-contact friction tangent rows when requested.
The supported mass-spring friction tangent pairs now use the lagged tangential
dual to switch between static sticking and dynamic sliding and project paired
forces to the circular Coulomb cone.
Explicit fallback coverage keeps unsupported mixed spring-plus-tet,
mass-spring self-contact without the self-contact AVBD flag,
finite-stiffness-only friction scenes, Chebyshev, Rayleigh-damped, parallel,
and unsupported-row requests on the existing VBD path without partial AVBD row
counters. Those slices are still foundation work; hard-contact/friction
completeness, full contact-manifold friction persistence, dynamic contact
manifold IDs, rigid/soft coupling, GPU parity, demos, and benchmark packets
remain open.

## Relationship To PLAN-081

PLAN-081 (IPC) and PLAN-104 (VBD) both target robust deformable dynamics on the
same experimental deformable ECS components and the same variational
implicit-Euler objective. They differ in the inner solver: PLAN-081 pursues an
implicit-barrier projected-Newton-style method; PLAN-104 pursues per-vertex
block coordinate descent. VBD reuses the PLAN-081 `deformable_contact` kernels
for contact/friction rather than duplicating them. OGC starts as a PLAN-104
codimensional-contact sidecar because the paper's fast path is VBD plus local
displacement bounds, but it must compare against PLAN-081 IPC-style barriers
before any supersession claim. The public deformable stage stays
algorithm-neutral; which inner solver or contact model runs is an
internal/explicit-opt-in decision, not a leaked solver registry.

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
11. **Offset Geometric Contact evaluation** — Follow
    [`ogc-gap-audit.md`](104-vertex-block-descent-solver/ogc-gap-audit.md):
    source/code audit, internal vertex-facet and edge-edge OGC contact
    construction, conservative per-vertex bounds and truncation, VBD
    force/Hessian integration, reduced paper-scene corpus, CPU benchmark packets,
    then private GPU evidence. Keep OGC names internal until those gates pass.

## AVBD Workstreams

1. **Scalar row foundation** — Internal `detail/deformable_vbd` utilities for
   augmented-Lagrangian scalar rows: `C(x) - alpha C(x_t)`, warm-started
   `lambda`/`k`, bounded hard-row force magnitudes, dual write-back, and finite
   stiffness ramping.
2. **Row inventory + storage** — Hard equality, bounded inequality, finite
   stiffness, contact normal, friction tangent, joint, motor, fracture, and
   attachment rows with persistent IDs for warm starting.
3. **CPU deformable AVBD** — Extend the existing VBD deformable path so hard
   attachments, finite-stiffness ramped springs/tets, self-contact, and static
   obstacle contact use AVBD rows rather than pure penalty terms.
4. **CPU rigid/articulated AVBD** — Add 6-DOF rigid blocks, tangent angular
   updates, rigid contact manifolds, ball/revolute/limited joints, motors,
   breakage, and high mass-ratio articulated chains.
5. **Unified soft/rigid coupling** — Solve cloth/deformables, rigid bodies,
   articulated chains, collision constraints, joints, and attachments in one
   AVBD loop.
6. **GPU parity** — Port every row family, candidate-generation path, color
   update, dual/stiffness update, and benchmark scene through the private CUDA
   compute boundary.
7. **Paper/demo corpus** — Reproduce the 2D and 3D online demo scenes, all paper
   figures, the parameter sweeps, and the video/headline scenes in tests,
   benchmark JSON, `py-demos`, and visual evidence.
8. **Performance leadership** — Optimize CPU and GPU until DART beats the
   reference demo repositories and the published paper numbers on every claimed
   case, with hardware and command packets recorded.

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
- promotes OGC only after the sidecar proves contact construction,
  conservative bounds, VBD force/Hessian agreement, penetration-free repeated
  steps, documented limitations, benchmark/profiling JSON, and headless visual
  evidence against the same corpus used for IPC/VBD comparison;
- records benchmark/profiling JSON for kernels, the solver, and scenes on both
  CPU and GPU, and demonstrates beating the reference and/or paper numbers
  before any performance-parity claim;
- verifies long-horizon headless Filament captures for GUI examples; and
- keeps `pixi run lint`, `pixi run build`, focused C++ tests, and
  `check-api-boundaries` green for every slice.

AVBD parity additionally requires:

- every algorithm and feature in `avbd-2025`, the project page, videos, and the
  `avbd-demo2d`/`avbd-demo3d` sources to be implemented, including hard
  constraints, bounded inequalities, friction cones, finite-stiffness ramping,
  warm-started dual/stiffness state, alpha regularization, quasi-Newton Hessian
  approximation, 6-DOF rigid bodies, joints, motors, breakable constraints,
  contact persistence, and soft/rigid coupling;
- both CPU and GPU implementations for the solver and all paper/demo benchmark
  families;
- all source demo scenes, paper figures, parameter sweeps, website demos, and
  video/headline scenes represented by DART tests, benchmark packets, py-demos,
  or documented visual evidence; and
- benchmark/profiling JSON proving DART beats the reference demo repositories
  and the paper's published CPU/GPU numbers before any AVBD-complete claim.
