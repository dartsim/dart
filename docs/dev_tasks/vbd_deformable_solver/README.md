# Vertex Block Descent Solver - Dev Task

## Current Status

- [ ] Phase 0: ground the upstream Vertex Block Descent (VBD) method, reference
      implementations, and the paper-vs-DART gap audit; add the references
      catalog entry and the PLAN-082 dashboard/plan files.
  - [x] Grounding sub-slice: PLAN-082 dashboard entry, the numbered plan file,
        the `chen-2024-vbd` references catalog entry, the comparative
        `tinyvbd`/`gaia` entries, the VBD paper/reference gap audit, and this
        dev-task tracker.
- [x] Phase 1: per-vertex block kernels for the variational implicit-Euler
      objective — inertia term plus mass-spring elastic force/Hessian, the
      positive-definite 3x3 vertex Hessian, and the block Newton step.
  - [x] Vertex block kernel sub-slice: `VertexBlock` accumulator, inertia-term
        force/Hessian, mass-spring force/Hessian with an opt-in PSD clamp of the
        transverse geometric-stiffness term, the regularized SPD 3x3 block
        solve with a zero-step fallback, finite-difference derivative
        regression tests, and `bm_vbd_vertex_block_kernel`.
- [x] Phase 2: vertex graph coloring for the parallel Gauss-Seidel sweep
      (parallel within a color, sequential across colors).
  - [x] Vertex coloring sub-slice: element-induced `VertexAdjacency` (edge,
        triangle, tetrahedron, and clique builders), deterministic
        Welsh-Powell greedy `greedyColorVertices`, per-color groups, an
        `isProperColoring` conflict-free verifier, and coloring benchmarks.
- [x] Phase 3: the single-body VBD block-descent solver driver and convergence
      parity against the existing mass-preconditioned gradient-descent solver
      on small mass-spring systems.
  - [x] Block-descent driver sub-slice: `SpringElement`/`SpringAdjacency`,
        `colorSprings`, the colored Gauss-Seidel `blockDescentMassSpring`
        driver with a residual-norm stat, the `massSpringObjective` evaluator,
        convergence-to-zero-residual, monotone energy-decrease,
        converged-state parity against an independent gradient-descent
        minimizer of the same objective, same-color independence regressions,
        and `bm_vbd_block_descent`.
- [x] Phase 4: FEM hyperelastic energy (Stable Neo-Hookean tetrahedra) and its
      per-vertex force/Hessian contributions, wired into the block-descent
      driver for a volumetric solve.
  - [x] Stable Neo-Hookean kernel sub-slice: Lame-from-Young/Poisson
        conversion, the tetrahedral rest-shape (`Dm^{-1}`, rest volume) and
        deformation gradient, the Smith-2018 energy density
        `Psi = (mu/2)(||F||^2 - 3) + (lambda/2)(det F - a)^2` with
        `a = 1 + mu/lambda`, the analytic first Piola stress, and the per-vertex
        force plus exact 3x3 Hessian (assembled column-wise from the analytic
        stress differential), with finite-difference force/Hessian regression
        tests, rest-state equilibrium, inversion stability, and
        `bm_vbd_neo_hookean`.
  - [x] Tetrahedral driver sub-slice: `TetMeshElement`/`TetAdjacency`,
        `colorTetMesh`, the colored Gauss-Seidel `blockDescentTetMesh` driver
        accumulating inertia + incident-tet Neo-Hookean blocks, and the
        `tetMeshObjective` evaluator, validated for rest-stability, residual
        convergence, monotone energy decrease, and converged-state parity
        against an independent global gradient-descent minimizer of the tet
        objective, plus a `BM_VbdTetMeshStep` benchmark.
- [ ] Phase 5: VBD acceleration — adaptive inertial/previous-step initialization
      and optional Chebyshev semi-iterative acceleration — plus the VBD damping
      model.
  - [x] Acceleration/damping primitives sub-slice: `adaptiveInitialPosition`
        (gravity-projected previous-acceleration blend, clamped to
        `[0, ||a_ext||]`, with first-step inertial fallback),
        `chebyshevOmega`/`applyChebyshev` (the semi-iterative weight recurrence
        and affine over-relaxation), and `addRayleighDamping`
        (stiffness-proportional `(k_d/h) H_elastic` Hessian/force), with unit
        tests for each.
  - [ ] Remaining Phase 5 work: thread these through a multi-step VBD stepping
        loop (initialization at step start, Chebyshev across sweeps, damping in
        the per-vertex assembly) once the solver-stepping slice (Phase 6) lands.
- [ ] Phase 6: wire VBD as a DART-owned, selectable deformable solver path in
      the experimental World step and add integration tests.
  - [x] Stepping-loop sub-slice: a self-contained `vbdStepMassSpring` driver
        that performs one implicit-Euler step (inertial targets, adaptive warm
        start, colored block-descent sweeps with optional Chebyshev
        over-relaxation, velocity update, and previous-velocity bookkeeping),
        with tests for implicit-Euler free-fall, fixed vertices, multi-step
        stability, Chebyshev no-op/convergence, and adaptive-init convergence.
  - [x] World solver-wiring sub-slice: an internal, opt-in
        `comps::DeformableVbdConfig` (not serialized, not public) selects the VBD
        inner solver inside the algorithm-neutral `DeformableDynamicsStage` for
        contact-free mass-spring bodies; the stage caches the spring
        coloring/adjacency per body and reuses the existing inertial-target
        setup and write-back. World-level integration tests confirm VBD runs
        when opted in, the default solver runs otherwise, VBD matches the
        gradient-descent solver on a contact-free scene, and a hanging chain is
        stable, with no regression in the existing deformable suite.
  - [ ] Remaining Phase 6 work: thread element damping and Chebyshev into the
        World VBD path, extend it to tetrahedral and surface-contact bodies, and
        decide the public solver-selection surface.
- [ ] Phase 7: vertex-based contact and friction (reusing the existing
      `deformable_contact` distance/barrier/CCD kernels).
  - [x] Half-space penalty-contact sub-slice: `contact_kernel.hpp`
        (`ContactPlane`, `addHalfSpacePenaltyContact`) adds the VBD penalty
        contact `E_c = (k_c/2) d^2` for a vertex against a static half-space
        (force `k_c d n`, PSD Hessian `k_c n n^T`), plus the contact-aware driver
        `blockDescentMassSpringGround`. Tests cover the finite-difference force
        match, inactivity above the plane, PSD Hessian, a particle resting on
        the ground without tunneling, and a pinned spring net sagging onto the
        ground.
  - [ ] Remaining Phase 7 work: vertex-triangle / edge-edge penalty contact
        (reusing the `deformable_contact` distance kernels), IPC-style lagged
        friction, self-collision, and wiring contact into the World VBD path.
- [ ] Phase 8: CPU performance optimization (SoA layout, multithreaded color
      sweeps) with benchmarks targeting the reference CPU numbers.
  - [x] CPU baseline-comparison sub-slice: `bm_vbd_world_solver` steps the same
        contact-free mass-spring grid through the real World pipeline with the
        VBD inner solver and with the default gradient-descent solver. Honest
        finding: fixed-iteration single-threaded VBD is currently SLOWER than
        the early-terminating gradient-descent baseline on small, well-
        conditioned scenes (it always runs its full sweep budget). This matches
        the paper — VBD's wins are GPU throughput from graph coloring and
        stiff/high-mass-ratio stability, not single-thread CPU speed. It
        motivates the remaining Phase 8 work below.
  - [x] Residual early-termination sub-slice: a `convergenceDisplacement`
        option stops the block-descent sweeps once the largest per-vertex update
        falls below a threshold (wired through `DeformableVbdConfig`), with tests
        that early termination reaches the same converged state as the full
        sweep budget and that it is disabled at threshold 0. With early
        termination, single-threaded VBD now BEATS the in-repo gradient-descent
        baseline solver by ~2-4x on the contact-free mass-spring grid
        benchmark (it converges in fewer per-vertex Newton sweeps and skips the
        baseline's line-search objective re-evaluations).
  - [x] Multithreaded color-sweep sub-slice: `parallelBlockDescentMassSpring`
        (`detail/deformable_vbd/parallel_block_descent.hpp`) runs the colored
        Gauss-Seidel sweep on a fixed worker-thread pool with a C++20
        `std::barrier` between colors. Same-color vertices are independent, so
        the parallel result is bit-identical to the serial driver (verified).
        Thread scaling on a 96x96 grid (9216 verts, 20 sweeps, wall clock): 1
        thread 30.2 ms, 2 threads 18.3 ms (1.65x), 4 threads 11.7 ms (2.6x), 8
        threads 8.66 ms (3.5x).
  - [ ] Remaining Phase 8 work: SoA layout, wiring the parallel driver into the
        World stage, and benchmarking against the external Gaia CPU numbers.
- [ ] Phase 9: GPU (CUDA) VBD backend behind the experimental compute boundary
      with benchmarks targeting the reference/paper GPU numbers.
  - [x] CUDA mass-spring kernel sub-slice: `compute/cuda/vbd_block_descent_cuda`
        (a per-color block-update kernel with an analytic SPD 3x3 cofactor solve,
        a single-stream Gauss-Seidel sweep over colors, and a host wrapper that
        uploads once / launches per color per sweep / downloads once), added to
        the experimental `-cuda` static target behind `DART_ENABLE_EXPERIMENTAL_CUDA`.
        A device-skipping test confirms the GPU result matches the CPU
        block-descent solve to 1e-6 and that fixed vertices stay put; a GPU-vs-CPU
        benchmark shows the GPU ~9x faster at 4k vertices and ~26x faster at 16k
        vertices, scaling near-flat while the CPU path grows linearly.
  - [x] Device-resident rollout sub-slice: `vbdRolloutMassSpringCuda` runs the
        full per-step pipeline on the GPU (inertial-target prediction kernel,
        colored sweeps, velocity-update kernel) for many steps with one upload
        and one download. A device-skipping test confirms it matches the CPU
        stepper over 20 steps; the rollout benchmark gives ~1.08 ms/step
        steady-state, i.e. ~45x faster than the single-threaded CPU at 16k
        vertices (the per-step cost is launch-overhead-bound at these grid
        sizes, so the GPU has spare capacity for larger meshes).
  - [ ] Remaining Phase 9 work: a tetrahedral Neo-Hookean GPU kernel, CUDA-graph
        capture of the per-color sweeps, float/mixed precision, and reproducing
        the paper's tetrahedral scenes on an RTX-4090 host (see the
        same-GPU reproduction plan below).
- [ ] Phase 10: complete the upstream example/scene corpus as DART-native
      tests, examples, benchmarks, profiling artifacts, and headless Filament
      visual evidence.

## Goal

Implement Vertex Block Descent (Chen et al., SIGGRAPH 2024) as a DART-owned,
backend-neutral deformable solver inside the experimental World, in bounded
PRs, until DART covers the VBD method family (variational implicit-Euler block
coordinate descent, hyperelastic energies, graph-colored parallel sweeps,
acceleration, contact, and friction), proves correctness against the reference
implementations (`TinyVBD`/`Gaia`) and the paper, and beats the reference and/or
paper performance numbers on CPU and GPU.

VBD minimizes the **same** variational implicit-Euler objective the existing
experimental deformable solver already targets:

```
G(x) = sum_i (m_i / (2 h^2)) ||x_i - y_i||^2 + E_elastic(x) [+ E_contact(x)]
```

where `y_i` is the inertial target. The existing solver minimizes `G` with a
global mass-preconditioned gradient descent plus Armijo line search
(`evaluateDeformableObjective` in
`dart/simulation/experimental/compute/world_step_stage.cpp`). VBD instead
minimizes `G` by **per-vertex 3x3 block coordinate descent**: for each vertex
`i`, take one regularized Newton step on the local objective
`G` restricted to `x_i` (all other vertices held fixed), using the local force
`f_i = -dG/dx_i` and the local Hessian `H_i = d^2G/dx_i^2`. Vertices that share
no element are grouped by graph coloring and updated in parallel; colors are
swept Gauss-Seidel. This shares the existing node/spring/tetra ECS components
and the inertial-target setup, so VBD is an alternative solver on the same
foundation rather than a parallel data model.

## Non-Goals For The Current Implementation Slices

- No claim that DART has full VBD, FEM hyperelasticity, VBD contact, VBD
  friction, GPU VBD, or paper/reference performance parity until the relevant
  phase lands with evidence.
- No vendored or runtime dependency on `AnkaChan/Gaia` or `AnkaChan/TinyVBD`;
  the DART implementation is independent and DART-owned.
- The public solver/stage names stay algorithm-neutral and DART-owned; VBD
  vocabulary is not exposed as a user-facing solver selector until a deliberate
  API slice decides the public surface.
- Internal kernel slices are scaffolding only and must not be described as a
  wired, scene-level VBD solver until Phase 6 lands.

## Key Decisions

- VBD reuses the existing deformable ECS components
  (`comps::DeformableNodeState`, `DeformableSpringModel`, `DeformableMeshTopology`,
  `DeformableMaterial`) and the existing inertial-target/gravity/damping setup.
  It does not introduce a parallel deformable data model.
- The VBD-specific internal kernels live under
  `dart/simulation/experimental/detail/deformable_vbd/` in the
  `dart::simulation::experimental::detail::deformable_vbd` namespace, mirroring
  the `deformable_contact` layout used by the IPC slices.
- Contact/friction slices reuse the `deformable_contact` distance, barrier,
  CCD, and tangent kernels rather than duplicating them.
- The per-vertex local Hessian is kept symmetric positive-definite (inertia
  term plus PD-projected element blocks) so the 3x3 block solve is always well
  posed, matching the VBD robustness guarantee.
- Performance claims are gated on controlled benchmark runs; local smoke
  numbers collected with CPU frequency scaling enabled are recorded as smoke
  numbers, not final performance claims.

## Immediate Next Steps

1. Land Phase 4: Stable Neo-Hookean tetrahedral energy and its per-vertex
   force/Hessian contribution (deformation gradient `F = Ds Dm^{-1}`, energy
   `A(mu*0.5(||F||^2-3) + lambda*0.5(det F - a)^2)` with `a = 1 + mu/lambda`),
   extending the block kernel and driver to volumetric meshes.
2. Land Phase 5: adaptive inertial/previous-step initialization, optional
   Chebyshev acceleration of the sweeps, and the Rayleigh damping term in the
   per-vertex block.
3. Land Phase 6: wire VBD as a DART-owned, selectable inner solver for the
   deformable stage with integration tests.
4. Add focused unit tests and a benchmark for each landed slice before any
   solver-wiring or scene-level claim.

## Verification

See the `chen-2024-vbd` entry in
[`../../readthedocs/papers.md`](../../readthedocs/papers.md) and the
[VBD paper gap audit](vbd-paper-gap-audit.md) for the method scope and the
reference numbers being targeted.

Per-slice local gates mirror the PLAN-081 IPC dev-task verification discipline:
`pixi run lint`, a focused target build, the focused test binary, the CTest
registration path, the focused benchmark smoke, and
`pixi run check-api-boundaries`.

For the vertex block kernel sub-slice, keep the verification language precise:
it covers the internal per-vertex inertia force/Hessian, the mass-spring
force/Hessian (matching the reference's `k[(1-L/l)I + (L/l) n n^T]` block) with
an opt-in PSD clamp, the regularized SPD 3x3 block solve with a zero-step
fallback for non-PD/non-finite blocks, finite-difference gradient/Hessian
checks against the spring energy in stretched and compressed configurations,
PSD verification of the clamped block, and microbenchmark timings. It does not
yet cover FEM hyperelasticity, damping, acceleration, contact, friction, the
solver driver, or any solver wiring.

For the vertex coloring sub-slice, keep the verification language precise: it
covers the element-induced vertex adjacency (edge/triangle/tetrahedron/clique
builders with self-loop, out-of-range, and duplicate handling), the
deterministic Welsh-Powell greedy coloring, the per-color groups, and the
`isProperColoring` conflict-free verifier, with tests for isolated vertices,
bipartite chains, clique color counts, the `maxDegree + 1` bound, partition
correctness, and determinism. It does not yet cover parallel execution,
re-coloring on topology change, or solver wiring.

For the block-descent driver sub-slice, keep the verification language precise:
it covers the single-body colored Gauss-Seidel mass-spring driver, the
incident-spring adjacency, the residual-norm stat, and the objective evaluator,
with tests that fixed vertices stay put, the residual is driven to zero, the
objective decreases monotonically per sweep, the converged state matches an
independent mass-preconditioned gradient-descent minimizer of the same
objective to 1e-6, and a serial within-color sweep equals the parallel Jacobi
update (same-color independence). It is a single-body CPU driver only; it does
not yet cover FEM energy, damping, acceleration, multi-body coupling, contact,
friction, multithreaded or GPU execution, or any wiring into `World::step()`.

For the Stable Neo-Hookean kernel sub-slice, keep the verification language
precise: it covers the Lame conversion, the tetrahedral rest-shape and
deformation gradient, the energy density and analytic first Piola stress, and
the per-vertex force plus the exact (non-PSD-projected) 3x3 Hessian, with tests
that the rest state has zero stress/force, the force and Hessian match finite
differences in stretched and inverted configurations, the energy/force/Hessian
stay finite under element inversion, and the inertia-anchored block is
positive-definite. It is a per-tetrahedron kernel only; the tetrahedral blocks
are not yet wired into the block-descent driver (no tet adjacency/coloring or
volumetric solve yet), and damping, acceleration, contact, and friction are not
implemented.

For the tetrahedral driver sub-slice, keep the verification language precise: it
covers the tet element/adjacency structures, tet-induced vertex coloring, the
colored Gauss-Seidel `blockDescentTetMesh` driver accumulating inertia plus
incident-tet Neo-Hookean blocks, and the `tetMeshObjective` evaluator, with
tests that the rest configuration stays at rest, the residual is driven to zero,
the objective decreases monotonically per sweep, the mesh coloring is proper,
and the converged state matches an independent global gradient-descent minimizer
of the same tet objective. It is a single-body CPU driver only; multi-body
coupling, damping, acceleration, contact, friction, multithreaded/GPU execution,
and any wiring into `World::step()` are not implemented.

For the acceleration/damping primitives sub-slice, keep the verification
language precise: it covers the pure-function `adaptiveInitialPosition`,
`chebyshevOmega`, `applyChebyshev`, and `addRayleighDamping` helpers, with tests
for the first-step inertial fallback, free-fall saturation, supported-body
gravity drop, partial-alignment blend, the Chebyshev weight recurrence and its
boundedness, the affine over-relaxation (identity at unit weight), the scaled
damping Hessian/force, dissipativity, and the zero-coefficient no-op. These are
standalone helpers; they are not yet threaded through a multi-step VBD stepping
loop (no step-start initialization, cross-sweep Chebyshev, or in-assembly
damping yet), which depends on the Phase 6 solver-stepping slice.

For the stepping-loop sub-slice, keep the verification language precise: it
covers the single-body mass-spring `vbdStepMassSpring` implicit-Euler step
(inertial-target computation, adaptive warm start, colored block-descent sweeps,
optional Chebyshev over-relaxation across sweeps, velocity update, and
previous-velocity bookkeeping), with tests for the no-spring implicit-Euler
free-fall trajectory, fixed-vertex immobility, 400-step finiteness/boundedness
plus net downward stretch of a hanging chain, Chebyshev being a no-op at
`rho = 0` and converging to the same step solution at large iteration counts,
adaptive initialization converging to the same step solution as a cold start,
and previous-velocity advancement. It is a single-body mass-spring CPU stepper;
element damping, a tetrahedral stepper, and any wiring behind
`DeformableDynamicsStage` are not implemented.

Local gate (Phase 6 stepping loop, first pass) on 2026-05-28: the focused target
build and 7 `test_vbd_stepper` cases passing.

For the World solver-wiring sub-slice, keep the verification language precise:
it covers the internal opt-in `comps::DeformableVbdConfig`, the VBD branch in
`advanceDeformableBody` for contact-free mass-spring bodies (no tetrahedra,
ground barriers, rigid obstacles, or surface-contact topology), the per-body
cached spring coloring/adjacency, and the new VBD stage stats. Tests confirm the
VBD path runs only when opted in, the default gradient-descent path runs
otherwise, VBD and the default solver agree on a contact-free mass-spring scene,
and a hanging chain is stable over 200 steps, with the existing
`test_deformable_body` suite (43 cases) still green. It does not yet cover
tetrahedral, surface-contact, ground-barrier, or rigid-obstacle bodies under
VBD, World-path damping/Chebyshev, or a public solver-selection API.

Local gate (Phase 6 World solver wiring, first pass) on 2026-05-28: the library
rebuild plus 4 `test_vbd_world_solver` cases and 43 `test_deformable_body` cases
passing.

Local gate (Phase 8 CPU baseline comparison + early termination, on 2026-05-28,
CPU scaling enabled, treat as smoke): `bm_vbd_world_solver` steps a pinned
spring grid through the real World pipeline.

- Fixed-iteration VBD (20 sweeps, no early termination) was SLOWER than the
  early-terminating default gradient-descent solver: e.g. ~4.17 ms vs ~1.12 ms
  at 24x24, because it always ran its full sweep budget.
- After adding residual early termination (cap 50 sweeps, stop at 1e-6
  displacement), single-threaded VBD BEATS the default solver: per-step times of
  ~34 us vs ~77 us (8x8, 64 verts), ~152 us vs ~641 us (16x16, 256 verts), and
  ~358 us vs ~1063 us (24x24, 576 verts) — roughly 2-4x faster.

This beats the in-repo gradient-descent reference solver on contact-free
mass-spring scenes on a single CPU thread.

For the multithreaded color-sweep sub-slice, keep the verification language
precise: it covers `parallelBlockDescentMassSpring`, which distributes each
color's vertices across a fixed worker-thread pool with a `std::barrier` between
colors. `test_vbd_parallel_block_descent` confirms the parallel result is
identical to the serial driver for 2/4/8 threads, the single-thread path falls
back to the serial driver, and it converges to a low residual. The scaling
benchmark (`BM_VbdParallelGridStep`, 96x96 grid) gave 1.65x/2.6x/3.5x speedups
at 2/4/8 threads. It does not yet wire the parallel driver into the World stage
or add a SoA layout.

For the half-space penalty-contact sub-slice, keep the verification language
precise: it covers `addHalfSpacePenaltyContact` (VBD penalty contact against a
static half-space) and the contact-aware `blockDescentMassSpringGround` driver,
with tests for the finite-difference force match while penetrating, inactivity
above the plane, the positive-semidefinite contact Hessian, a particle resting
on the ground without tunneling, and a pinned spring net sagging onto the
ground. It is half-space (ground/obstacle) penalty contact only:
vertex-triangle / edge-edge contact, friction, self-collision, and World wiring
are not implemented. Local gate on 2026-05-28: the focused target build and 4
`test_vbd_contact` cases passing.

Local gate (external TinyVBD reference comparison, on 2026-05-28, single CPU
thread, compute-only, treat as smoke): the upstream `AnkaChan/TinyVBD` reference
was cloned and built (its `Eigen/core` include was case-fixed to `Eigen/Core`
for Linux, file output disabled, and a compute timer added) and run on its
default tilted-strand scene (20 vertices, structural + skip springs, 100
iterations/frame, 1:1000 tip mass ratio): **0.468 ms/frame**. DART's VBD on the
matched scene (`BM_VbdTinyStrandStep`, same vertex/spring/iteration counts) ran
at **0.212 ms/step — about 2.2x faster than the TinyVBD reference**. DART uses a
double-precision LDLT 3x3 block solve while TinyVBD uses a float
`colPivHouseholderQr` solve, so this is not bit-identical physics; it is a
like-for-like per-step compute comparison on the reference's own scene. This
beats the external reference implementation on CPU. The full `Gaia` GPU
framework and the paper's tetrahedral RTX-4090 scene numbers (different scenes
and hardware) remain out of scope for this environment.

For the CUDA mass-spring kernel sub-slice, keep the verification language
precise: it covers the device per-color block-update kernel (analytic SPD 3x3
cofactor solve), the single-stream Gauss-Seidel sweep, and the host
upload/launch/download wrapper. `test_vbd_block_descent_cuda` skips without a
device; on the local NVIDIA RTX 5000 Ada GPU it confirms the GPU result matches
the CPU block-descent solve to 1e-6 and fixed vertices stay put. It is
mass-spring only (no tetrahedra, contact, damping, or device-resident rollout),
uses double precision, and does not yet target the paper's tetrahedral scenes or
the published RTX-4090 numbers.

Local gate (Phase 9 CUDA mass-spring kernel, on 2026-05-28, NVIDIA RTX 5000 Ada
Laptop GPU, CUDA build via `pixi run -e cuda`, treat as smoke):
`test_vbd_block_descent_cuda` (2 cases) passing on the GPU. `bm_vbd_cuda`
per-step times (20 sweeps, mass-spring grid) — CPU vs GPU: 32x32 (1024 verts)
3.3 ms vs a 128 ms first-call CUDA-context-init warmup (not steady state); 64x64
(4096 verts) 13.7 ms vs 1.5 ms (~9x); 128x128 (16384 verts) 57.0 ms vs 2.2 ms
(~26x). The GPU scales near-flat with vertex count while the single-threaded CPU
grows linearly, consistent with VBD's graph-colored GPU-throughput design. This
is GPU-vs-our-own-CPU; it is not yet a comparison against the external
TinyVBD/Gaia GPU numbers or the paper's tet-scene numbers.

Local gate (Phase 9 device-resident rollout, on 2026-05-28, NVIDIA RTX 5000 Ada
Laptop GPU): `test_vbd_block_descent_cuda` `RolloutMatchesCpuStepper` confirms
the GPU rollout matches the CPU stepper over 20 steps. `bm_vbd_cuda`
`BM_VbdCudaRollout` ran 50 device-resident steps in ~54 ms (~1.08 ms/step,
nearly flat across 1k/4k/16k vertices, i.e. launch-overhead-bound with spare GPU
capacity). Against the single-threaded CPU per-step (`BM_VbdCpuStep`: ~3.0 ms at
1k, ~10.2 ms at 4k, ~49.2 ms at 16k), the device-resident GPU rollout is ~45x
faster at 16k vertices. DART's VBD wins decisively on this GPU.

## Same-GPU (RTX-4090) Reproduction Plan

The paper's Table 1 numbers are on an NVIDIA RTX 4090; this environment has an
RTX 5000 Ada Laptop GPU, so absolute-number parity is not directly comparable
here. To compare on the same GPU as the paper later:

1. On an RTX-4090 host, configure with `DART_ENABLE_EXPERIMENTAL_CUDA=ON` and
   `DART_CUDA_ARCHITECTURES=89` (Ada). Build `bm_vbd_cuda` and run
   `BM_VbdCudaRollout`; record per-step times.
2. Add the tetrahedral Neo-Hookean GPU kernel (remaining Phase 9) so the paper's
   volumetric scenes (e.g. the 97K-vertex twisting beams, the 230K-vertex
   squishy-ball drops) can be stepped, and capture per-frame times at the
   paper's iteration counts and substeps.
3. Build the `AnkaChan/Gaia` reference on the same host (deps: OneTBB 2021.12,
   Eigen 3.4, Embree 3.13.1, submodules MeshFrame2/CuMatrix/polyscope; flags
   `BUILD_VBD`, `BUILD_Collision_Detector`) and run its shipped twist-beam /
   squishy-ball scenes, then compare DART's per-frame GPU times against both
   Gaia and the paper's Table 1 on the matched RTX-4090.
4. Publish the comparison as benchmark JSON through the performance dashboard.

Local gate (Phase 5 acceleration/damping primitives, first pass) on 2026-05-28:
the focused target build and 9 `test_vbd_acceleration` cases passing. These are
constant-time pure helpers, so no separate microbenchmark is added; their effect
will be measured in the Phase 6 step-loop benchmarks.

Local gate (Phase 4 Neo-Hookean kernel + tetrahedral driver, first pass) on
2026-05-28: the focused target build, 7 `test_vbd_neo_hookean` cases and 5
`test_vbd_tet_mesh_descent` cases passing. Kernel benchmark smoke (CPU scaling
enabled) of ~12 ns energy density, ~8.8 ns Piola stress, and ~146 ns for the
full per-vertex force-plus-Hessian block. Tetrahedral 20-sweep step smoke of
~0.30 ms (20 verts, 24 tets, 5 colors), ~1.18 ms (68 verts, 96 tets), and
~5.1 ms (260 verts, 384 tets, 6 colors).

Local gate (Phases 1-3, first pass) on 2026-05-28: `pixi run cmake`
reconfigure, the focused target build, 12 `test_vbd_vertex_block_kernel` cases,
10 `test_vbd_vertex_coloring` cases, and 5 `test_vbd_block_descent` cases all
passing. Benchmark smoke (CPU frequency scaling enabled, treat as smoke):
`bm_vbd_vertex_block_kernel` reported ~1.7 ns inertia term, ~9.8 ns spring
term, ~30 ns block solve, and ~92/164/267 ns assemble-and-solve for 4/8/16
incident springs; `bm_vbd_block_descent` reported a 20-sweep grid step at
~0.19 ms (8x8, 64 verts, 4 colors), ~0.88 ms (16x16, 256 verts), and ~3.7 ms
(32x32, 1024 verts, ~3.9k springs), with coloring builds of ~7/27/106 us. The
cloth grid colored with 4 colors, consistent with the paper's 3-8 vertex-graph
color range.

```bash
pixi run cmake build/default/cpp/Release
pixi run cmake --build build/default/cpp/Release --target \
  test_vbd_vertex_block_kernel test_vbd_vertex_coloring test_vbd_block_descent \
  test_vbd_neo_hookean test_vbd_tet_mesh_descent test_vbd_acceleration \
  test_vbd_stepper bm_vbd_vertex_block_kernel bm_vbd_block_descent \
  bm_vbd_neo_hookean
ctest --test-dir build/default/cpp/Release -R '^test_vbd_' --output-on-failure
./build/default/cpp/Release/bin/bm_vbd_vertex_block_kernel --benchmark_min_time=0.05s --benchmark_filter='BM_Vbd'
./build/default/cpp/Release/bin/bm_vbd_block_descent --benchmark_min_time=0.05s --benchmark_filter='BM_Vbd'
./build/default/cpp/Release/bin/bm_vbd_neo_hookean --benchmark_min_time=0.05s --benchmark_filter='BM_Vbd'
```
