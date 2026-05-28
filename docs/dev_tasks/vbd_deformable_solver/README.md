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
- [ ] Phase 6: wire VBD as a DART-owned, selectable deformable solver path in
      the experimental World step and add integration tests.
- [ ] Phase 7: vertex-based contact and friction (reusing the existing
      `deformable_contact` distance/barrier/CCD kernels).
- [ ] Phase 8: CPU performance optimization (SoA layout, multithreaded color
      sweeps) with benchmarks targeting the reference CPU numbers.
- [ ] Phase 9: GPU (CUDA) VBD backend behind the experimental compute boundary
      with benchmarks targeting the reference/paper GPU numbers.
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
  test_vbd_neo_hookean test_vbd_tet_mesh_descent bm_vbd_vertex_block_kernel \
  bm_vbd_block_descent bm_vbd_neo_hookean
ctest --test-dir build/default/cpp/Release -R '^test_vbd_' --output-on-failure
./build/default/cpp/Release/bin/bm_vbd_vertex_block_kernel --benchmark_min_time=0.05s --benchmark_filter='BM_Vbd'
./build/default/cpp/Release/bin/bm_vbd_block_descent --benchmark_min_time=0.05s --benchmark_filter='BM_Vbd'
./build/default/cpp/Release/bin/bm_vbd_neo_hookean --benchmark_min_time=0.05s --benchmark_filter='BM_Vbd'
```
