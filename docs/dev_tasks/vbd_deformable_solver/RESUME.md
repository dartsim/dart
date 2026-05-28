# Resume: Vertex Block Descent Solver

## Last Session Summary

Started PLAN-082: implementing Vertex Block Descent (VBD, Chen et al. SIGGRAPH 2024) as a DART-owned deformable solver in the experimental World. Landed
Phases 0-4 with all gates green (lint clean, focused build, 39 tests across
5 binaries, benchmark smoke):

- Phase 0 grounding: PLAN-082 dashboard entry and numbered plan file, the
  `chen-2024-vbd` references catalog entry plus comparative `tinyvbd`/`gaia`
  entries, the VBD paper/reference gap audit (verified formulas + Table 1
  targets), and this tracker.
- Phase 1 `detail/deformable_vbd/vertex_block_kernel.hpp`: per-vertex inertia +
  mass-spring force/Hessian, opt-in PSD clamp, regularized SPD 3x3 block solve.
- Phase 2 `detail/deformable_vbd/vertex_coloring.hpp`: element-induced
  adjacency + Welsh-Powell greedy coloring + conflict-free verifier.
- Phase 3 `detail/deformable_vbd/block_descent.hpp`: colored Gauss-Seidel
  mass-spring driver, validated to converge to an independent gradient-descent
  minimizer of the same objective.
- Phase 4 `detail/deformable_vbd/neo_hookean.hpp` + `block_descent.hpp`: Stable
  Neo-Hookean tetrahedral energy, analytic Piola stress, per-vertex force +
  exact 3x3 Hessian (FD-verified, inversion-stable), and a `blockDescentTetMesh`
  volumetric driver (tet adjacency + tet coloring) validated to converge to an
  independent global gradient-descent minimizer of the tet objective.

Key grounding fact: VBD minimizes the **same** variational implicit-Euler
objective the existing experimental deformable solver already minimizes with a
global mass-preconditioned gradient descent (see `evaluateDeformableObjective`
in `dart/simulation/experimental/compute/world_step_stage.cpp`). VBD instead
uses per-vertex 3x3 block coordinate descent with graph-colored parallel
Gauss-Seidel sweeps. So VBD is an alternative solver on the existing deformable
ECS components, not a new data model.

## Current Branch

`feature/vbd-solver-foundation` — Phases 0-4 landed. Local commits only;
pushes/PRs require explicit approval.

## Immediate Next Step

Phases 0-4 are complete (mass-spring and tetrahedral Neo-Hookean block descent
both converge to independent reference minimizers). Next:

- Phase 5: adaptive inertial/previous-step initialization
  (`a_tilde = clamp(a_prev . g_hat, 0, ||a_ext||)`,
  `x_init = x^t + h v^t + h^2 g_hat a_tilde`), optional Chebyshev acceleration
  (`omega_1=1, omega_2=2/(2-rho^2), omega_n=4/(4-rho^2 omega_{n-1})`,
  `x^n = omega_n (x_bar^n - x^{n-2}) + x^{n-2}`), and Rayleigh damping
  (`H += (k_d/h) d2E/dx^2`, force `-= (k_d/h) d2E/dx^2 (x - x^t)`).

## Context That Would Be Lost

- The existing deformable solver objective is the VBD objective. Reuse
  `comps::DeformableNodeState` (positions/previousPositions/velocities/masses/
  fixed), `DeformableSpringModel` (edges + stiffness + damping),
  `DeformableMeshTopology` (restPositions/surfaceTriangles/tetrahedra/
  tetrahedronRestVolumes), and `DeformableMaterial` (density/youngsModulus/
  poissonRatio).
- The inertial target setup is in `world_step_stage.cpp` around line 2352:
  `y_i = x_i + h*dampingScale*v_i + g*h^2 + h^2*extAccel_i`, with
  `dampingScale = 1/(1 + damping*h)`.
- The spring energy and its analytic gradient already exist in
  `evaluateDeformableObjective`; the matching per-vertex spring Hessian is
  `H_aa = k[ n n^T + (1 - L/l)(I - n n^T) ]` with the transverse factor clamped
  to be non-negative for the PD local Hessian.
- Internal VBD kernels live under `detail/deformable_vbd/`; tests go in
  `tests/unit/simulation/experimental/deformable_vbd/` (auto-globbed as
  `test_*.cpp`; the dir is registered in that tree's `CMakeLists.txt` via
  `dart_experimental_add_unit_test_dir`) and benchmarks in
  `tests/benchmark/simulation/experimental/` (registered in that dir's
  `CMakeLists.txt` via `dart_experimental_add_benchmark`).
- The references catalog is `docs/readthedocs/papers.md` (edit via the
  `dart-references` workflow); the old `docs/design/simulation_experimental_references.md`
  is only a compatibility pointer.

## How To Resume

```bash
git checkout feature/vbd-solver-foundation
git status && git log -8 --oneline
pixi run cmake build/default/cpp/Release
pixi run cmake --build build/default/cpp/Release --target \
  test_vbd_vertex_block_kernel test_vbd_vertex_coloring test_vbd_block_descent \
  test_vbd_neo_hookean test_vbd_tet_mesh_descent bm_vbd_vertex_block_kernel \
  bm_vbd_block_descent bm_vbd_neo_hookean
ctest --test-dir build/default/cpp/Release -R '^test_vbd_' --output-on-failure
./build/default/cpp/Release/bin/bm_vbd_block_descent --benchmark_min_time=0.05s --benchmark_filter='BM_Vbd'
./build/default/cpp/Release/bin/bm_vbd_neo_hookean --benchmark_min_time=0.05s --benchmark_filter='BM_Vbd'
```
