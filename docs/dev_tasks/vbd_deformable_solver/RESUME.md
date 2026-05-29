# Resume: Vertex Block Descent Solver

## Last Session Summary

Started PLAN-082: implementing Vertex Block Descent (VBD, Chen et al. SIGGRAPH 2024) as a DART-owned deformable solver in the experimental World. Landed
Phases 0-4, the Phase 5 acceleration/damping primitives, Phase 6 (stepping loop
and World solver wiring), and Phase 8a (CPU baseline comparison plus residual
early termination) with all gates green: lint clean, focused build, 61 VBD tests
across 8 binaries plus the 43-case deformable regression suite, and benchmark
smoke showing VBD roughly 2-4x faster than the in-repo gradient-descent baseline
on contact-free mass-spring scenes.

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
- Phase 5 (primitives) `detail/deformable_vbd/acceleration.hpp`:
  `adaptiveInitialPosition`, `chebyshevOmega`/`applyChebyshev`, and
  `addRayleighDamping`, each unit-tested.
- Phase 6 (stepper) `detail/deformable_vbd/stepper.hpp`: `vbdStepMassSpring`
  performs one implicit-Euler step (inertial targets, adaptive warm start,
  colored sweeps + optional Chebyshev, velocity update), validated against the
  implicit-Euler free-fall trajectory and for multi-step stability.
- Phase 6 (World wiring) `comps::DeformableVbdConfig` (opt-in, non-public) +
  the VBD branch in `advanceDeformableBody` (world_step_stage.cpp): VBD now runs
  inside `DeformableDynamicsStage` for contact-free mass-spring bodies.
  `test_vbd_world_solver` confirms it matches the default solver, with no
  regression in `test_deformable_body` (43 cases).
- Phase 8a (CPU perf): residual early termination
  (`BlockDescentOptions::convergenceDisplacement` /
  `DeformableVbdConfig::convergenceDisplacement`) + `bm_vbd_world_solver`. With
  early termination, single-threaded VBD is ~2-4x faster per step than the
  in-repo gradient-descent baseline on the spring-grid benchmark. Also added
  `BM_VbdTinyStrandStep` matching the upstream TinyVBD default scene: DART VBD
  ~0.21 ms/step vs the built-and-run TinyVBD reference ~0.47 ms/frame (~2.2x
  faster, single CPU thread, compute-only).
- Phase 9 (GPU) `compute/cuda/vbd_block_descent_cuda.cu/.cuh`: a CUDA per-color
  mass-spring block-update kernel (analytic SPD 3x3 solve, single-stream
  Gauss-Seidel) in the `-cuda` target. `test_vbd_block_descent_cuda` (skips
  without a device; ran on the local RTX 5000 Ada) matches the CPU solve to
  1e-6; `bm_vbd_cuda` shows the GPU ~9x faster at 4k verts and ~26x at 16k
  verts. Build/run with `pixi run -e cuda` against `build/cuda/cpp/Release`.

Key grounding fact: VBD minimizes the **same** variational implicit-Euler
objective the existing experimental deformable solver already minimizes with a
global mass-preconditioned gradient descent (see `evaluateDeformableObjective`
in `dart/simulation/experimental/compute/world_step_stage.cpp`). VBD instead
uses per-vertex 3x3 block coordinate descent with graph-colored parallel
Gauss-Seidel sweeps. So VBD is an alternative solver on the existing deformable
ECS components, not a new data model.

## Current Branch

`feature/vbd-solver-foundation` — Phases 0-6, Phase 8a, and the Phase 9 CUDA
mass-spring kernel landed. Local commits only; pushes/PRs require explicit
approval.

## Immediate Next Step

Phases 0-4 are complete (mass-spring and tetrahedral Neo-Hookean block descent
both converge to independent reference minimizers), and the Phase 5
acceleration/damping primitives exist as tested standalone helpers.

VBD now runs as an opt-in inner solver inside `DeformableDynamicsStage` for
contact-free mass-spring bodies (`comps::DeformableVbdConfig` selects it), and
matches the default solver. Remaining work, in order:

1. Extend the World VBD path to tetrahedral bodies (the stage would fetch
   `comps::DeformableMaterial` for Lame params and reuse `blockDescentTetMesh`
   /the combined springs+tets assembly), and thread Chebyshev + element damping
   into the World path. Then decide the public solver-selection surface.
2. Phase 7: half-space penalty contact + semi-implicit Coulomb friction land
   (`contact_kernel.hpp`: `addHalfSpacePenaltyContact`, `addHalfSpaceFriction`;
   drivers `blockDescentMassSpringGround[Friction]`; bodies rest on the ground,
   sliding particles are stopped by friction). Next: vertex-triangle / edge-edge
   penalty contact (reuse `deformable_contact` distance/CCD kernels),
   self-collision, and wiring contact/friction into the World VBD path.
3. Phase 8: residual early termination + multithreaded color sweeps
   (`parallelBlockDescentMassSpring`, std::barrier, ~3.5x at 8 threads) both
   land. Next: SoA layout, wire the parallel driver into the World stage, and
   benchmark vs the external Gaia CPU numbers.
4. Phase 9: the CUDA mass-spring kernel, the device-resident rollout
   (`vbdRolloutMassSpringCuda`, ~45x faster than single-threaded CPU at 16k
   verts), and the tetrahedral Neo-Hookean GPU kernel (`vbdStepTetMeshCuda`,
   verified vs CPU, ~4.4x faster than CPU at ~2k verts) all land. Next:
   CUDA-graph capture, a device-resident tet rollout, float/mixed precision, and
   the RTX-4090 same-GPU reproduction plan in the dev-task README (build on a
   4090, run paper scenes + Gaia, compare to Table 1).
5. Phase 10: reproduce the paper scenes as DART examples/tests/benchmarks with
   profiling JSON and headless Filament visual evidence; the TinyVBD tilted
   strand (20 verts, stiffness 1e8, mass ratio 1:1000) is the first
   correctness/perf comparison target.

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
  test_vbd_neo_hookean test_vbd_tet_mesh_descent test_vbd_acceleration \
  test_vbd_stepper test_vbd_world_solver test_deformable_body \
  bm_vbd_vertex_block_kernel bm_vbd_block_descent bm_vbd_neo_hookean
ctest --test-dir build/default/cpp/Release -R '^test_vbd_|^test_deformable_body$' --output-on-failure
./build/default/cpp/Release/bin/bm_vbd_block_descent --benchmark_min_time=0.05s --benchmark_filter='BM_Vbd'
./build/default/cpp/Release/bin/bm_vbd_neo_hookean --benchmark_min_time=0.05s --benchmark_filter='BM_Vbd'
```
