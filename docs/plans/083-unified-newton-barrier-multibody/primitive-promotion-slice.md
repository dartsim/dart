# PLAN-083 Primitive Promotion Slice

This sidecar records the implemented first consolidation slice for the unified
Newton-barrier family. It remains as the contract checklist for preserving the
DART-owned internal Newton-barrier layer while PLAN-081, PLAN-082, and PLAN-083
continue to evolve.

## Outcome

DART has one internal source of truth for point/edge/triangle distance,
barrier, and tangent-stencil math used by deformable IPC, rigid IPC, and the
planned affine-body track. Existing deformable include paths and tests continue
to work during active branch reconciliation.

## Current Evidence

| Evidence            | Current state                                                                                                                                                                                                       |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Distance kernels    | `dart/simulation/detail/newton_barrier/primitive_distance.hpp` owns PT/EE/PE/PP squared distances, features, gradients, Hessians, and the edge-edge mollifier; `deformable_contact` forwards compatibility aliases. |
| Barrier kernels     | `dart/simulation/detail/newton_barrier/barrier_kernel.hpp` owns the C2 clamped-log scalar barrier and primitive barrier derivatives; `deformable_contact` forwards compatibility aliases.                           |
| Tangent stencils    | `dart/simulation/detail/newton_barrier/tangent_stencil.hpp` owns PT/EE/PE/PP tangent bases, projections, and metrics for friction; `deformable_contact` forwards compatibility aliases.                             |
| Rigid/ABD reuse     | `dart/simulation/detail/rigid_ipc/rigid_ipc_barrier.{hpp,cpp}` and `dart/simulation/detail/affine_body_dynamics.{hpp,cpp}` consume the Newton-barrier owner directly for shared barrier/friction primitives.        |
| PSD backend         | `dart/simulation/compute/deformable_psd_backend.*` is already a CPU/CUDA-pluggable block PSD projector, but its name is deformable-specific.                                                                        |
| Existing tests      | `test_primitive_distance`, `test_barrier_kernel`, `test_tangent_stencil`, `test_newton_barrier_primitives`, `test_rigid_ipc_barrier`, `test_affine_body_dynamics`, and CUDA PSD coverage cover the active pieces.   |
| Existing benchmarks | `bm_ipc_distance_kernels`, `bm_ipc_barrier_kernel`, `bm_ipc_tangent_stencil`, `bm_rigid_ipc_solver`, and `bm_affine_body_dynamics` are the baseline timing surfaces.                                                |

## Implemented Internal Shape

Header-only primitive owners live under:

```text
dart/simulation/detail/newton_barrier/
├── primitive_distance.hpp
├── barrier_kernel.hpp
├── friction_kernel.hpp
└── tangent_stencil.hpp
```

Use namespace:

```cpp
dart::simulation::detail::newton_barrier
```

Compatibility forwarding remains under:

```text
dart/simulation/detail/deformable_contact/
├── primitive_distance.hpp
├── barrier_kernel.hpp
└── tangent_stencil.hpp
```

The forwarding headers include the new owner headers and provide `using`
declarations or aliases for the existing `deformable_contact` names. This keeps
active PLAN-081 code and branch-local includes compiling while making the
Newton-barrier owner explicit for PLAN-082 and PLAN-083 work.

## Slice Scope

| Step | Change                                                                                                                                 | Required invariant                                                                                                             |
| ---- | -------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| 1    | Move distance types/functions into `detail/newton_barrier/primitive_distance.hpp`.                                                     | Old `deformable_contact` names compile and produce bit-identical values, gradients, Hessians, features, and mollifier outputs. |
| 2    | Move scalar and primitive barriers into `detail/newton_barrier/barrier_kernel.hpp`.                                                    | Old and new calls return identical active flags, safe distances, values, gradients, Hessians, and mollifier metadata.          |
| 3    | Move tangent-stencil helpers into `detail/newton_barrier/tangent_stencil.hpp`.                                                         | Old and new calls return identical bases, coordinates, projections, metrics, and fallback flags.                               |
| 4    | Update rigid IPC primitive calls to include and use the Newton-barrier owner directly.                                                 | `test_rigid_ipc_barrier` remains green and no reduced-coordinate result changes.                                               |
| 5    | Leave deformable solver includes on compatibility headers unless the active branch state is clean enough for mechanical include churn. | PLAN-081 remains insulated from unnecessary merge conflicts.                                                                   |
| 6    | Keep existing benchmark binary names for continuity.                                                                                   | Dashboard/performance history is not reset by a namespace move.                                                                |

## Cross-Variant Test Additions

Add a focused test file under the existing DART 7 simulation contact test
directory, for example:

```text
tests/unit/simulation/contact/test_newton_barrier_primitives.cpp
```

Minimum rows:

| Test row              | Purpose                                                                                                                                                            |
| --------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Include compatibility | Include both old and new headers in one TU and prove aliases resolve without ambiguous overloads.                                                                  |
| Distance alias parity | Compare old and new PT, EE, PE, PP values, gradients, Hessians, features, closest points, and mollifier terms on representative regular and degenerate cases.      |
| Barrier alias parity  | Compare old and new scalar/primitive barrier outputs for inactive, active, near-floor, nonfinite, and zero-stiffness inputs.                                       |
| Tangent alias parity  | Compare old and new PT, EE, PE, PP tangent stencils for regular and fallback-basis cases.                                                                          |
| Rigid consumer parity | Evaluate one rigid reduced barrier and one rigid friction potential through the updated rigid IPC code and compare against the pre-move primitive result contract. |
| Namespace boundary    | Assert no public headers or dartpy bindings include `detail/newton_barrier`; the layer remains internal.                                                           |

Existing tests remain mandatory; the new test is additive.

## Benchmark Handling

Do not rename benchmark binaries in the first code PR. Keep:

- `bm_ipc_distance_kernels`
- `bm_ipc_barrier_kernel`
- `bm_ipc_tangent_stencil`
- `bm_rigid_ipc_solver`

The first promotion PR should record smoke output from the existing binaries to
prove no obvious performance regression. A later dashboard migration may add
`bm_newton_barrier_*` aliases, but only if it preserves historical trend
continuity.

## Validation Gate For The Code Slice

The implementation PR derived from this plan should run:

```bash
pixi run lint
pixi run build-simulation-tests
pixi run test-simulation-quick
pixi run check-api-boundaries
pixi run bm bm_ipc_distance_kernels -- --benchmark_min_time=0.05s
pixi run bm bm_ipc_barrier_kernel -- --benchmark_min_time=0.05s
pixi run bm bm_ipc_tangent_stencil -- --benchmark_min_time=0.05s
pixi run bm bm_rigid_ipc_solver -- --benchmark_min_time=0.05s
```

CUDA-specific evidence is required only if the PSD wrapper is touched:

```bash
pixi run -e cuda test-cuda
```

## Non-Goals

- No public API changes.
- No dartpy binding changes.
- No scene, fixture, or py-demos migration.
- No rigid curved-trajectory CCD move.
- No sparse Newton loop merge.
- No rigid IPC default behavior change.
- No ABD runtime stage.

## Done Means

- The Newton-barrier primitive owner exists and is used by rigid IPC.
- Old deformable-contact include paths still compile and test.
- Cross-variant primitive tests cover old/new alias parity and one rigid
  consumer path.
- Existing deformable, rigid IPC, benchmark, API-boundary, and docs gates stay
  green.
