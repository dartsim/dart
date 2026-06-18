# PLAN-083 Shared Primitive Audit

This sidecar records the first current-state audit of PLAN-081 and PLAN-082
primitive ownership for the unified Newton-barrier solver family. It is not
implementation evidence by itself. It names the existing DART artifacts that
make consolidation possible, the pieces that must stay variant-local for now,
and the promotion order for future implementation slices.

## Evidence Snapshot

- Pure distance, barrier, tangent-stencil, and tangent-displacement friction
  kernels now live under
  `dart/simulation/detail/newton_barrier/`.
- Deformable compatibility headers under
  `dart/simulation/detail/deformable_contact/` forward to the
  shared Newton-barrier owner so active PLAN-081 code keeps compiling.
- Rigid IPC includes the shared Newton-barrier owner directly from
  `rigid_ipc_barrier.{hpp,cpp}` for primitive barriers and friction potentials.
- ABD includes the same shared owner from `affine_body_dynamics.{hpp,cpp}` for
  affine primitive barrier and friction chain-rule rows.
- `test_newton_barrier_primitives` covers old/new forwarding parity and rigid
  consumer parity for the promoted primitive layer.
- Deformable projected Newton currently uses
  `compute/deformable_psd_backend.*` for batched CPU/GPU PSD projection.
- Rigid IPC currently owns its reduced-coordinate chain rules, rigid
  curved-trajectory CCD, scene-level surface assembly, line search, projected
  Newton loop, friction assembly, adaptive stiffness, and opt-in runtime stage
  under `detail/rigid_ipc_*` and `compute/world_step_stage.cpp`.
- PLAN-081 and PLAN-082 already expose separate benchmark surfaces:
  `bm_ipc_distance_kernels`, `bm_ipc_candidate_set`,
  `bm_ipc_motion_aware_candidate_set`, `bm_ipc_continuous_collision_step`,
  `bm_ipc_barrier_kernel`, `bm_ipc_tangent_stencil`, and
  `bm_rigid_ipc_solver`.
- py-demos already has separate `IPC Deformable (sx)` and `Rigid IPC (sx)`
  categories; PLAN-083 mixed scenes must reuse those bridges where possible
  before adding new unified scenes.

## Promotion Policy

- Promote a primitive only after a second variant consumes it or the ABD slice
  would otherwise duplicate it. The distance/barrier/tangent-stencil and
  tangential-friction promotion already met that bar; future promotions should
  follow the same evidence shape.
- Keep public APIs DART-owned and backend-neutral. Internal paths may say
  Newton barrier, implicit barrier, rigid IPC, or IPC for provenance, but public
  user docs and bindings must not expose upstream project names as solver
  identities.
- Preserve existing PLAN-081 and PLAN-082 tests while adding cross-variant
  tests at the promoted layer. A move is not complete until the old variant
  behavior is still covered.
- Do not move scene/corpus ownership into PLAN-083. PLAN-081 keeps deformable
  IPC corpus rows; PLAN-082 keeps rigid IPC fixture rows; PLAN-083 owns shared
  primitives, ABD, mixed scenes, and cross-variant evidence.

## Current Primitive Map

| Area                                                                                | Current home                                                                                                                                           | Current reuse evidence                                                                                                                                                                  | PLAN-083 action                                                                                                                                                                                           | Required gates before promotion                                                                                                                                               |
| ----------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| PT/EE/PE/PP distance values, features, gradients, Hessians, and edge-edge mollifier | `detail/newton_barrier/primitive_distance.hpp`; `detail/deformable_contact/primitive_distance.hpp` forwards compatibility aliases.                     | Deformable contact reaches the shared owner through forwarding headers; rigid IPC and ABD consume the shared owner directly; `test_newton_barrier_primitives` covers forwarding parity. | Done for the pure world-primitive owner. Keep forwarding headers until active PLAN-081 branches no longer need them; route any new primitive-distance extension through this owner first.                 | Existing primitive-distance tests, `test_newton_barrier_primitives`, rigid IPC and ABD derivative/equivalence tests, and `bm_ipc_distance_kernels`.                           |
| C2 clamped-log scalar and primitive barriers                                        | `detail/newton_barrier/barrier_kernel.hpp`; `detail/deformable_contact/barrier_kernel.hpp` forwards compatibility aliases.                             | Deformable contact reaches the shared owner through forwarding headers; rigid IPC and ABD directly call shared primitive barriers.                                                      | Done for scalar and primitive barriers. Keep variant-specific chain rules outside the shared owner unless another variant proves the same mapping contract.                                               | Existing deformable barrier tests, rigid reduced-coordinate tests, ABD affine barrier derivative/equivalence tests, and benchmark smokes.                                     |
| Tangent stencils for PT, EE, PE, and PP friction                                    | `detail/newton_barrier/tangent_stencil.hpp`; `detail/deformable_contact/tangent_stencil.hpp` forwards compatibility aliases.                           | Rigid IPC and ABD share tangent stencils for world-coordinate lagged friction potentials.                                                                                               | Done for tangent-basis/stencil semantics. Keep normal-force estimation, lag refresh, and coordinate-specific chain rules in their variant owners.                                                         | Existing `test_tangent_stencil`, `test_newton_barrier_primitives`, rigid friction coverage, ABD affine friction derivative/equivalence tests, and `bm_ipc_tangent_stencil`.   |
| Candidate sets and sweep-and-prune traversal                                        | `detail/deformable_contact/candidate_set.hpp`; deformable solver scratch in `world_step_stage.cpp`                                                     | Currently deformable-local. Rigid IPC uses its own surface-pair assembly and fixture/runtime extraction.                                                                                | Do not promote yet. Define a shared `SurfaceSnapshot` / `PrimitiveCandidateSet` design first, then consume it from ABD or mixed rigid-deformable coupling.                                                | Deformable candidate tests and benchmarks plus a rigid or ABD consumer with equivalent activation-distance, incident-filter, and determinism coverage.                        |
| Endpoint-linear conservative CCD step bounds                                        | `detail/deformable_contact/continuous_collision_step.*` plus native collision primitive CCD                                                            | Deformable surface line search uses endpoint-linear PT/EE bounds. Rigid IPC needs curved rigid trajectories and owns separate interval/subdivision helpers.                             | Keep variant-local. Extract only shared result/stat shapes after both endpoint-linear and curved-trajectory backends can satisfy one abstract line-search contract.                                       | Existing deformable CCD tests, rigid curved CCD tests, and a contract test that proves both backends return conservative zero-step results for initial separation violations. |
| Rigid curved-trajectory CCD                                                         | `detail/rigid_ipc/rigid_ipc_ccd.*`                                                                                                                     | Rigid IPC-specific rotation-vector interpolation, residual equations, and interval/subdivision query semantics.                                                                         | Keep in PLAN-082 until ABD introduces affine/piecewise-linear body trajectories. Then generalize to a trajectory-adapter interface instead of moving rigid-only code.                                     | Rigid direct-row parity, codimensional coverage, and an ABD adapter test that recovers the rigid result when affine deformation is constrained to a rigid transform.          |
| PSD projection                                                                      | `compute/deformable_psd_backend.*` and CUDA sidecar                                                                                                    | Deformable projected Newton uses the pluggable CPU/CUDA PSD backend; rigid IPC currently uses a local dense reduced-coordinate PSD projection helper.                                   | Introduce an internal Newton-barrier PSD backend wrapper before ABD. Later route rigid reduced blocks through the shared backend when batch sizes justify it.                                             | CPU bit-parity against current deformable path, CUDA parity where available, rigid reduced-Hessian eigenvalue tests, and no GPU runtime dependency in the core library.       |
| Sparse projected Newton scaffold                                                    | Deformable solver logic in `compute/world_step_stage.cpp`; rigid loop in `detail/rigid_ipc/rigid_ipc_barrier.*`                                        | Both variants assemble PSD-projected Hessians and solve sparse systems, but DOF layouts, state updates, line search, and convergence stats differ.                                      | Do not merge loops yet. First define shared row/stat terminology and linear-solver hooks; merge only after ABD supplies a third consumer or a common state-adapter layer exists.                          | Variant tests for convergence, non-converged-result policy, deterministic solve, fallback behavior, and benchmark counters.                                                   |
| Lagged friction potentials and outer passes                                         | Shared tangent-displacement friction math in `detail/newton_barrier/friction_kernel.hpp`; deformable, rigid, and ABD assemblies remain variant-local.  | Rigid IPC and ABD consume the shared friction kernel; deformable/VBD-style rows still own their row assembly, normal-force lagging, and coordinate maps.                                | Done for the world-primitive tangential potential math. Keep normal-force estimation, outer refresh, coordinate mapping, and row scheduling variant-local until a second-use contract proves them shared. | Static/dynamic branch finite-difference tests, friction work diagnostics, deformable/rigid/affine sliding differentials, and CPU/GPU parity for the shared potential kernel.  |
| Adaptive barrier stiffness                                                          | Rigid IPC owns `initialRigidIpcBarrierStiffness` / `updateRigidIpcBarrierStiffness`; deformable plan still lists adaptive stiffness as remaining work. | Not yet shared.                                                                                                                                                                         | Keep in PLAN-082 until PLAN-081 implements adaptive stiffness or ABD needs it. PLAN-083 should then define one parameter contract matching `unb-table-01`.                                                | Falling-box kappa/dhat sweeps, closest-distance diagnostics, and no regression to rigid activated-contact behavior.                                                           |
| Diagnostics and benchmark JSON                                                      | Variant-specific solver stats and benchmarks                                                                                                           | Both variants report contacts, projected-Newton counters, friction diagnostics, and benchmark rows, but field names are not unified.                                                    | Add a PLAN-083 diagnostics schema before paper Table 2/Fig. 24 evidence. Keep existing benchmark binaries as sources.                                                                                     | Schema check, benchmark smoke, and migration that preserves existing dashboard history.                                                                                       |
| py-demos evidence                                                                   | `python/examples/demos` categories `IPC Deformable (sx)` and `Rigid IPC (sx)`                                                                          | Existing scenes prove separate category plumbing; unified paper scenes are still planned.                                                                                               | Reuse bridges and add PLAN-083 mixed-scene rows only when the underlying solver slice exists.                                                                                                             | Headless smoke, nonblank image check, motion-difference check, and benchmark/profiling packet for every promoted paper scene.                                                 |

## Next Implementation Order

1. **Preserve the completed primitive promotion.** Keep
   `detail/newton_barrier` as the internal owner for distance, barrier, tangent
   stencil, and tangent-displacement friction math. Keep deformable forwarding
   headers until active PLAN-081 branches no longer need them, and keep
   cross-variant primitive tests green whenever a variant changes the shared
   owner.
2. **Promote ABD benchmark evidence.** Move the first
   `bm_affine_body_dynamics` smoke rows into a comparison manifest row before
   claiming runtime solver progress.
3. **Generalize PSD backend naming.** Wrap the deformable PSD backend behind a
   Newton-barrier internal name while preserving the current CPU/CUDA behavior
   and no-runtime-CUDA policy. Do not route rigid IPC through it until batch
   packing and benchmark evidence justify the move.
4. **Define common line-search and diagnostics contracts.** Record shared
   result/stat structures for endpoint-linear, curved-rigid, and future affine
   trajectories, but keep the actual CCD backends variant-local.
5. **Promote solver contracts only after second use.** Shared projected-Newton,
   lagged-friction outer-loop, row diagnostics, and benchmark schemas should
   move only when ABD or another variant proves they are the same contract
   rather than similar-looking local loops.

## ABD First-Slice Dependencies

Before implementing ABD, PLAN-083 needs these concrete interfaces or decisions:

- `AffineBodyState`: 12-DOF state, velocity, mass/inertia proxy, orthogonality
  stiffness, and serialization/diagnostic fields.
- `AffineSurfaceAdapter`: maps local vertices to world positions and Jacobians
  for point, edge, and triangle stencils; the rigid adapter is the correctness
  oracle when the affine transform is constrained to rotation plus translation.
- `NewtonBarrierPrimitiveResult`: shared value/gradient/Hessian/tangent output
  consumed by deformable, rigid, and affine mappings. The first primitive and
  tangent-displacement friction owners exist; future result extensions should
  remain backward compatible with those cross-variant tests.
- `BarrierLineSearchContract`: common result/stat vocabulary for initial
  violation, conservative zero step, indeterminate CCD, limiting primitive, and
  accepted step scale.
- `StiffBodyBenchmarkPacket`: matched scene, timestep, body/triangle count,
  activation distance, friction, accuracy metric, CPU time, GPU time when
  available, and comparison rows against rigid IPC, Bullet when applicable, and
  paper/deck numbers.

## Open Risks

- Dropping compatibility forwarding too early would create churn in active
  PLAN-081/082 branches; future code moves should preserve old include paths
  until those branches land or are reconciled.
- Rigid IPC's curved-trajectory CCD and ABD's affine trajectories are related
  but not identical. A premature shared CCD abstraction could hide conservative
  guarantees that need separate proofs.
- The PSD backend has a proven deformable CUDA path, but rigid and ABD reduced
  blocks may not have large enough batches to justify GPU offload. The shared
  API must allow CPU fallback without weakening the full PLAN-083 GPU target.
- Benchmarks with `ipc` names are useful baseline history. Rename only after a
  dashboard migration can preserve trend continuity.
