# PLAN-083 Shared Primitive Audit

This sidecar records the first current-state audit of PLAN-081 and PLAN-082
primitive ownership for the unified Newton-barrier solver family. It is not
implementation evidence by itself. It names the existing DART artifacts that
make consolidation possible, the pieces that must stay variant-local for now,
and the promotion order for future implementation slices.

## Evidence Snapshot

- Deformable primitive kernels currently live under
  `dart/simulation/experimental/detail/deformable_contact/`.
- Rigid IPC already includes the deformable barrier kernel and reuses the
  deformable point/edge/triangle distance, barrier, and tangent-stencil
  functions from `rigid_ipc_barrier.cpp`.
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
  would otherwise duplicate it.
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

| Area                                                                                | Current home                                                                                                                                           | Current reuse evidence                                                                                                                                                  | PLAN-083 action                                                                                                                                                                  | Required gates before promotion                                                                                                                                               |
| ----------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| PT/EE/PE/PP distance values, features, gradients, Hessians, and edge-edge mollifier | `detail/deformable_contact/primitive_distance.hpp`                                                                                                     | Deformable contact kernels use it directly; rigid barrier code reaches it through the deformable barrier kernel and uses the same primitive ordering in tests.          | Promote first into an internal `detail/newton_barrier/primitive_distance.hpp` module with compatibility forwarding from `deformable_contact`.                                    | Existing `test_primitive_distance`, `test_barrier_kernel`, `test_rigid_ipc_barrier`, `bm_ipc_distance_kernels`, and a new cross-variant derivative parity test.               |
| C2 clamped-log scalar and primitive barriers                                        | `detail/deformable_contact/barrier_kernel.hpp`                                                                                                         | Rigid IPC directly aliases and calls `deformable_contact::pointTriangleBarrier`, `pointEdgeBarrier`, `edgeEdgeBarrier`, `pointPointBarrier`, and `c2ClampedLogBarrier`. | Promote with the distance kernels; this is already second-use code whose namespace/file ownership is misleading.                                                                 | Existing deformable barrier tests and rigid reduced-coordinate tests must remain green; benchmark names may keep `ipc` aliases until baseline dashboards are migrated.        |
| Tangent stencils for PT, EE, PE, and PP friction                                    | `detail/deformable_contact/tangent_stencil.hpp`                                                                                                        | Rigid IPC friction uses all four deformable tangent-stencil helpers for world-coordinate lagged friction potentials.                                                    | Promote with the barrier primitives or immediately after, because friction parity depends on identical tangent semantics.                                                        | Existing `test_tangent_stencil`, rigid friction finite-difference coverage, and `bm_ipc_tangent_stencil`; add a shared static/dynamic slip threshold test.                    |
| Candidate sets and sweep-and-prune traversal                                        | `detail/deformable_contact/candidate_set.hpp`; deformable solver scratch in `world_step_stage.cpp`                                                     | Currently deformable-local. Rigid IPC uses its own surface-pair assembly and fixture/runtime extraction.                                                                | Do not promote yet. Define a shared `SurfaceSnapshot` / `PrimitiveCandidateSet` design first, then consume it from ABD or mixed rigid-deformable coupling.                       | Deformable candidate tests and benchmarks plus a rigid or ABD consumer with equivalent activation-distance, incident-filter, and determinism coverage.                        |
| Endpoint-linear conservative CCD step bounds                                        | `detail/deformable_contact/continuous_collision_step.*` plus native collision primitive CCD                                                            | Deformable surface line search uses endpoint-linear PT/EE bounds. Rigid IPC needs curved rigid trajectories and owns separate interval/subdivision helpers.             | Keep variant-local. Extract only shared result/stat shapes after both endpoint-linear and curved-trajectory backends can satisfy one abstract line-search contract.              | Existing deformable CCD tests, rigid curved CCD tests, and a contract test that proves both backends return conservative zero-step results for initial separation violations. |
| Rigid curved-trajectory CCD                                                         | `detail/rigid_ipc_ccd.*`                                                                                                                               | Rigid IPC-specific rotation-vector interpolation, residual equations, and interval/subdivision query semantics.                                                         | Keep in PLAN-082 until ABD introduces affine/piecewise-linear body trajectories. Then generalize to a trajectory-adapter interface instead of moving rigid-only code.            | Rigid direct-row parity, codimensional coverage, and an ABD adapter test that recovers the rigid result when affine deformation is constrained to a rigid transform.          |
| PSD projection                                                                      | `compute/deformable_psd_backend.*` and CUDA sidecar                                                                                                    | Deformable projected Newton uses the pluggable CPU/CUDA PSD backend; rigid IPC currently uses a local dense reduced-coordinate PSD projection helper.                   | Introduce an internal Newton-barrier PSD backend wrapper before ABD. Later route rigid reduced blocks through the shared backend when batch sizes justify it.                    | CPU bit-parity against current deformable path, CUDA parity where available, rigid reduced-Hessian eigenvalue tests, and no GPU runtime dependency in the core library.       |
| Sparse projected Newton scaffold                                                    | Deformable solver logic in `compute/world_step_stage.cpp`; rigid loop in `detail/rigid_ipc_barrier.*`                                                  | Both variants assemble PSD-projected Hessians and solve sparse systems, but DOF layouts, state updates, line search, and convergence stats differ.                      | Do not merge loops yet. First define shared row/stat terminology and linear-solver hooks; merge only after ABD supplies a third consumer or a common state-adapter layer exists. | Variant tests for convergence, non-converged-result policy, deterministic solve, fallback behavior, and benchmark counters.                                                   |
| Lagged friction potentials and outer passes                                         | Deformable assembly in `compute/world_step_stage.cpp`; rigid potentials and reduced assembly in `detail/rigid_ipc_barrier.*`                           | Both variants use lagged smoothed Coulomb friction and tangent operators, but normal-force lagging and coordinate mapping differ.                                       | Promote only world-primitive tangential potential math after tangent stencils move. Keep normal-force estimation, outer refresh, and reduced-coordinate mapping variant-local.   | Static/dynamic branch finite-difference tests, friction work diagnostics, deformation and rigid sliding differentials, and CPU/GPU parity for the shared potential kernel.    |
| Adaptive barrier stiffness                                                          | Rigid IPC owns `initialRigidIpcBarrierStiffness` / `updateRigidIpcBarrierStiffness`; deformable plan still lists adaptive stiffness as remaining work. | Not yet shared.                                                                                                                                                         | Keep in PLAN-082 until PLAN-081 implements adaptive stiffness or ABD needs it. PLAN-083 should then define one parameter contract matching `unb-table-01`.                       | Falling-box kappa/dhat sweeps, closest-distance diagnostics, and no regression to rigid activated-contact behavior.                                                           |
| Diagnostics and benchmark JSON                                                      | Variant-specific solver stats and benchmarks                                                                                                           | Both variants report contacts, projected-Newton counters, friction diagnostics, and benchmark rows, but field names are not unified.                                    | Add a PLAN-083 diagnostics schema before paper Table 2/Fig. 24 evidence. Keep existing benchmark binaries as sources.                                                            | Schema check, benchmark smoke, and migration that preserves existing dashboard history.                                                                                       |
| py-demos evidence                                                                   | `python/examples/demos` categories `IPC Deformable (sx)` and `Rigid IPC (sx)`                                                                          | Existing scenes prove separate category plumbing; unified paper scenes are still planned.                                                                               | Reuse bridges and add PLAN-083 mixed-scene rows only when the underlying solver slice exists.                                                                                    | Headless smoke, nonblank image check, motion-difference check, and benchmark/profiling packet for every promoted paper scene.                                                 |

## First Implementation Order

1. **Promote pure world-primitive math.** Move distance, barrier, and tangent
   stencil kernels into an internal Newton-barrier namespace/path with
   compatibility forwarding from `deformable_contact`. This is the lowest-risk
   consolidation because rigid IPC already consumes these functions. The
   concrete first-slice design lives in
   [`primitive-promotion-slice.md`](primitive-promotion-slice.md).
2. **Add cross-variant primitive tests.** Keep existing PLAN-081 and PLAN-082
   tests, then add shared derivative, PSD, tangent, and friction-potential
   checks that exercise deformable and rigid callers through the same primitive
   results.
3. **Generalize PSD backend naming.** Wrap the deformable PSD backend behind a
   Newton-barrier internal name while preserving the current CPU/CUDA behavior
   and no-runtime-CUDA policy. Do not route rigid IPC through it until batch
   packing and benchmark evidence justify the move.
4. **Define common line-search and diagnostics contracts.** Record shared
   result/stat structures for endpoint-linear, curved-rigid, and future affine
   trajectories, but keep the actual CCD backends variant-local.
5. **Draft the ABD first slice on top of promoted primitives.** The first ABD
   design should consume the shared distance/barrier/tangent layer, add affine
   body state and chain-rule assembly, and compare against rigid IPC on matched
   primitive and two-body fixtures before any paper-scale benchmark claim. The
   initial ABD design lives in
   [`abd-first-slice-design.md`](abd-first-slice-design.md).

## ABD First-Slice Dependencies

Before implementing ABD, PLAN-083 needs these concrete interfaces or decisions:

- `AffineBodyState`: 12-DOF state, velocity, mass/inertia proxy, orthogonality
  stiffness, and serialization/diagnostic fields.
- `AffineSurfaceAdapter`: maps local vertices to world positions and Jacobians
  for point, edge, and triangle stencils; the rigid adapter is the correctness
  oracle when the affine transform is constrained to rotation plus translation.
- `NewtonBarrierPrimitiveResult`: shared value/gradient/Hessian/tangent output
  consumed by deformable, rigid, and affine mappings.
- `BarrierLineSearchContract`: common result/stat vocabulary for initial
  violation, conservative zero step, indeterminate CCD, limiting primitive, and
  accepted step scale.
- `StiffBodyBenchmarkPacket`: matched scene, timestep, body/triangle count,
  activation distance, friction, accuracy metric, CPU time, GPU time when
  available, and comparison rows against rigid IPC, Bullet when applicable, and
  paper/deck numbers.

## Open Risks

- Moving file ownership before compatibility forwarding would create churn in
  active PLAN-081/082 branches; future code moves should preserve old include
  paths until those branches land or are reconciled.
- Rigid IPC's curved-trajectory CCD and ABD's affine trajectories are related
  but not identical. A premature shared CCD abstraction could hide conservative
  guarantees that need separate proofs.
- The PSD backend has a proven deformable CUDA path, but rigid and ABD reduced
  blocks may not have large enough batches to justify GPU offload. The shared
  API must allow CPU fallback without weakening the full PLAN-083 GPU target.
- Benchmarks with `ipc` names are useful baseline history. Rename only after a
  dashboard migration can preserve trend continuity.
