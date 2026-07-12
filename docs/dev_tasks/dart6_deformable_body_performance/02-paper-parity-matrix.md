# Paper parity matrix

This file converts the requested papers into tracked DART 6 acceptance targets.
The paper models and published numbers are also preserved in the durable
reference `docs/background/deformable_body_paper_targets.md`; approved DART 6
scope decisions live in `docs/design/dart6_deformable_body.md`.

Sources:

- Kim and Pollard project page:
  <https://www.cs.cmu.edu/~junggon/projects/fastsimuldbody/fastsimuldbody.htm>
- Jain and Liu author PDF:
  <https://sites.cc.gatech.edu/graphics/projects/Sumit/homepage/papers/sigasia11/jain_softcontacts_siga11.pdf>
- Jain and Liu ACM DOI:
  <https://dl.acm.org/doi/10.1145/2024156.2024197>

## Kim and Pollard 2011

The paper target is a skeleton-driven deformable character system with a
reduced nonlinear FEM body, embedded fine surface mesh, linear-time skeleton
dynamics, explicit or symplectic Euler integration, and one-way or two-way
coupling. The project page states that the representative characters were
real-time or near real-time, with optional GPU acceleration for complicated
characters. GPU is recorded here as reference evidence only; DART 6
implementation work remains CPU-first.

| Representative target | Paper number or feature to replicate | Current DART 6 evidence | Acceptance gate |
| --- | --- | --- | --- |
| Passive character jiggle | Fatman one-way kinematic-skeleton simulation: 4,887 total DOFs, 60 skeleton DOFs, 2,121 volumetric nodes, 8,619 volumetric elements, 34,362 surface nodes, 0.5 ms step size, 1.33 s compute per 1 s simulation on GPU; same scene was 13.24 s per 1 s simulation on CPU. | No tracked DART 6 equivalent. Existing `soft_bodies` is much smaller, surface point-mass based, and not skeleton-driven at this scale. | Add or explicitly defer a one-way skeleton-driven soft character scene with point/surface counts, timestep, deterministic checksum, and same-host CPU rows for 1 and host-capped multi-thread runs. Owner: WP-DB.03, WP-DB.06, WP-DB.09. |
| Self-propelled starfish | Two-way coupled skeleton, deformable body, and environment interaction. Jump-turning row: 260 total DOFs, 26 skeleton DOFs, 78 volumetric nodes, 200 volumetric elements, 1,162 surface nodes, 1.0 ms step size, 0.49 s compute per 1 s CPU simulation. Escaping-obstacle row: 0.94 s compute per 1 s CPU simulation, with more than 40% of time in simple point-triangle collision checking. | No volumetric FEM backend or equivalent obstacle scene. Current native soft collision work only covers DART 6 point-mass `SoftMeshShape` contact lanes. | Add or defer a two-way soft character/obstacle benchmark with collision-time breakdown. Native collision must match or beat FCL on the same scene before it is marked preferred. Owner: WP-DB.04, WP-DB.08, WP-DB.09. |
| Fish jump | Internally actuated skeleton-driven deformable body with gravity and contact only during the freefall/landing phase. Paper row: 258 total DOFs, 9 skeleton DOFs, 107 volumetric nodes, 415 volumetric elements, 958 surface nodes, 1.0 ms step size, 0.50 s compute per 1 s CPU simulation. | No equivalent DART 6 scene or controller. | Add or defer an actuated soft-chain/soft-character landing scene with contact count, final-state checksum, and 1-thread/multi-thread CPU rows. Owner: WP-DB.08, WP-DB.09. |
| Worm roll | Internally actuated rolling worm with large deformation near skeleton joints and ground contact. Paper row: 543 total DOFs, 9 skeleton DOFs, 224 volumetric nodes, 714 volumetric elements, 262 surface nodes, 1.0 ms step size, 0.57 s compute per 1 s CPU simulation. | **Representative scene landed (WP-DB.09)**: `examples/soft_worm` is an internally actuated five-link chain carrying point-mass flesh (not volumetric FEM) that crawls 1.64 m over 3000 deterministic headless steps through soft ground contact, with a position checksum and finite-state output. Paper-scale volumetric deformation remains deferred per `decisions.md`. | Representative bar met via `soft_worm` (locomotion, checksum, finite state, capture evidence in `06-pr-evidence.md`); volumetric parity explicitly deferred. Owner: WP-DB.08, WP-DB.09. |
| Obstacle escape | Contact-heavy two-way starfish scene where collision checking dominates runtime on the paper implementation. | No soft-body contact benchmark with paper-scale contact metrics or obstacle interaction. | Add a contact-heavy native/FCL comparison that records contacts, collision profiler rows, and deterministic checksums; native must be faster without unsupported-pair warnings. Owner: WP-DB.05, WP-DB.07, WP-DB.08. |
| CPU/GPU scaling | Paper reports CPU near-real-time for the smaller characters and GPU acceleration for Fatman; selective diagonalization speedups ranged from about 1.16x to 3.60x depending on mesh and SVD implementation. | DART 6 stays CPU-only. The branch carries scalar/cache slices, allocation gates, and soft-phase/collision profiler scopes. Retained SoA scratch was implemented and **rejected on measurement** (mirror copies cost more than the pointer-chasing they removed) and contiguous point-mass storage is parked with evidence, so no point-mass SIMD kernel landed; both dispositions are recorded in `04-data-layout-and-memory-hardening.md`. Multi-thread rows for every representative scene come from the benchmark matrix (threads 1 and 16). | CPU-only maintained; SoA/SIMD line closed with measured negative results rather than a kernel; matrix rows cover 1 and 16 threads per scene. Owner: WP-DB.06, WP-DB.07. |

## Jain and Liu 2011

The paper target is closer to DART 6 `SoftBodyNode`: surface point masses
attached to rigid bodies, vertex and edge springs, adaptive active vertices near
contact, and mixed LCP/friction soft contact.

| Representative target | Paper number or feature to replicate | Current DART 6 evidence | Acceptance gate |
| --- | --- | --- | --- |
| Adaptive active vertices | Only vertices in a contact-local p-ring are simulated until they leave contact and return to rest; inactive point masses still contribute their rest-pose mass and inertia to the parent rigid body. | **Implemented (WP-DB.05)**: opt-in per-soft-body activation with constraint-time seeding, ring expansion, rest-gated linger, and rigid-lump frozen points contributing full rest mass/inertia to the parent body. Deterministic active-DOF counts (`getNumActivePointMasses`), thread-invariant hashes, activation-enabled zero-allocation gates, and contact-behavior tolerance tests landed in `578ea17a049` + `2382b971244`; the `adaptive_soft_contact` demo visualizes the active region. | Met: contact-local activation with deterministic active-DOF counts, contact behavior within tolerance (`adaptiveContactActivationMatchesSoftDropBehavior`, CoP gate with activation on), and no post-warmup heap growth. Owner: WP-DB.05, WP-DB.06. |
| Biped push recovery | Same SIMBICON-style controller with rigid contact vs soft contact; soft contact withstands larger perturbations and motor noise while maintaining more ground contact points. | No tracked soft-foot locomotion regression. Existing Atlas SIMBICON demo is rigid. | Add or defer a soft-foot locomotion regression with rigid-vs-soft push thresholds, contact count time series, and finite-state gate. Owner: WP-DB.03, WP-DB.08, WP-DB.09. |
| Noisy-floor biped | Floor segmented into 5 x 5 cm tiles; vertices receive random offsets uniformly sampled from 0-2 cm in vertical and horizontal positions; soft character stays on course in cases where rigid character wanders or falls. | No tracked noisy-floor soft contact scene. | Add deterministic seeded noisy-floor scene with rigid-vs-soft outcome, contact count, CoP trace, and 1-thread/multi-thread rows. Owner: WP-DB.08, WP-DB.09. |
| Center of pressure and force variance | Soft contact produces smoother contact transitions and lower force variation; pinch-grasp figure shows soft fingers maintaining many contacts while rigid contact fluctuates. | **Instrumented and gated (WP-DB.02)**: `test_SoftDynamics` now bounds total vertical contact force around system weight and step-to-step CoP displacement over a 300-step settled window, under both detectors and with adaptive activation off and on (`370ac803e00`). The legacy FCL lane carries documented wider bounds reflecting its manifold churn. | Met: total-force band, CoP smoothness bound, and activation-on coverage are active regressions. Owner: WP-DB.02, WP-DB.05. |
| Finger flick | 2,573 total DOFs, 576 +/- 88 simulated DOFs, 39 +/- 6 contacts, 3.9 +/- 3.2 fps, 86 +/- 6% LCP time, 1.7 ms step, stiffness 1.5e4. | No hand/finger soft-contact scene. | Add or defer a hand/finger contact scene with adaptive DOF/contact counts, LCP/collision profiler rows, and launch-direction or CoP correctness gate. Owner: WP-DB.03, WP-DB.08, WP-DB.09. |
| Arm fold | 2,802 total DOFs, 322 +/- 89 simulated DOFs, 33 +/- 10 contacts, 3.5 +/- 1.7 fps, 68 +/- 10% LCP time, 8.3 ms step, stiffness 1e4. | No upper-body self-contact scene. | Add or defer a self-contact soft scene with native-vs-FCL contact parity and solver-time breakdown. Owner: WP-DB.08, WP-DB.09. |
| Pinch-grasp | 1,427 total DOFs, 258 +/- 22 simulated DOFs, 29 +/- 4 contacts, 5.2 +/- 3.2 fps, 85 +/- 6% LCP time, 1.7 ms step, stiffness 1.5e4. | No soft-hand manipulation scene. | Add or defer manipulation contact scene with contact-count stability and object-state checksum. Owner: WP-DB.08, WP-DB.09. |
| Biped walk | 334 total DOFs, 197 +/- 43 simulated DOFs, 16 +/- 3 contacts, 18.5 +/- 4.5 fps, 63 +/- 5% LCP time, 4.0 ms step, stiffness 1e3; LCP solved every 8 SIMBICON steps with a 0.5 ms SIMBICON step. | No soft-contact SIMBICON scene. | Add or defer a soft-foot walking scene with soft-contact contact counts, solver-time share, and deterministic controller outcome. Owner: WP-DB.08, WP-DB.09. |
| LCP initial-point robustness | Contact-force magnitudes remain close when the LCP initial point is all zeros, all ones, or random values. | **Gated via reset proxy (WP-DB.02)**: `BoxedLcpConstraintSolver` exposes no public initial-guess policy hook (investigated and recorded in the test), so `test_SoftDynamics` compares steady contact-force magnitudes between an uninterrupted run and a fresh world after reset/state restoration, agreeing within 5% of system weight under both detectors (`370ac803e00`). | Met at the level the public API allows; a direct initial-guess sweep remains follow-up if a solver hook is ever exposed. Owner: WP-DB.02, WP-DB.05. |
| Flexible rigid foot comparison | Four-link rigid foot is more stable than a simpler rigid foot, but deformable foot remains more stable because contact changes are more continuous. | DART has `atlas_v3_no_head_soft_feet.sdf`, and the Atlas SIMBICON terminal condition recognizes `SoftBodyNode` contact, but the current demo hardcodes the simple rigid-foot asset. The soft-foot asset's test only parses and steps it without ground, controller, perturbation, or comparison assertions. No four-link rigid comparator or paired evidence exists. | Add or defer a same-model comparison among simple rigid, four-link flexible-rigid, and deformable feet with the same controller and seeded perturbations. Record stability/contact metrics and deterministic outcomes. Owner: WP-DB.08, WP-DB.09. |

## Approved-deferral mapping

The maintainer-approved 2026-07-11 list in `decisions.md` covers the
paper-scale volumetric-FEM rows (including the starfish obstacle-escape row),
the SIMBICON/controller rows (including noisy-floor biped), and the named hand
scenes. `soft_worm` and `adaptive_soft_contact` are representative reduced
scenes, not claims of full paper-scale parity.

The flexible rigid-foot versus deformable-foot comparison is not explicitly in
the approved list and remains open. The approved mapping is now preserved in
`docs/design/dart6_deformable_body.md`, and PLAN-622 names this exact remaining
decision so the temporary task folder is no longer its only owner.

## Acceptance rule

Paper parity is not satisfied by matching API names. Each row needs:

- a runnable DART 6 scene or benchmark, or a maintainer-approved deferral in a
  durable owner doc,
- correctness/stability evidence with deterministic state or metric thresholds,
- native-vs-FCL contact and performance evidence for any row that depends on
  deformable collision,
- performance rows for single-core and host-capped multi-core CPU,
- an explicit SIMD-off/SIMD-on result when the row touches vectorizable kernels,
- comparison against the paper number or an approved modern normalized target.
