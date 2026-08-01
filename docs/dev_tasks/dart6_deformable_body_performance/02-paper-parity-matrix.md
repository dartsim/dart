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

## Kim and Pollard 2011 — retargeted to DART 7, not DART 6 scope

> **These rows are not DART 6 work (decided 2026-07-29).** The volumetric FEM
> lane was removed from `release-6.20` and retargeted to DART 7; see
> `docs/design/dart6_deformable_body.md`. They are retained here as the paper
> ledger, not as DART 6 acceptance targets, and are excluded from the DART 6
> completion rule.


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
| Self-propelled starfish | Two-way coupled skeleton, deformable body, and environment interaction. Jump-turning row: 260 total DOFs, 26 skeleton DOFs, 78 volumetric nodes, 200 volumetric elements, 1,162 surface nodes, 1.0 ms step size, 0.49 s compute per 1 s CPU simulation. Escaping-obstacle row: 0.94 s compute per 1 s CPU simulation, with more than 40% of time in simple point-triangle collision checking. | No volumetric FEM backend or equivalent obstacle scene. Current DART-owned soft collision work covers only DART 6 point-mass `SoftMeshShape` contact lanes. | Add or defer a two-way soft character/obstacle benchmark with collision-time breakdown. The `dart` detector must match or beat FCL on the same scene before it is marked preferred. Owner: WP-DB.04, WP-DB.08, WP-DB.09. |
| Fish jump | Internally actuated skeleton-driven deformable body with gravity and contact only during the freefall/landing phase. Paper row: 258 total DOFs, 9 skeleton DOFs, 107 volumetric nodes, 415 volumetric elements, 958 surface nodes, 1.0 ms step size, 0.50 s compute per 1 s CPU simulation. | No equivalent DART 6 scene or controller. | Add or defer an actuated soft-chain/soft-character landing scene with contact count, final-state checksum, and 1-thread/multi-thread CPU rows. Owner: WP-DB.08, WP-DB.09. |
| Worm roll | Internally actuated rolling worm with large deformation near skeleton joints and ground contact. Paper row: 543 total DOFs, 9 skeleton DOFs, 224 volumetric nodes, 714 volumetric elements, 262 surface nodes, 1.0 ms step size, 0.57 s compute per 1 s CPU simulation. | **Representative scene landed (WP-DB.09)**: the `dart-demos` scene `soft_worm` is an internally actuated five-link chain carrying point-mass flesh (not volumetric FEM) that crawled 1.64 m over 3000 deterministic verification steps through soft ground contact, with a position checksum and finite-state output. Paper-scale volumetric deformation remains deferred per `decisions.md`. | Representative bar met via `soft_worm`; merged evidence is summarized in `verification.md`. Volumetric parity is explicitly out of DART 6 scope. Owner: WP-DB.08, WP-DB.09. |
| Obstacle escape | Contact-heavy two-way starfish scene where collision checking dominates runtime on the paper implementation. | No soft-body contact benchmark with paper-scale contact metrics or obstacle interaction. | Add a contact-heavy `dart`/FCL comparison that records contacts, collision profiler rows, and deterministic checksums; `dart` must be faster without unsupported-pair warnings. Owner: WP-DB.05, WP-DB.07, WP-DB.08. |
| CPU/GPU scaling | Paper reports CPU near-real-time for the smaller characters and GPU acceleration for Fatman; selective diagonalization speedups ranged from about 1.16x to 3.60x depending on mesh and SVD implementation. | DART 6 stays CPU-only. The branch carries scalar/cache slices, allocation gates, and soft-phase/collision profiler scopes. Retained SoA scratch was implemented and **rejected on measurement** (mirror copies cost more than the pointer-chasing they removed) and contiguous point-mass storage is parked with evidence, so no point-mass SIMD kernel landed; both dispositions are recorded in `04-data-layout-and-memory-hardening.md`. Multi-thread rows for every representative scene come from the benchmark matrix (threads 1 and 16). | CPU-only maintained; SoA/SIMD line closed with measured negative results rather than a kernel; matrix rows cover 1 and 16 threads per scene. Owner: WP-DB.06, WP-DB.07. |

## Jain and Liu 2011

The paper target is closer to DART 6 `SoftBodyNode`: surface point masses
attached to rigid bodies, vertex and edge springs, adaptive active vertices near
contact, and mixed LCP/friction soft contact.

| Representative target | Paper number or feature to replicate | Current DART 6 evidence | Acceptance gate |
| --- | --- | --- | --- |
| Adaptive active vertices | Only vertices in a contact-local p-ring are simulated until they leave contact and return to rest; inactive point masses still contribute their rest-pose mass and inertia to the parent rigid body. | **Implemented (WP-DB.05)**: opt-in per-soft-body activation with constraint-time seeding, ring expansion, rest-gated linger, and rigid-lump frozen points contributing full rest mass/inertia to the parent body. Deterministic active-DOF counts (`getNumActivePointMasses`), thread-invariant hashes, activation-enabled zero-allocation gates, and contact-behavior tolerance tests landed in `578ea17a049` + `2382b971244`; the `adaptive_soft_contact` demo visualizes the active region. | Met: contact-local activation with deterministic active-DOF counts, contact behavior within tolerance (`adaptiveContactActivationMatchesSoftDropBehavior`, CoP gate with activation on), and no post-warmup heap growth. Owner: WP-DB.05, WP-DB.06. |
| Biped push recovery | Same SIMBICON-style controller with rigid contact vs soft contact; soft contact withstands larger perturbations and motor noise while maintaining more ground contact points. | #3408/#3423 built the matched comparison (equal mass, one collision surface per foot, same rest tessellation, matched rest inertia, point-mass-aware COM sensor, re-tuned damping). **Contact spreading reproduces and is gate-asserted**: 51.2 soft vs 15.64 rigid (3.3x, enforced at 1.5x). **The push ordering does not reproduce robustly**: ensemble re-measurement (2026-08-01, 5 deterministic replicas per magnitude with strided push-arrival phases, prefix-threshold semantics) shows the previously reported soft 18000 N was an isolated resonance pocket that shrivels to 1/5 under phase sampling; robust thresholds (rigid vs soft) are 8000 vs 4000 N clean, 4000 vs 4000 under 20% held motor noise, 14000 vs 2000 on the paper's 2 cm floor -- no configuration shows a soft advantage. Gates publish the response curves and protect the measured floors. | **Open gap**: the paper's soft-advantage push/noise ordering inverts at shipped parameters with the reused rigid-tuned controller. Mechanism suspects (measured so far in `12-pr3a-soft-foot-simbicon.md` robustness section): kv 5e3-1e4 monotonizes the soft response but ceilings at 6000 N; controller adaptation to soft contact and SoftContactConstraint solve quality under impulsive load remain uninvestigated. Owner: WP-DB.03, WP-DB.08, WP-DB.09. |
| Noisy-floor biped | Floor segmented into 5 x 5 cm tiles; vertices receive random offsets uniformly sampled from 0-2 cm in vertical and horizontal positions; soft character stays on course in cases where rigid character wanders or falls. | Deterministic seeded 5 cm tile floor (heights dug 0-2 cm below the plane, splitmix64-seeded, structure-gated for determinism and spread) landed with a robust push-recovery gate at the paper's 2 cm amplitude. **Measured outcome inverts the paper's**: the dug-down tiles key the rigid foot against lateral sliding (robust threshold rises to 14000 N from 8000 flat) while the soft arm collapses to 2000 N; standing alone does not discriminate (both arms idle through 3.2 cm). | **Open gap shared with the push-recovery row** (same mechanism suspects). Still unbuilt for this row: CoP trace and 1-thread/multi-thread rows. Owner: WP-DB.08, WP-DB.09. |
| Center of pressure and force variance | Soft contact produces smoother contact transitions and lower force variation; pinch-grasp figure shows soft fingers maintaining many contacts while rigid contact fluctuates. | **Instrumented and gated (WP-DB.02)**: `test_SoftDynamics` now bounds total vertical contact force around system weight and step-to-step CoP displacement over a 300-step settled window, under both detectors and with adaptive activation off and on (`370ac803e00`). The legacy FCL lane carries documented wider bounds reflecting its manifold churn. | Met: total-force band, CoP smoothness bound, and activation-on coverage are active regressions. Owner: WP-DB.02, WP-DB.05. |
| Finger flick | 2,573 total DOFs, 576 +/- 88 simulated DOFs, 39 +/- 6 contacts, 3.9 +/- 3.2 fps, 86 +/- 6% LCP time, 1.7 ms step, stiffness 1.5e4. | No hand/finger soft-contact scene. | Add a hand/finger contact scene with adaptive DOF/contact counts, LCP/collision profiler rows, and launch-direction or CoP correctness gate. Owner: WP-DB.03, WP-DB.08, WP-DB.09. |
| Arm fold | 2,802 total DOFs, 322 +/- 89 simulated DOFs, 33 +/- 10 contacts, 3.5 +/- 1.7 fps, 68 +/- 10% LCP time, 8.3 ms step, stiffness 1e4. | No upper-body self-contact scene. | Add a self-contact soft scene with `dart`-versus-FCL contact parity and solver-time breakdown. Owner: WP-DB.08, WP-DB.09. |
| Pinch-grasp | 1,427 total DOFs, 258 +/- 22 simulated DOFs, 29 +/- 4 contacts, 5.2 +/- 3.2 fps, 85 +/- 6% LCP time, 1.7 ms step, stiffness 1.5e4. | No soft-hand manipulation scene. | Add manipulation contact scene with contact-count stability and object-state checksum. Owner: WP-DB.08, WP-DB.09. |
| Biped walk | 334 total DOFs, 197 +/- 43 simulated DOFs, 16 +/- 3 contacts, 18.5 +/- 4.5 fps, 63 +/- 5% LCP time, 4.0 ms step, stiffness 1e3; LCP solved every 8 SIMBICON steps with a 0.5 ms SIMBICON step. | No soft-contact SIMBICON scene. | Add a soft-foot walking scene with soft-contact contact counts, solver-time share, and deterministic controller outcome. Owner: WP-DB.08, WP-DB.09. |
| LCP initial-point robustness | Contact-force magnitudes remain close when the LCP initial point is all zeros, all ones, or random values. | **Gated via reset proxy (WP-DB.02)**: `BoxedLcpConstraintSolver` exposes no public initial-guess policy hook (investigated and recorded in the test), so `test_SoftDynamics` compares steady contact-force magnitudes between an uninterrupted run and a fresh world after reset/state restoration, agreeing within 5% of system weight under both detectors (`370ac803e00`). | Met at the level the public API allows; a direct initial-guess sweep remains follow-up if a solver hook is ever exposed. Owner: WP-DB.02, WP-DB.05. |
| Flexible rigid foot comparison | Four-link rigid foot is more stable than a simpler rigid foot, but deformable foot remains more stable because contact changes are more continuous. | DART has `atlas_v3_no_head_soft_feet.sdf`, and the Atlas SIMBICON terminal condition recognizes `SoftBodyNode` contact, but the current demo hardcodes the simple rigid-foot asset. The soft-foot asset's test only parses and steps it without ground, controller, perturbation, or comparison assertions. No four-link rigid comparator or paired evidence exists. | Add a same-model comparison among simple rigid, four-link flexible-rigid, and deformable feet with the same controller and seeded perturbations. Record stability/contact metrics and deterministic outcomes. Owner: WP-DB.08, WP-DB.09. |

## Deferral mapping — retracted

The 2026-07-11 maintainer-approved deferral list is **retracted**. Nothing in
this matrix is deferred any more; each row is either active DART 6 work or out of
DART 6 scope:

- **Kim/Pollard rows are out of DART 6 scope**, retargeted to DART 7 on
  2026-07-29. They are retained above as the paper ledger and are excluded from
  the DART 6 completion rule.
- **Every Jain/Liu row is active DART 6 work**, including the SIMBICON and
  controller rows (biped push recovery, noisy floor, biped walk), the hand scenes
  (finger flick, arm fold, pinch grasp), and the four-link flexible-rigid-foot
  versus deformable-foot comparison. Their deferrals were retracted on
  2026-07-23.

`soft_worm` and `adaptive_soft_contact` remain representative reduced scenes and
are not claims of full paper-scale parity.

The durable owner of this scope is `docs/design/dart6_deformable_body.md`, so it
survives the retirement of this temporary task folder.

## Acceptance rule

This rule applies to the **Jain/Liu rows only**. The Kim/Pollard rows are out of
DART 6 scope and are not acceptance targets here; do not apply the rule to them.
Deferral is no longer an available outcome for any row.

Parity is not satisfied by matching API names. Each active Jain/Liu row needs:

- a runnable DART 6 scene or benchmark,
- correctness/stability evidence with deterministic state or metric thresholds,
- `dart`-versus-FCL contact and performance evidence for any row that depends on
  deformable collision,
- performance rows for single-core and host-capped multi-core CPU,
- an explicit SIMD-off/SIMD-on result when the row touches vectorizable kernels,
- comparison against the paper number or an approved modern normalized target.
