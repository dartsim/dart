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

## Kim and Pollard 2011 — not DART 6 scope

Kim and Pollard's volumetric FEM characters are not DART 6 work (decided
2026-07-29; see `decisions.md`). Their paper numbers remain in the
[paper targets](../../background/deformable_body_paper_targets.md), the removed
implementation remains in [PR #3404](https://github.com/dartsim/dart/pull/3404),
and they are excluded from the acceptance rule below.

## Jain and Liu 2011

The paper target is closer to DART 6 `SoftBodyNode`: surface point masses
attached to rigid bodies, vertex and edge springs, adaptive active vertices near
contact, and mixed LCP/friction soft contact.

| Representative target | Paper number or feature to replicate | Current DART 6 evidence | Acceptance gate |
| --- | --- | --- | --- |
| Adaptive active vertices | Only vertices in a contact-local p-ring are simulated until they leave contact and return to rest; inactive point masses still contribute their rest-pose mass and inertia to the parent rigid body. | **Implemented**: opt-in per-soft-body activation with constraint-time seeding, ring expansion, rest-gated linger, and rigid-lump frozen points contributing full rest mass/inertia to the parent body. Deterministic active-DOF counts (`getNumActivePointMasses`), thread-invariant hashes, activation-enabled zero-allocation gates, and contact-behavior tolerance tests landed in `578ea17a049` + `2382b971244`; the `adaptive_soft_contact` demo visualizes the active region. | Met: contact-local activation with deterministic active-DOF counts, contact behavior within tolerance (`adaptiveContactActivationMatchesSoftDropBehavior`, CoP gate with activation on), and no post-warmup heap growth. |
| Biped push recovery | Same SIMBICON-style controller with rigid contact vs soft contact; soft contact withstands larger perturbations and motor noise while maintaining more ground contact points. | PR #3408 added the soft-foot SIMBICON regression with the models normalized to equal mass, one foot collision surface, and the same rest-pose collision tessellation; the follow-up matched the control's rest inertia (the unmatched tensor was worth 4000 N of its threshold) and re-tuned the asset's under-damped feet (<damp> 1000 -> 4000, kv unchanged, chosen from the interior of a measured 18000 N basin). **Only the contact claim reproduces** and is gate-asserted: 51.2 soft vs 15.64 rigid contacts (3.3x, enforced at 1.5x). The push claim does not: the soft biped falls from the 6000 and 8000 N pushes the rigid control recovers from. The former 18000 N soft maximum was one isolated recovery above falls from 6000-16000 N (#3431) and disappears under Eigen 5's summation order (#3834), so the push sweep reports per-push outcomes without asserting soft >= rigid. | Both decisions (plus a review-found third confound: the SIMBICON COM signal was blind to point masses, fixed in State) resolved 2026-08-01 with A/B evidence; `decisions.md` records them under the soft-foot comparison, and the push comparison is measured but not asserted until the model reproduces it; #3431 found asset tuning alone does not. Motor-noise variant still unbuilt. |
| Noisy-floor biped | Floor segmented into 5 x 5 cm tiles; vertices receive random offsets uniformly sampled from 0-2 cm in vertical and horizontal positions; soft character stays on course in cases where rigid character wanders or falls. | No tracked noisy-floor soft contact scene. | Add deterministic seeded noisy-floor scene with rigid-vs-soft outcome, contact count, CoP trace, and 1-thread/multi-thread rows. |
| Center of pressure and force variance | Soft contact produces smoother contact transitions and lower force variation; pinch-grasp figure shows soft fingers maintaining many contacts while rigid contact fluctuates. | **Instrumented and gated**: `test_SoftDynamics` now bounds total vertical contact force around system weight and step-to-step CoP displacement over a 300-step settled window, under both detectors and with adaptive activation off and on (`370ac803e00`). The legacy FCL lane carries documented wider bounds reflecting its manifold churn. | Met: total-force band, CoP smoothness bound, and activation-on coverage are active regressions. |
| Finger flick | 2,573 total DOFs, 576 +/- 88 simulated DOFs, 39 +/- 6 contacts, 3.9 +/- 3.2 fps, 86 +/- 6% LCP time, 1.7 ms step, stiffness 1.5e4. | No hand/finger soft-contact scene. | Add a hand/finger contact scene with adaptive DOF/contact counts, LCP/collision profiler rows, and launch-direction or CoP correctness gate. |
| Arm fold | 2,802 total DOFs, 322 +/- 89 simulated DOFs, 33 +/- 10 contacts, 3.5 +/- 1.7 fps, 68 +/- 10% LCP time, 8.3 ms step, stiffness 1e4. | No upper-body self-contact scene. | Add a self-contact soft scene with `dart`-versus-FCL contact parity and solver-time breakdown. |
| Pinch-grasp | 1,427 total DOFs, 258 +/- 22 simulated DOFs, 29 +/- 4 contacts, 5.2 +/- 3.2 fps, 85 +/- 6% LCP time, 1.7 ms step, stiffness 1.5e4. | No soft-hand manipulation scene. | Add manipulation contact scene with contact-count stability and object-state checksum. |
| Biped walk | 334 total DOFs, 197 +/- 43 simulated DOFs, 16 +/- 3 contacts, 18.5 +/- 4.5 fps, 63 +/- 5% LCP time, 4.0 ms step, stiffness 1e3; LCP solved every 8 SIMBICON steps with a 0.5 ms SIMBICON step. | `soft_foot_simbicon` (#3408/#3423) is a soft-contact SIMBICON scene, but it walks in place; the walking gait with the paper's LCP cadence, contact counts, solver-time share, and deterministic outcome remains unbuilt. | Add a soft-foot walking scene with soft-contact contact counts, solver-time share, and deterministic controller outcome. |
| LCP initial-point robustness | Contact-force magnitudes remain close when the LCP initial point is all zeros, all ones, or random values. | **Gated via reset proxy**: `BoxedLcpConstraintSolver` exposes no public initial-guess policy hook (investigated and recorded in the test), so `test_SoftDynamics` compares steady contact-force magnitudes between an uninterrupted run and a fresh world after reset/state restoration, agreeing within 5% of system weight under both detectors (`370ac803e00`). | Met at the level the public API allows; a direct initial-guess sweep remains follow-up if a solver hook is ever exposed. |
| Flexible rigid foot comparison | Four-link rigid foot is more stable than a simpler rigid foot, but deformable foot remains more stable because contact changes are more continuous. | #3408/#3423 landed the two-way baseline this row extends: `soft_foot_simbicon` runs the same SIMBICON controller with rigid or deformable feet on ground under scheduled pushes, and `SoftFootSimbiconModelTest` gates that comparison (equal mass/COM/inertia, same collision tessellation, contact spreading) and measures push recovery without gating it. No four-link flexible-rigid comparator or three-way paired evidence exists yet. | Add the four-link flexible-rigid foot variant to the existing comparison — three-way, same controller and seeded perturbations. Record stability/contact metrics and deterministic outcomes. |

## Deferral mapping — retracted

The 2026-07-11 maintainer-approved deferral list is **retracted**. Nothing in
this matrix is deferred any more; each row is either active DART 6 work or out of
DART 6 scope:

- **Kim/Pollard rows are out of DART 6 scope** (decided 2026-07-29) and are
  excluded from the DART 6 completion rule.
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
